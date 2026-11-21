#include "fh3mod.h"

#include <windows.h>
#include <Xinput.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <mutex>
#include <unordered_map>

namespace {

constexpr unsigned player = 1;
constexpr unsigned others = 2;
constexpr unsigned all = player | others;
constexpr int min_bpm = 40;
constexpr int max_bpm = 240;
constexpr wchar_t ini_section[] = L"Settings";

struct Target {
    const wchar_t* stick_key;
    const char* stick_setting;
    const char* stick_label;
    unsigned mask;
    std::atomic<int> stick{0};
};

Target targets[] = {
    {L"PlayerStick", "PlayerStick", "Player stick binding", player},
    {L"OthersStick", "OthersStick", "Other cars stick binding", others},
    {L"AllStick", "AllStick", "All cars stick binding", all},
};

const FH3ModApi* platform = nullptr;
FH3Mod* mod = nullptr;
std::wstring ini_path;
std::atomic<int> bpm{120};
std::atomic<bool> hold_mode{false};

// One atomic snapshot keeps the active scopes and samples for each car kind
// together when the controller reader and multiple update callers overlap.
constexpr unsigned sampled_shift = 2;
std::atomic<unsigned> bounce_state{0};

using CarUpdate = void (*)(void*, float, float, int);
using CarKind = int (*)(void*);
CarUpdate original_update = nullptr;
CarKind car_kind = nullptr;

// June 5, 2018 Xbox One executable as mapped by Azimuth.
constexpr unsigned game_timestamp = 0x5B16ECCA;
constexpr unsigned game_image_size = 0x5495000;
constexpr unsigned car_update_rva = 0x3442A0;
constexpr char car_update_bytes[] = "\x48\x89\x5C\x24\x18\x57\x48\x81\xEC\x80\x00\x00\x00";
constexpr unsigned car_kind_rva = 0x188E430;
constexpr char car_kind_bytes[] = "\x8B\x81\x70\x73\x00\x00\xC3";

// ForzaCar holds the deferred ICarPresentation interface. The next word
// is the concrete presentation targeted by deferred render commands.
constexpr unsigned car_presentation_offset = 0x73A0;
constexpr unsigned deferred_interface_vtable_rva = 0x468B9B8;
constexpr unsigned body_submit_rva = 0x1E32C00;
constexpr char body_submit_bytes[] = "\xE8\x7B\xCD\x3C\x00";
// The driver pose update receives its own unscaled copy of the car matrix.
constexpr unsigned driver_pose_rva = 0x1E33F73;
constexpr char driver_pose_bytes[] = "\xE8\xC8\xA2\x03\x00";
constexpr unsigned transform_sample_shift = 5;
constexpr unsigned driver_sample_shift = 10;
const void* deferred_interface_vtable = nullptr;
constexpr unsigned body_call_seen = 1u << 8;
constexpr unsigned owner_missing_seen = 1u << 9;

struct RenderOwner {
    int kind;
    ULONGLONG last_update;
};
std::mutex owners_mutex;
std::unordered_map<const void*, RenderOwner> render_owners;
ULONGLONG next_prune = 0;

float body_scale(ULONGLONG now) {
    const unsigned tempo = static_cast<unsigned>(bpm.load());
    const auto phase = ((now % 60000) * tempo) % 60000;
    const double wave = std::sin(6.283185307179586 * static_cast<double>(phase) / 60000.0);
    return static_cast<float>(1.0 + wave * (wave >= 0.0 ? 0.23 : 0.17));
}

bool mark_once(unsigned bit) {
    unsigned state = bounce_state.load();
    while ((state & all) && !(state & bit)) {
        if (bounce_state.compare_exchange_weak(state, state | bit)) return true;
    }
    return false;
}

void scale_model_matrix(FH3Context* context, const void* presentation, bool driver) {
    unsigned state = bounce_state.load();
    if (!(state & all) || platform->playing_online()) return;
    if (!driver && mark_once(body_call_seen)) platform->log(mod, "Reached car presentation body transform.");

    const auto now = GetTickCount64();
    int kind = -1;
    {
        std::lock_guard lock(owners_mutex);
        auto owner = render_owners.find(presentation);
        if (owner == render_owners.end()) {
            // The deferred target can use car presentation's secondary interface.
            owner = render_owners.find(static_cast<const unsigned char*>(presentation) + 8);
        }
        if (owner != render_owners.end() && now - owner->second.last_update <= 250) {
            kind = owner->second.kind;
        }
    }
    if (kind < 0) {
        if (!driver && mark_once(owner_missing_seen)) platform->log(mod, "car presentation body transform has no recent car owner.");
        return;
    }
    const unsigned scope = kind == 0 ? player : others;
    if (!(state & scope)) return;

    // Pass an aligned copy to the model or pose update. The original matrix
    // remains available to the wheel update and other callers.
    const auto* original = reinterpret_cast<const float*>(context->rdx);
    if (!original || !(std::abs(original[15] - 1.0f) <= 0.001f) ||
        !std::isfinite(original[4]) || !std::isfinite(original[5]) ||
        !std::isfinite(original[6]) || !(std::abs(original[7]) <= 0.001f)) return;

    alignas(16) thread_local float matrices[2][16];
    auto& matrix = matrices[driver ? 1 : 0];
    std::memcpy(matrix, original, sizeof matrix);
    const float scale = body_scale(now);
    matrix[4] *= scale;
    matrix[5] *= scale;
    matrix[6] *= scale;
    context->rdx = reinterpret_cast<uintptr_t>(matrix);

    const unsigned sampled = 1u << (kind + (driver ? driver_sample_shift : transform_sample_shift));
    if (mark_once(sampled)) {
        char line[128];
        std::snprintf(line, sizeof line, "Scaled car presentation %s transform for car kind %d (%.3fx).",
                      driver ? "driver" : "body", kind, scale);
        platform->log(mod, line);
    }
}

void scale_body_submission(FH3Context* context, void*) {
    scale_model_matrix(context, reinterpret_cast<const void*>(context->r15), false);
}

void scale_driver_pose(FH3Context* context, void*) {
    scale_model_matrix(context, reinterpret_cast<const void*>(context->rsi), true);
}

void track_render_owner(const void* car, int kind) {
    const auto* bytes = static_cast<const unsigned char*>(car);
    const auto* deferred = *reinterpret_cast<const void* const* const*>(bytes + car_presentation_offset);
    if (!deferred || deferred[0] != deferred_interface_vtable || !deferred[1]) return;
    const void* concrete = deferred[1];

    const auto now = GetTickCount64();
    std::lock_guard lock(owners_mutex);
    render_owners.insert_or_assign(concrete, RenderOwner{kind, now});
    if (now < next_prune) return;

    next_prune = now + 250;
    for (auto entry = render_owners.begin(); entry != render_owners.end();) {
        if (now - entry->second.last_update > 250) {
            entry = render_owners.erase(entry);
        } else {
            ++entry;
        }
    }
}

void save_number(const wchar_t* key, unsigned value) {
    wchar_t number[16];
    swprintf_s(number, L"%u", value);
    if (!WritePrivateProfileStringW(ini_section, key, number, ini_path.c_str())) {
        platform->log(mod, "Could not save Why's this dealer settings.");
    }
}

void change_bpm(void*, const char*, float value) {
    const int rounded = std::clamp(static_cast<int>(value + 0.5f), min_bpm, max_bpm);
    bpm.store(rounded);
    save_number(L"BPM", rounded);
}

void toggle(const Target& target) {
    if (hold_mode.load() || platform->playing_online()) return;

    unsigned previous = bounce_state.load();
    unsigned next;
    do {
        const unsigned enabled = previous & all;
        next = (enabled & target.mask) == target.mask
            ? enabled & ~target.mask
            : enabled | target.mask;
    } while (!bounce_state.compare_exchange_weak(previous, next));

    char line[128];
    std::snprintf(line, sizeof line, "Bounce targets: player %s, others %s (%d BPM).",
                  next & player ? "on" : "off", next & others ? "on" : "off", bpm.load());
    platform->log(mod, line);
}

unsigned held_bindings(uint32_t buttons) {
    unsigned held = 0;
    for (unsigned i = 0; i < 3; ++i) {
        const int stick = targets[i].stick.load();
        if ((stick == 1 && (buttons & XINPUT_GAMEPAD_LEFT_THUMB)) ||
            (stick == 2 && (buttons & XINPUT_GAMEPAD_RIGHT_THUMB))) {
            held |= 1u << i;
        }
    }
    return held;
}

uint32_t controller_buttons() {
    for (DWORD index = 0; index < XUSER_MAX_COUNT; ++index) {
        XINPUT_STATE state{};
        if (XInputGetState(index, &state) == ERROR_SUCCESS) {
            return state.Gamepad.wButtons;
        }
    }
    return 0;
}

DWORD WINAPI poll_controls(void*) {
    unsigned previous = 0;
    bool was_focused = false;
    bool was_holding = false;
    while (!platform->playing_online()) {
        DWORD foreground_pid = 0;
        GetWindowThreadProcessId(GetForegroundWindow(), &foreground_pid);
        const bool focused = foreground_pid == GetCurrentProcessId();
        const bool holding = hold_mode.load();
        const unsigned bindings = focused ? held_bindings(controller_buttons()) : 0;
        if (holding) {
            unsigned held = 0;
            for (unsigned i = 0; i < 3; ++i) {
                if (bindings & (1u << i)) held |= targets[i].mask;
            }
            unsigned state = bounce_state.load();
            while ((state & all) != held &&
                   !bounce_state.compare_exchange_weak(state, held)) {}
        } else if (focused && was_focused && !was_holding) {
            const unsigned pressed = bindings & ~previous;
            if (pressed & (1u << 2)) {
                toggle(targets[2]);
            } else {
                if (pressed & (1u << 0)) toggle(targets[0]);
                if (pressed & (1u << 1)) toggle(targets[1]);
            }
        }
        previous = bindings;
        was_focused = focused;
        was_holding = holding;
        Sleep(16);
    }
    bounce_state.store(0);
    return 0;
}

void change_mode(void*, const char*, int choice) {
    bounce_state.store(0);
    hold_mode.store(choice == 1);
    save_number(L"HoldMode", choice == 1);
}

void change_stick(void* user, const char*, int choice) {
    auto& target = *static_cast<Target*>(user);
    target.stick.store(choice);
    save_number(target.stick_key, choice);
}

void update(void* car, float dt, float second_step, int flags) {
    unsigned state = bounce_state.load();
    int kind = -1;
    if ((state & all) && !platform->playing_online()) {
        kind = car_kind(car);
        if (kind < 0 || kind > 2) kind = -1;
    }

    if (kind >= 0 && dt > 0.0f) track_render_owner(car, kind);
    original_update(car, dt, second_step, flags);

    if (kind < 0) return;
    const unsigned scope = kind == 0 ? player : others;
    const unsigned sampled = 1u << (kind + sampled_shift);
    do {
        if (!(state & scope) || (state & sampled)) return;
    } while (!bounce_state.compare_exchange_weak(state, state | sampled));

    const int tempo = bpm.load();
    char line[160];
    std::snprintf(line, sizeof line,
                  "Observed car kind %d, update dt %.4f, beat period %.3f s at %d BPM.",
                  kind, dt, 60.0 / tempo, tempo);
    platform->log(mod, line);
}

DWORD WINAPI install_bounce(void*) {
    auto* game = static_cast<unsigned char*>(platform->wait_for_code(60000));
    if (!game) {
        platform->log(mod, "Bounce unavailable: game code did not become ready.");
        return 0;
    }
    if (platform->playing_online()) return 0;

    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(game);
    if (dos->e_magic != IMAGE_DOS_SIGNATURE || dos->e_lfanew <= 0 || dos->e_lfanew > 0x1000) {
        platform->log(mod, "Bounce unavailable: invalid FH3 executable header.");
        return 0;
    }
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(game + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE || nt->FileHeader.TimeDateStamp != game_timestamp ||
        nt->OptionalHeader.SizeOfImage != game_image_size) {
        platform->log(mod, "Bounce unavailable: expected the June 5, 2018 FH3 executable.");
        return 0;
    }

    auto* update_site = game + car_update_rva;
    auto* kind_site = game + car_kind_rva;
    if (std::memcmp(update_site, car_update_bytes,
                    sizeof car_update_bytes - 1) ||
        std::memcmp(kind_site, car_kind_bytes,
                    sizeof car_kind_bytes - 1)) {
        platform->log(mod, "Bounce unavailable: car update or kind signature did not match.");
        return 0;
    }

    auto* render_site = game + body_submit_rva;
    if (std::memcmp(render_site, body_submit_bytes,
                    sizeof body_submit_bytes - 1)) {
        platform->log(mod, "Bounce unavailable: body submission signature did not match.");
        return 0;
    }

    deferred_interface_vtable = game + deferred_interface_vtable_rva;
    car_kind = reinterpret_cast<CarKind>(kind_site);
    FH3Hook* classifier = platform->hook_function(mod, "Bounce car classification", update_site,
                                reinterpret_cast<void*>(&update),
                                reinterpret_cast<void**>(&original_update));
    FH3Hook* render = platform->hook_instruction(mod, "Scale car body submission", render_site,
                                                 scale_body_submission, nullptr);
    if (!classifier || !render || !platform->enable_hooks(mod, classifier)) {
        platform->log(mod, "Could not enable the bounce car classification hook.");
        return 0;
    }
    if (!platform->enable_hooks(mod, render)) {
        platform->disable_hooks(mod, classifier);
        platform->log(mod, "Could not enable the bounce render hook.");
        return 0;
    }
    platform->log(mod, "car presentation bounce hooks enabled.");

    auto* driver_site = game + driver_pose_rva;
    if (std::memcmp(driver_site, driver_pose_bytes, sizeof driver_pose_bytes - 1)) {
        platform->log(mod, "Driver bounce unavailable: driver pose signature did not match.");
        return 0;
    }
    FH3Hook* driver = platform->hook_instruction(mod, "Scale seated driver pose", driver_site,
                                                  scale_driver_pose, nullptr);
    if (!driver || !platform->enable_hooks(mod, driver)) {
        platform->log(mod, "Could not enable the seated driver pose hook.");
    }
    return 0;
}

} // namespace

FH3MOD_EXPORT int fh3mod_init(const FH3ModApi* api) {
    if (!api || api->api_version < FH3MOD_API_VERSION || api->size < sizeof(FH3ModApi) ||
        !api->register_mod || !api->log || !api->azimuth_dir || !api->add_setting ||
        !api->playing_online || !api->wait_for_code || !api->hook_function ||
        !api->hook_instruction || !api->enable_hooks || !api->disable_hooks) {
        return 1;
    }

    platform = api;
    FH3ModInfo info{};
    info.size = sizeof info;
    info.name = "FH3-WhysThisDealer";
    info.version = MOD_VERSION;
    info.github_repo = "suderek22/FH3_WhysThisDealer";
    info.check_updates = 1;
    info.description = "Makes car bodies and seated drivers squash and stretch to a beat.";
    info.platform_version = "1.4.2";
    info.offline_only = 1;
    mod = api->register_mod(&info);
    if (!mod) return 1;

    ini_path = std::wstring(api->azimuth_dir) + L"\\fh3-mods\\FH3-WhysThisDealer.ini";
    bpm.store(std::clamp(static_cast<int>(GetPrivateProfileIntW(
        ini_section, L"BPM", 120, ini_path.c_str())), min_bpm, max_bpm));

    FH3Setting tempo{};
    tempo.size = sizeof tempo;
    tempo.key = "BPM";
    tempo.label = "Bounce BPM";
    tempo.description = "Beats per minute for the visual bounce.";
    tempo.kind = FH3_SETTING_SLIDER;
    tempo.min = min_bpm;
    tempo.max = max_bpm;
    tempo.step = 1;
    tempo.slider_value = static_cast<float>(bpm.load());
    tempo.on_slide = change_bpm;
    api->add_setting(mod, &tempo);

    hold_mode.store(GetPrivateProfileIntW(ini_section, L"HoldMode", 0, ini_path.c_str()) == 1);
    static const char* const modes[] = {"Toggle", "Hold"};
    static const char* const sticks[] = {"Off", "LS", "RS"};
    FH3Setting activation{};
    activation.size = sizeof activation;
    activation.key = "ActivationMode";
    activation.label = "Activation mode";
    activation.kind = FH3_SETTING_CHOICE;
    activation.choices = modes;
    activation.choice_count = 2;
    activation.value = hold_mode.load();
    activation.on_change = change_mode;
    api->add_setting(mod, &activation);

    for (Target& target : targets) {
        FH3Setting stick = activation;
        stick.key = target.stick_setting;
        stick.label = target.stick_label;
        stick.choices = sticks;
        stick.choice_count = 3;
        stick.value = std::clamp(static_cast<int>(GetPrivateProfileIntW(
            ini_section, target.stick_key, 0, ini_path.c_str())), 0, 2);
        stick.on_change = change_stick;
        stick.user = &target;
        target.stick.store(stick.value);
        api->add_setting(mod, &stick);
    }

    if (HANDLE thread = CreateThread(nullptr, 0, poll_controls, nullptr, 0, nullptr)) {
        CloseHandle(thread);
    } else {
        api->log(mod, "Could not start the activation input reader.");
    }

    if (HANDLE thread = CreateThread(nullptr, 0, install_bounce, nullptr, 0, nullptr)) {
        CloseHandle(thread);
    } else {
        api->log(mod, "Could not start the visual bounce.");
    }
    return 0;
}
