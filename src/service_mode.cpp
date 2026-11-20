#include "service_mode.hpp"
#include "main.h"

namespace {
// Boot window: service mode can only be entered shortly after reset, so a
// driver can never trigger it mid-match by accident.
// 开机窗口：维修模式只能在复位后短时间内进入，避免比赛过程中误触发。
constexpr uint32_t kBootWindowMs = 5000;
// How long Select+Start must be held continuously to latch service mode.
// Select+Start 需要持续按住的时间，达到后才锁定维修模式。
constexpr uint32_t kHoldMs = 2000;

bool active = false;
bool initialized = false;
bool holding = false;
uint32_t boot_ms = 0;
uint32_t hold_start_ms = 0;
}

void service_mode_update(const Ps2Frame &frame) {
    if (active) {
        return;
    }
    const uint32_t now = HAL_GetTick();
    if (!initialized) {
        initialized = true;
        boot_ms = now;
    }
    if (now - boot_ms > kBootWindowMs) {
        // Boot window expired; service mode stays off until the next reset.
        // 开机窗口已过；下次复位前保持正常模式。
        return;
    }
    const bool both = ps2_pressed(frame, Ps2Button::Select) && ps2_pressed(frame, Ps2Button::Start);
    if (!both) {
        // Releasing either button restarts the hold timer.
        // 松开任意一个按键都会重新计时。
        holding = false;
        return;
    }
    if (!holding) {
        holding = true;
        hold_start_ms = now;
    } else if (now - hold_start_ms >= kHoldMs) {
        active = true;
    }
}

bool service_mode_active() { return active; }
