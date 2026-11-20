#pragma once

#include "ps2_protocol.hpp"

// Latches on when Select+Start is held for 2 s within the boot window (first 5 s
// after reset). While active, service features are enabled: Triangle wheel
// self-test and Start+D-pad single-motor calibration. Competition buttons stay
// free in normal mode.
// 复位后的开机窗口期（前 5 秒）内，持续按住 Select+Start 达 2 秒即锁定进入维修模式。
// 维修模式激活时启用检修功能：Triangle 单轮自检、Start+方向键 单电机标定。
// 正常模式下比赛按键保持可用。

// Called once per main loop with the latest frame. Must run before any code
// that checks service_mode_active().
// 每次主循环调用一次，传入最新的手柄帧。必须先于所有读取
// service_mode_active() 的代码执行。
void service_mode_update(const Ps2Frame &frame);

// True once service mode has latched on; never turns back off until reset.
// 维修模式锁定后返回 true；复位前不会解除。
bool service_mode_active();
