#pragma once

#include "ps2_protocol.hpp"

struct ArmCommand {
    // 角度 0~180（90 为中位），输出层换算成 500~2500us 脉宽。
    int servo[4] = {};
};

ArmCommand arm_from_ps2(const Ps2Frame &frame);
// 急停时调用，返回安全收车姿态
ArmCommand arm_emergency_stop();

// R3（按下右摇杆）切换“全部舵机停止”，自锁。
// true 时 outputs 停发 PWM，舵机不出力、不发热，卡住时按一下保命。
bool arm_servos_released();