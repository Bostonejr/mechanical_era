#pragma once

#include "ps2_protocol.hpp"

struct ArmCommand {
    // 行程百分比 -100..100，0 为中位；硬件层换算成舵机脉宽。
    int servo[4] = {};
};

ArmCommand arm_from_ps2(const Ps2Frame &frame);
// 【新增】供 main.cpp 急停时调用，返回安全收车姿态
ArmCommand arm_emergency_stop();
