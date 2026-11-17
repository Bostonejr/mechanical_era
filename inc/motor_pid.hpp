#pragma once

#include "drive_control.hpp"
#include <cstdint>

// 四路 AB 相编码器速度闭环。输入和输出均为 -100..100 的 PWM 百分比。
void motor_pid_init();
DriveCommand motor_pid_update(const DriveCommand &target, uint32_t now_ms);

struct MotorPidStatus {
    int target = 0;
    int encoder_ticks = 0;
    int output = 0;
    bool stalled = false;
};

MotorPidStatus motor_pid_status(unsigned motor);
