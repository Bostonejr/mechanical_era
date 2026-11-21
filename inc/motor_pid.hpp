#pragma once

#include "drive_control.hpp"
#include <cstdint>

// 四路 AB 相编码器速度闭环。输入和输出均为 -100..100 的 PWM 百分比。
// encoder_ticks 为有符号计数：正值表示正转，负值表示反转；
// 每轮方向在 motor_pid.cpp 的 kEncoderSign 中标定。
void motor_pid_init();
DriveCommand motor_pid_update(const DriveCommand &target, uint32_t now_ms);

struct MotorPidStatus {
    int target = 0;
    int encoder_ticks = 0; // 有符号：正=正转，负=反转
    int output = 0;        // 幅值 0..100，方向由 target 决定
    bool stalled = false;
};

MotorPidStatus motor_pid_status(unsigned motor);
