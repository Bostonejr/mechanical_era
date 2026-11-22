#include "outputs.hpp"
#include "main.h"
#include "motor_pid.hpp"
#include <algorithm>

extern "C" TIM_HandleTypeDef htim1;
extern "C" TIM_HandleTypeDef htim8;

namespace {
struct Motor {
    GPIO_TypeDef *a_port;
    uint16_t a_pin;
    GPIO_TypeDef *b_port;
    uint16_t b_pin;
    uint32_t channel;
};

const Motor kMotors[] = {
    {motor1_A_GPIO_Port, motor1_A_Pin, motor1_B_GPIO_Port, motor1_B_Pin, TIM_CHANNEL_1},
    {motor2_A_GPIO_Port, motor2_A_Pin, motor2_B_GPIO_Port, motor2_B_Pin, TIM_CHANNEL_2},
    {motor3_A_GPIO_Port, motor3_A_Pin, motor3_B_GPIO_Port, motor3_B_Pin, TIM_CHANNEL_3},
    {motor4_A_GPIO_Port, motor4_A_Pin, motor4_B_GPIO_Port, motor4_B_Pin, TIM_CHANNEL_4}};

constexpr uint32_t servo_channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

int previous_motor_duty[4] = {};
uint32_t reversal_ready_ms[4] = {};
constexpr uint32_t kReversalDeadTimeMs = 2;
}

void apply_servos(ArmCommand command, bool enabled, uint16_t shoulder_speed_us) {
    // R3 自锁释放：CCR=0，输出持续低电平，不发送舵机脉冲，舵机卸力。
    if (!enabled || arm_servos_released()) {
        for (const auto channel : servo_channels) {
            __HAL_TIM_SET_COMPARE(&htim8, channel, 0);
        }
        return;
    }

    // 每路允许的物理行程（µs）。180° 舵机：500=0°、1500=90°、2500=180°。
    // 标定时收到“机械限位前一点点”，舵机就不会顶死堵转发烫。
    static const int kMinUs[4] = {600, 600, 600, 600};
    static const int kMaxUs[4] = {2400, 2400, 2400, 2400};
    for (unsigned i = 0; i < 4; ++i) {
        if (servo_channels[i] == TIM_CHANNEL_2 && kShoulderContinuousRotation) {
            // 连续旋转舵机接收速度脉宽，位置预设不能驱动它。
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, shoulder_speed_us);
            continue;
        }
        // ArmCommand.servo 是角度 0~180（90=中位），不是百分比。
        int degrees = command.servo[i];
        if (degrees < 0) degrees = 0;
        if (degrees > 180) degrees = 180;
        int pulse_us = 500 + degrees * 2000 / 180;
        if (pulse_us < kMinUs[i]) pulse_us = kMinUs[i];
        if (pulse_us > kMaxUs[i]) pulse_us = kMaxUs[i];
        __HAL_TIM_SET_COMPARE(&htim8, servo_channels[i], static_cast<uint32_t>(pulse_us));
    }
}

void apply_motors(DriveCommand command) {
    const uint32_t now_ms = HAL_GetTick();
    command = motor_pid_update(command, now_ms);
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
    const uint32_t full_scale = period + 1U;

    for (unsigned i = 0; i < 4; ++i) {
        const Motor &motor = kMotors[i];
        const int duty = std::max(-100, std::min(100, command.motor[i]));

        // 刚清零的通道也要留出换向间隔，避免闭环先给零、下一圈立即反向。
        if (previous_motor_duty[i] != 0 &&
            (duty == 0 || duty * previous_motor_duty[i] < 0)) {
            __HAL_TIM_SET_COMPARE(&htim1, motor.channel, 0);
            HAL_GPIO_WritePin(motor.a_port, motor.a_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor.b_port, motor.b_pin, GPIO_PIN_RESET);
            previous_motor_duty[i] = 0;
            reversal_ready_ms[i] = now_ms + kReversalDeadTimeMs;
            continue;
        }

        // now_ms + 2 可能回绕，用有符号差值判断剩余死区时间。
        if (static_cast<int32_t>(now_ms - reversal_ready_ms[i]) < 0) {
            continue;
        }

        if (duty == previous_motor_duty[i]) {
            continue;
        }
        HAL_GPIO_WritePin(motor.a_port, motor.a_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor.b_port, motor.b_pin, GPIO_PIN_RESET);
        if (duty != 0) {
            HAL_GPIO_WritePin(duty > 0 ? motor.a_port : motor.b_port,
                              duty > 0 ? motor.a_pin : motor.b_pin, GPIO_PIN_SET);
        }
        const uint32_t magnitude = static_cast<uint32_t>(duty < 0 ? -duty : duty);
        const uint32_t scaled = (full_scale * magnitude) / 100U;
        const uint32_t compare = scaled > period ? period : scaled;
        __HAL_TIM_SET_COMPARE(&htim1, motor.channel, compare);
        previous_motor_duty[i] = duty;
    }
}

void init_outputs() {
    motor_pid_init();
    // 先写入停止状态，再启动 PWM
    for (unsigned i = 0; i < 4; ++i) {
        const auto &motor = kMotors[i];
        __HAL_TIM_SET_COMPARE(&htim1, motor.channel, 0);
        HAL_GPIO_WritePin(motor.a_port, motor.a_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor.b_port, motor.b_pin, GPIO_PIN_RESET);
        previous_motor_duty[i] = 0;
        reversal_ready_ms[i] = 0;
    }
    for (const Motor &motor : kMotors) {
        if (HAL_TIM_PWM_Start(&htim1, motor.channel) != HAL_OK) {
            Error_Handler();
        }
    }
    for (const auto channel : servo_channels) {
        __HAL_TIM_SET_COMPARE(&htim8, channel, 0);
        if (HAL_TIM_PWM_Start(&htim8, channel) != HAL_OK) {
            Error_Handler();
        }
    }
}
