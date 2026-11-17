#include "motor_pid.hpp"
#include "main.h"

namespace {
constexpr uint32_t kUpdateMs = 10;
constexpr uint32_t kEncoderStallMs = 300;
// 100% 目标速度对应每个 10 ms 控制周期的编码器计数。实车可在此处统一调速。
constexpr float kTicksAtFullSpeed = 20.0F;
constexpr float kProportionalGain = 3.0F;
constexpr float kIntegralGain = 8.0F;
constexpr float kIntegralLimit = 12.5F;

struct MotorPid {
    TIM_HandleTypeDef timer{};
    uint16_t previous_count = 0;
    float integral = 0.0F;
    uint32_t last_motion_ms = 0;
    int previous_target = 0;
    int target = 0;
    int encoder_ticks = 0;
    bool stalled = false;
    int output = 0;
};

MotorPid controllers[4];
uint32_t last_update_ms = 0;

int magnitude(int value) { return value < 0 ? -value : value; }

float clamp(float value, float lower, float upper) {
    return value < lower ? lower : (value > upper ? upper : value);
}

void configure_encoder(MotorPid &controller, TIM_TypeDef *instance) {
    controller.timer.Instance = instance;
    controller.timer.Init.Prescaler = 0;
    controller.timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    controller.timer.Init.Period = 0xFFFF;
    controller.timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    controller.timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef config{};
    config.EncoderMode = TIM_ENCODERMODE_TI12;
    config.IC1Polarity = TIM_ICPOLARITY_RISING;
    config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC1Prescaler = TIM_ICPSC_DIV1;
    config.IC1Filter = 4;
    config.IC2Polarity = TIM_ICPOLARITY_RISING;
    config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC2Prescaler = TIM_ICPSC_DIV1;
    config.IC2Filter = 4;
    if (HAL_TIM_Encoder_Init(&controller.timer, &config) != HAL_OK ||
        HAL_TIM_Encoder_Start(&controller.timer, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(&controller.timer, 0);
}

void configure_encoder_pins() {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();

    // PA15/PB3 原本是 JTAG 引脚；关闭 JTAG，保留 SWD 下载与调试。
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_AFIO_REMAP_TIM2_PARTIAL_2();

    GPIO_InitTypeDef pins{};
    pins.Mode = GPIO_MODE_INPUT;
    pins.Pull = GPIO_PULLUP;
    pins.Speed = GPIO_SPEED_FREQ_HIGH;

    pins.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &pins); // M1: TIM2_CH1
    pins.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &pins); // M1: TIM2_CH2
    pins.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &pins); // M2: TIM3_CH1/CH2
    pins.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &pins); // M3: TIM5_CH1/CH2
    pins.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &pins); // M4: TIM4_CH1/CH2
}

int update_controller(MotorPid &controller, int target, uint32_t now_ms) {
    const uint16_t count = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&controller.timer));
    const int ticks = magnitude(static_cast<int16_t>(count - controller.previous_count));
    controller.previous_count = count;
    controller.target = target;
    controller.encoder_ticks = ticks;

    if (target == 0) {
        controller.integral = 0.0F;
        controller.previous_target = 0;
        controller.output = 0;
        controller.stalled = false;
        controller.last_motion_ms = now_ms;
        return 0;
    }
    if (controller.previous_target == 0 ||
        (target < 0) != (controller.previous_target < 0)) {
        // 从停车起步或换向时，重新开始堵转计时和积分。
        controller.integral = 0.0F;
        controller.last_motion_ms = now_ms;
        controller.stalled = false;
    }
    controller.previous_target = target;
    if (ticks != 0) {
        controller.last_motion_ms = now_ms;
    }
    if (now_ms - controller.last_motion_ms >= kEncoderStallMs) {
        controller.integral = 0.0F;
        controller.output = 0;
        controller.stalled = true;
        return 0;
    }

    const float desired_ticks = magnitude(target) * kTicksAtFullSpeed / 100.0F;
    const float error = desired_ticks - ticks;
    const float candidate_integral = clamp(
        controller.integral + error * (static_cast<float>(kUpdateMs) / 1000.0F),
        -kIntegralLimit, kIntegralLimit);
    const float requested = kProportionalGain * error + kIntegralGain * candidate_integral;
    const float output = clamp(requested, 0.0F, 100.0F);
    if (requested == output) {
        controller.integral = candidate_integral;
    }
    controller.output = static_cast<int>(output + 0.5F);
    controller.stalled = false;
    return target < 0 ? -controller.output : controller.output;
}
} // namespace

void motor_pid_init() {
    configure_encoder_pins();
    configure_encoder(controllers[0], TIM2); // M1: PA15/PB3
    configure_encoder(controllers[1], TIM3); // M2: PA6/PA7
    configure_encoder(controllers[2], TIM5); // M3: PA0/PA1
    configure_encoder(controllers[3], TIM4); // M4: PB6/PB7
    const uint32_t now_ms = HAL_GetTick();
    for (auto &controller : controllers) {
        controller.previous_count = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&controller.timer));
        controller.last_motion_ms = now_ms;
    }
    last_update_ms = now_ms - kUpdateMs;
}

DriveCommand motor_pid_update(const DriveCommand &target, uint32_t now_ms) {
    DriveCommand output{};
    if (now_ms - last_update_ms < kUpdateMs) {
        for (unsigned i = 0; i < 4; ++i) {
            output.motor[i] = target.motor[i] == 0 ? 0 :
                              (target.motor[i] < 0 ? -controllers[i].output : controllers[i].output);
        }
        return output;
    }
    last_update_ms = now_ms;
    for (unsigned i = 0; i < 4; ++i) {
        output.motor[i] = update_controller(controllers[i], target.motor[i], now_ms);
    }
    return output;
}

MotorPidStatus motor_pid_status(unsigned motor) {
    if (motor >= 4) {
        return {};
    }
    const MotorPid &controller = controllers[motor];
    return {controller.target, controller.encoder_ticks, controller.output, controller.stalled};
}
