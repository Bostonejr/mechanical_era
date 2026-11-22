#include "motor_pid.hpp"
#include "main.h"

// External linkage on purpose: GDB/live watch cannot resolve variables inside
// anonymous namespaces, and PID tuning needs live access to controllers.
// 故意使用外部链接：GDB/live watch 无法解析匿名命名空间内的变量，
// 而 PID 整定需要实时访问 controllers。
constexpr unsigned kMotorCount = 4;
struct MotorPid {
    TIM_HandleTypeDef timer{};
    uint16_t previous_count = 0;
    float integral = 0.0F;
    uint32_t last_motion_ms = 0;
    int previous_target = 0;
    int target = 0;
    int encoder_ticks =
        0; // signed: positive = forward, negative = reverse / 有符号：正=正转，负=反转
    int output =
        0; // magnitude 0..100, direction comes from target / 幅值 0..100，方向由 target 决定
    int stalled_target = 0;
    bool stalled = false;
    int8_t encoder_sign = 1;
};
MotorPid controllers[kMotorCount];

namespace {
constexpr int kMaxTargetPercent = 100;
constexpr float kMaxOutput = 100.0F;
constexpr uint32_t kControlPeriodMs = 10;
constexpr uint32_t kMaxDtMs = 50;
constexpr uint32_t kEncoderStallMs = 300;
constexpr int kStallMinTarget = 10;
constexpr int kStallRetryDelta = 10;

// 100% 目标速度对应每秒的编码器计数。当前按 20 ticks / 10 ms 标定，
// 即 2000 ticks/s；实测后只改这一个数。
constexpr float kTicksPerSecondAtFullSpeed = 2000.0F;
constexpr float kProportionalGain = 10.0F;
constexpr float kIntegralGain = 0.01F;
// I 项上限始终保持 100%，改 Ki 时不需要再手算 kIntegralLimit。
constexpr float kIntegralLimit = kMaxOutput / kIntegralGain;

// 每轮编码器方向。目标为正时，串口 PID 行的 k= 必须为正；
// 如果某个轮子的 k= 为负，就把对应项改成 -1。
// 首次上车前必须架空车轮逐个标定，符号错误会形成正反馈并让轮子全速。
constexpr int8_t kEncoderSign[kMotorCount] = {1, -1, 1, -1};

constexpr bool valid_encoder_signs() {
    for (unsigned i = 0; i < kMotorCount; ++i) {
        if (kEncoderSign[i] != 1 && kEncoderSign[i] != -1) {
            return false;
        }
    }
    return true;
}
static_assert(valid_encoder_signs(), "kEncoderSign entries must be +1 or -1");
static_assert(kMotorCount == 4, "DriveCommand and telemetry assume four motors");
static_assert(kIntegralGain > 0.0F, "kIntegralGain must be positive");

uint32_t last_update_ms = 0;

int magnitude(int value) { return value < 0 ? -value : value; }

int clamp_target(int value) {
    if (value > kMaxTargetPercent) {
        return kMaxTargetPercent;
    }
    if (value < -kMaxTargetPercent) {
        return -kMaxTargetPercent;
    }
    return value;
}

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
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();

    // PA15/PB3 原本是 JTAG 引脚；关闭 JTAG，保留 SWD 下载与调试。
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    // M1 编码器接 PA15/PB3，必须用 TIM2 部分重映射 1；
    // PARTIAL_2 会把 CH1/CH2 映射到 PA0/PA1，与 M3 的 TIM5 冲突。
    __HAL_AFIO_REMAP_TIM2_PARTIAL_1();

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

int update_controller(MotorPid &controller, int target, uint32_t now_ms, float dt_s) {
    const uint16_t count = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&controller.timer));
    const int16_t delta = static_cast<int16_t>(count - controller.previous_count);
    controller.previous_count = count;
    controller.encoder_ticks = static_cast<int>(delta) * controller.encoder_sign;
    controller.target = target;

    if (target == 0) {
        controller.integral = 0.0F;
        controller.previous_target = 0;
        controller.output = 0;
        controller.stalled = false;
        controller.stalled_target = 0;
        controller.last_motion_ms = now_ms;
        return 0;
    }

    if (controller.previous_target == 0 || (target < 0) != (controller.previous_target < 0)) {
        // 从停车起步或换向时，重新开始积分和堵转计时。
        controller.integral = 0.0F;
        controller.last_motion_ms = now_ms;
        controller.stalled = false;
        controller.stalled_target = 0;
    }
    controller.previous_target = target;

    // 堵转后，只有轮子重新转动或目标再加大一档才允许重试；
    // 否则保持输出 0，避免对卡死的轮子持续加大电流。
    if (controller.stalled) {
        if (controller.encoder_ticks != 0) {
            controller.stalled = false;
            controller.integral = 0.0F;
            controller.last_motion_ms = now_ms;
        } else if (magnitude(target) >= magnitude(controller.stalled_target) + kStallRetryDelta) {
            controller.stalled = false;
            controller.integral = 0.0F;
            controller.last_motion_ms = now_ms;
        } else {
            controller.output = 0;
            return 0;
        }
    }

    const int direction = target < 0 ? -1 : 1;
    const float desired_ticks =
        static_cast<float>(magnitude(target)) * kTicksPerSecondAtFullSpeed * dt_s / 100.0F;
    // 把实测速度换算到“目标方向为正”的坐标系：
    // 轮子反转时 actual_ticks 为负，误差会变大并立即往前拉。
    const float actual_ticks = static_cast<float>(controller.encoder_ticks * direction);
    const float error = desired_ticks - actual_ticks;

    const float candidate_integral =
        clamp(controller.integral + error * dt_s, -kIntegralLimit, kIntegralLimit);
    const float requested = kProportionalGain * error + kIntegralGain * candidate_integral;

    // 只有目标足够大且控制器确实在输出时才判堵转；
    // 低速蠕动、滑行和反转不会被误判为堵转。
    const bool driving = magnitude(target) >= kStallMinTarget && requested > 0.0F;
    if (!driving || controller.encoder_ticks != 0) {
        controller.last_motion_ms = now_ms;
    } else if (now_ms - controller.last_motion_ms >= kEncoderStallMs) {
        controller.integral = 0.0F;
        controller.output = 0;
        controller.stalled = true;
        controller.stalled_target = target;
        return 0;
    }

    // 方向感知的条件积分：输出饱和时，只允许积分朝退出饱和的方向变化。
    const bool push_high = requested >= kMaxOutput && error > 0.0F;
    const bool push_low = requested <= 0.0F && error < 0.0F;
    if (!push_high && !push_low) {
        controller.integral = candidate_integral;
    }

    const float output = clamp(requested, 0.0F, kMaxOutput);
    controller.output = static_cast<int>(output + 0.5F);
    controller.stalled = false;
    return direction * controller.output;
}
} // namespace

void motor_pid_init() {
    configure_encoder_pins();
    configure_encoder(controllers[0], TIM2); // M1: PA15/PB3
    configure_encoder(controllers[1], TIM3); // M2: PA6/PA7
    configure_encoder(controllers[2], TIM5); // M3: PA0/PA1
    configure_encoder(controllers[3], TIM4); // M4: PB6/PB7

    const uint32_t now_ms = HAL_GetTick();
    for (unsigned i = 0; i < kMotorCount; ++i) {
        MotorPid &controller = controllers[i];
        controller.encoder_sign = kEncoderSign[i];
        controller.previous_count = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&controller.timer));
        controller.integral = 0.0F;
        controller.last_motion_ms = now_ms;
        controller.previous_target = 0;
        controller.target = 0;
        controller.encoder_ticks = 0;
        controller.output = 0;
        controller.stalled_target = 0;
        controller.stalled = false;
    }
    last_update_ms = now_ms - kControlPeriodMs;
}

DriveCommand motor_pid_update(const DriveCommand &target, uint32_t now_ms) {
    DriveCommand output{};
    const uint32_t elapsed = now_ms - last_update_ms;
    if (elapsed < kControlPeriodMs) {
        // 两次控制更新之间保持上一次的输出幅值，目标归零时立即停车。
        for (unsigned i = 0; i < kMotorCount; ++i) {
            const int command = clamp_target(target.motor[i]);
            output.motor[i] =
                command == 0 ? 0 : (command < 0 ? -controllers[i].output : controllers[i].output);
        }
        return output;
    }

    last_update_ms = now_ms;
    const uint32_t dt_ms = elapsed > kMaxDtMs ? kMaxDtMs : elapsed;
    const float dt_s = static_cast<float>(dt_ms) / 1000.0F;
    for (unsigned i = 0; i < kMotorCount; ++i) {
        output.motor[i] =
            update_controller(controllers[i], clamp_target(target.motor[i]), now_ms, dt_s);
    }
    return output;
}

MotorPidStatus motor_pid_status(unsigned motor) {
    if (motor >= kMotorCount) {
        return {};
    }
    const MotorPid &controller = controllers[motor];
    return {controller.target, controller.encoder_ticks, controller.output, controller.stalled};
}
