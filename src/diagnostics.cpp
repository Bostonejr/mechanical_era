#include "diagnostics.hpp"
#include "arm_control.hpp"
#include "motor_pid.hpp"
#include "ps2.hpp"
#include "service_mode.hpp"
#include <cstdio>

extern "C" UART_HandleTypeDef huart3;

void report_status(const Ps2Frame &frame, DriveCommand command, bool emergency_stop,
                   bool key_pressed) {
    char message[192];
    const int length = std::snprintf(
        message, sizeof(message),
        "PS2 online=%lu mode=%02lX keys=%04lX LX=%lu LY=%lu RX=%lu RY=%lu M=%d,%d,%d,%d STOP=%u KEY=%u "
        "SVC=%u bad=%lu REL=%u CH4us=%lu\r\n",
        static_cast<unsigned long>(frame.connected), static_cast<unsigned long>(ps2_debug.mode),
        static_cast<unsigned long>(frame.buttons), static_cast<unsigned long>(frame.left_x),
        static_cast<unsigned long>(frame.left_y), static_cast<unsigned long>(frame.right_x),
        static_cast<unsigned long>(frame.right_y),
        command.motor[0], command.motor[1], command.motor[2], command.motor[3],
        static_cast<unsigned>(emergency_stop), static_cast<unsigned>(key_pressed),
        static_cast<unsigned>(service_mode_active()),
        static_cast<unsigned long>(ps2_debug.invalid_frames),
        static_cast<unsigned>(arm_servos_released()), static_cast<unsigned long>(TIM8->CCR4));
    if (length > 0 && static_cast<unsigned>(length) < sizeof(message)) {
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t *>(message),
                          static_cast<uint16_t>(length), 20);
    }

    // PID status for all four motors: target %, encoder ticks per 10 ms cycle,
    // PI output %, stall flags.
    // 全部四个电机的 PID 状态：目标百分比、每 10 ms 周期的编码器计数、
    // PI 输出百分比、堵转标志。
    const MotorPidStatus pid_status[4] = {motor_pid_status(0), motor_pid_status(1),
                                          motor_pid_status(2), motor_pid_status(3)};
    const int pid_length = std::snprintf(
        message, sizeof(message),
        "PID t=%d,%d,%d,%d k=%d,%d,%d,%d o=%d,%d,%d,%d s=%u%u%u%u\r\n",
        pid_status[0].target, pid_status[1].target, pid_status[2].target,
        pid_status[3].target, pid_status[0].encoder_ticks, pid_status[1].encoder_ticks,
        pid_status[2].encoder_ticks, pid_status[3].encoder_ticks, pid_status[0].output,
        pid_status[1].output, pid_status[2].output, pid_status[3].output,
        static_cast<unsigned>(pid_status[0].stalled),
        static_cast<unsigned>(pid_status[1].stalled),
        static_cast<unsigned>(pid_status[2].stalled),
        static_cast<unsigned>(pid_status[3].stalled));
    if (pid_length > 0 && static_cast<unsigned>(pid_length) < sizeof(message)) {
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t *>(message),
                          static_cast<uint16_t>(pid_length), 20);
    }

    // Verbose M4 hardware register dump for debugging the rear-left wheel
    // (GPIO config: PA11=0xA alternate push-pull, PB10/PB11=0x2 push-pull).
    // Service mode only, to keep match-day logs readable.
    // M4（左后轮）硬件寄存器转储，用于排查该轮问题
    // （GPIO 配置字段：PA11=0xA 复用推挽输出，PB10/PB11=0x2 推挽输出）。
    // 仅在维修模式输出，保持比赛日日志清爽。
    if (service_mode_active()) {
        const int m4_hardware_length =
            std::snprintf(message, sizeof(message),
                          "M4HW-v1 req=%d ccr=%lu arr=%lu run=%u en=%u moe=%u oc=%lu "
                          "pa11=%lX pb10=%lX pb11=%lX out=%u,%u map=%08lX\r\n",
                          command.motor[3], static_cast<unsigned long>(TIM1->CCR4),
                          static_cast<unsigned long>(TIM1->ARR),
                          static_cast<unsigned>((TIM1->CR1 & TIM_CR1_CEN) != 0U),
                          static_cast<unsigned>((TIM1->CCER & TIM_CCER_CC4E) != 0U),
                          static_cast<unsigned>((TIM1->BDTR & TIM_BDTR_MOE) != 0U),
                          static_cast<unsigned long>((TIM1->CCMR2 >> 12U) & 7U),
                          static_cast<unsigned long>((GPIOA->CRH >> 12U) & 15U),
                          static_cast<unsigned long>((GPIOB->CRH >> 8U) & 15U),
                          static_cast<unsigned long>((GPIOB->CRH >> 12U) & 15U),
                          static_cast<unsigned>((GPIOB->IDR & motor4_A_Pin) != 0U),
                          static_cast<unsigned>((GPIOB->IDR & motor4_B_Pin) != 0U),
                          static_cast<unsigned long>(AFIO->MAPR));
        if (m4_hardware_length > 0 && static_cast<unsigned>(m4_hardware_length) < sizeof(message)) {
            HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t *>(message),
                              static_cast<uint16_t>(m4_hardware_length), 20);
        }
    }
}
