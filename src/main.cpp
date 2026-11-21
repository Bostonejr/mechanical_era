#include "app.h"
#include "ps2.hpp"
#include "outputs.hpp"
#include "diagnostics.hpp"
#include "service_mode.hpp"

extern "C" SPI_HandleTypeDef hspi2;

extern "C" void main_task() {
    init_outputs();
    ps2_init(hspi2);

    constexpr uint32_t kWheelTestStepMs = 1000;
    constexpr int kWheelTestDutyPercent = 35;
    uint32_t last_report = HAL_GetTick() - 500U;
    bool emergency_stop = false;
    bool key_sample = false;
    bool key_pressed = false;
    uint32_t key_change_ms = HAL_GetTick();
    bool triangle_was_pressed = false;
    bool wheel_test_active = false;
    uint32_t wheel_test_started_ms = 0;

    while (true) {
        // 20 ms 去抖后按一次锁定急停，再按一次解除
        const bool key_now = HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin) == GPIO_PIN_RESET;
        if (key_now != key_sample) {
            key_sample = key_now;
            key_change_ms = HAL_GetTick();
        } else if (key_now != key_pressed && HAL_GetTick() - key_change_ms >= 20U) {
            key_pressed = key_now;
            if (key_pressed) {
                emergency_stop = !emergency_stop;
            }
        }
        if (emergency_stop) {
            apply_motors({});
        }
        ps2_update();
        const auto frame = ps2_read();
        service_mode_update(frame);
        const bool triangle_pressed = ps2_pressed(frame, Ps2Button::Triangle);
        // Wheel self-test only exists in service mode; in normal mode Triangle
        // belongs to the arm presets (see the button map in ps2_protocol.hpp).
        // 单轮自检仅在维修模式下可用；正常模式下 Triangle 属于机械臂预设
        // （见 ps2_protocol.hpp 中的按键映射）。
        if (service_mode_active() && triangle_pressed && !triangle_was_pressed && !emergency_stop) {
            // 每次按下三角键，仅执行一次 M1 -> M2 -> M3 -> M4 自检。
            wheel_test_active = true;
            wheel_test_started_ms = HAL_GetTick();
        }
        triangle_was_pressed = triangle_pressed;

        const bool cancel_wheel_test = emergency_stop || !frame.connected ||
                                       ps2_pressed(frame, Ps2Button::Cross) ||
                                       ps2_pressed(frame, Ps2Button::Select);
        if (cancel_wheel_test) {
            wheel_test_active = false;
        }

        DriveCommand command = emergency_stop ? DriveCommand{} : drive_from_ps2(frame);
        if (wheel_test_active) {
            const uint32_t elapsed = HAL_GetTick() - wheel_test_started_ms;
            const unsigned motor = elapsed / kWheelTestStepMs;
            if (motor < 4) {
                command = {};
                command.motor[motor] = kWheelTestDutyPercent;
            } else {
                wheel_test_active = false;
                command = {};
            }
        }
        apply_motors(command);
        // Coordination point with arm_control: emergency stop currently sends a
        // zeroed ArmCommand (all servos to neutral). Once P3 defines the STOW
        // preset, command STOW here instead if neutral is not a safe pose.
        // 与 arm_control 的协调点：急停时目前发送清零的 ArmCommand（所有舵机回中位）。
        // 待程序员 3 定义 STOW 姿态后，若中位不是安全姿态，此处应改为命令 STOW。
        const auto arm_command = emergency_stop ? arm_emergency_stop() : arm_from_ps2(frame); // P3
        apply_servos(arm_command);
        if (HAL_GetTick() - last_report >= 500U) {
            report_status(frame, command, emergency_stop, key_pressed);
            last_report = HAL_GetTick();
        }
        HAL_Delay(1);
    }
}
