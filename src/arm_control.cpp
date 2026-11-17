#include "arm_control.hpp"

namespace {
int axis_percent(uint8_t axis) {
    const int delta = static_cast<int>(axis) - 128;
    return delta < 0 ? delta * 100 / 128 : delta * 100 / 127;
}
}

ArmCommand arm_from_ps2(const Ps2Frame &frame) {
    ArmCommand command;
    if (!frame.connected) {
        return command;
    }
    if (frame.analog) {
        // 机械臂的两个摇杆轴交换：上下控制第 1 路，左右控制第 4 路。
        command.servo[0] = axis_percent(frame.right_y);
        command.servo[3] = axis_percent(frame.right_x);
    }
    if (ps2_pressed(frame, Ps2Button::Cross)) {
        command.servo[2] -= 10;
    }
    if (ps2_pressed(frame, Ps2Button::Circle)) {
        command.servo[2] += 10;
    }
    return command;
}
