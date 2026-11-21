#include "drive_control.hpp"
#include "service_mode.hpp"

namespace {
// 混控要找最大的轮速幅值，正反两个方向都纳入比较。
int magnitude(int value) { return value < 0 ? -value : value; }

constexpr bool valid_configuration() {
    // seen 的低四位分别标记四个物理轮位。重复轮位或非法极性会让编译失败。
    unsigned seen = 0;
    for (unsigned i = 0; i < 4; ++i) {
        const unsigned wheel = static_cast<unsigned>(car_config::kMotorWheel[i]);
        if (wheel >= 4 || (seen & (1U << wheel)) ||
            (car_config::kMotorPolarity[i] != 1 && car_config::kMotorPolarity[i] != -1)) {
            return false;
        }
        seen |= 1U << wheel;
    }
    return seen == 15U;
}
static_assert(valid_configuration(), "Assign every wheel once and use polarity +1/-1");

int axis(uint8_t value, bool invert) {
    // PS2 轴值为 0～255。减去中点 128 后，使用对称死区消除松杆抖动。
    const int delta = static_cast<int>(value) - 128;
    const int distance = magnitude(delta);
    if (distance <= car_config::kAxisDeadzone) {
        return 0;
    }
    // 两端有效行程相差 1 格。先减掉死区，再把剩余行程拉伸到完整目标范围。
    const int full_scale = delta < 0 ? 128 : 127;
    int scaled = (distance - car_config::kAxisDeadzone) * car_config::kMaxDutyPercent /
                 (full_scale - car_config::kAxisDeadzone);
    if (delta < 0) {
        scaled = -scaled;
    }
    // 左摇杆向上时 LY 变小，调用者用 invert=true 让前进为正。
    return invert ? -scaled : scaled;
}
} // namespace

DriveCommand mix_drive(int forward, int strafe, int rotation, car_config::MecanumLayout layout) {
    // O 型与 X 型的横向滚子方向相反。轮序为左前、右前、左后、右后。
    // 旋转指令仍表示归一化的左右差速量，不是角速度。
    if (layout == car_config::MecanumLayout::O) {
        strafe = -strafe;
    }
    // 数组按物理位置保存。每个元素都是前进、横移、旋转的带符号叠加。
    int wheel[4] = {forward + strafe + rotation, forward - strafe - rotation,
                    forward - strafe + rotation, forward + strafe - rotation};
    // 从上限开始找峰值。没超限时比例为 1，超限时四轮一起缩小。
    int peak = car_config::kMaxDutyPercent;
    for (int speed : wheel) {
        if (magnitude(speed) > peak) {
            peak = magnitude(speed);
        }
    }
    DriveCommand result{};
    for (unsigned i = 0; i < 4; ++i) {
        // 以电机通道 i 查物理轮位，再将该轮目标换成驱动接线所需的极性。
        result.motor[i] = wheel[static_cast<unsigned>(car_config::kMotorWheel[i])] *
                          car_config::kMaxDutyPercent / peak * car_config::kMotorPolarity[i];
    }
    return result;
}

DriveCommand drive_from_ps2(const Ps2Frame &frame) {
    // Select 按住时，正常驾驶的四路电机目标归零。Cross 由机械臂处理。
    if (!frame.connected || ps2_pressed(frame, Ps2Button::Select)) {
        return {};
    }
    // 主循环已更新维修模式。维修时只接受单轮标定，普通驾驶留在比赛模式。
    if (service_mode_active()) {
        return drive_calibration_from_ps2(frame);
    }
    const bool up = ps2_pressed(frame, Ps2Button::Up);
    const bool down = ps2_pressed(frame, Ps2Button::Down);
    const bool left = ps2_pressed(frame, Ps2Button::Left);
    const bool right = ps2_pressed(frame, Ps2Button::Right);

    // 模拟帧有摇杆轴值。数字帧仍能走下面的方向键和旋转键分支。
    int forward = frame.analog ? axis(frame.left_y, true) : 0;
    int strafe = frame.analog ? axis(frame.left_x, false) : 0;

    // 方向键提供固定纵横指令，并同时覆盖左摇杆的两轴。
    if (up || down || left || right) {
        forward = (static_cast<int>(up) - static_cast<int>(down)) * car_config::kButtonDutyPercent;
        strafe =
            (static_cast<int>(right) - static_cast<int>(left)) * car_config::kButtonDutyPercent;
    }
    const bool l1 = ps2_pressed(frame, Ps2Button::L1);
    const bool r1 = ps2_pressed(frame, Ps2Button::R1);
    // 右摇杆留给机械臂。只按 R1 为正、只按 L1 为负，两键同按或都松开时为零。
    const int rotation =
        (static_cast<int>(r1) - static_cast<int>(l1)) * car_config::kRotationButtonPercent;
    // 先完成轮位映射、极性和混控限幅，再同时缩放四个电机目标。
    DriveCommand command = mix_drive(forward, strafe, rotation);
    int scale_percent = car_config::kNormalScalePercent;
    // 两键同按时按本示例采用精细档，避免狭窄对位时突然升速。
    if (ps2_pressed(frame, Ps2Button::L2)) {
        scale_percent = car_config::kPrecisionScalePercent;
    } else if (ps2_pressed(frame, Ps2Button::R2)) {
        scale_percent = car_config::kTurboScalePercent;
    }
    for (int &target : command.motor) {
        // 引用会修改数组里的元素。整数结果向零取整，小目标可能变成零。
        target = target * scale_percent / 100;
    }
    return command;
}

DriveCommand drive_calibration_from_ps2(const Ps2Frame &frame) {
    DriveCommand command{};
    // 急停和是否处于维修模式由上层判断。这里检查本次单轮标定的输入。
    if (!frame.connected || !ps2_pressed(frame, Ps2Button::Start) ||
        ps2_pressed(frame, Ps2Button::Cross) || ps2_pressed(frame, Ps2Button::Select)) {
        return command;
    }

    const bool up = ps2_pressed(frame, Ps2Button::Up);
    const bool right = ps2_pressed(frame, Ps2Button::Right);
    const bool down = ps2_pressed(frame, Ps2Button::Down);
    const bool left = ps2_pressed(frame, Ps2Button::Left);
    // 同一时刻只允许一个方向键。上、右、下、左分别测试 M1～M4。
    if (static_cast<int>(up) + right + down + left != 1) {
        return command;
    }
    const unsigned motor = up ? 0 : right ? 1 : down ? 2 : 3;
    const int direction = ps2_pressed(frame, Ps2Button::R2) ? -1 : 1;
    // 直接写电机通道目标，由编码器闭环计算实际 PWM。
    command.motor[motor] = car_config::kCalibrationDutyPercent * direction;
    return command;
}
