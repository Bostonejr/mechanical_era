#include "arm_control.hpp"
#include "main.h"

namespace {
// 顺序 = 姿态表四个数的顺序。实体插座是 CH3=夹爪、CH4=肘。
enum ServoIndex {
    kServoBase = 0,     // CH1 底座
    kServoShoulder = 1, // CH2 肩
    kServoGripper = 2,  // CH3 夹爪
    kServoElbow = 3     // CH4 肘/腕
};

// ==================== 待填参数区（角度制 0~180，90=中位） ====================
// 先给临时行程，标定后改成机械实际角度。
constexpr int kServoMin[4] = {9, 9, 9, 9};
constexpr int kServoMax[4] = {171, 171, 171, 171};

// {底座, 肩, 夹爪, 肘}
constexpr int kPoseSTOW[4] = {50, 27, 90, 171};
constexpr int kPosePICK[4] = {90, 18, 162, 108};
constexpr int kPoseCARRY[4] = {90, 72, 18, 153};
constexpr int kPosePLACE_GROUND[4] = {90, 18, 162, 108};
constexpr int kPosePLACE_PLATFORM[4] = {90, 108, 162, 126};
// =====================================================================

int target_pose[4] = {90, 90, 90, 90};
int current_pose[4] = {90, 90, 90, 90};
uint16_t last_buttons = 0;
uint32_t last_interp_ms = 0;
bool is_place_platform = false;
bool servos_released = false; // R3 自锁：true 时停止全部舵机

// 姿态表不能超出 kServoMin/Max，超了编译就报错
constexpr bool valid_pose(const int pose[4]) {
    for (int i = 0; i < 4; ++i) {
        if (pose[i] < kServoMin[i] || pose[i] > kServoMax[i]) return false;
    }
    return true;
}
static_assert(valid_pose(kPoseSTOW) && valid_pose(kPosePICK) && valid_pose(kPoseCARRY) &&
                  valid_pose(kPosePLACE_GROUND) && valid_pose(kPosePLACE_PLATFORM),
              "Pose table outside kServoMin/kServoMax");

// 摇杆 -100~100 百分比 → 角度，90 为中位（1% = 0.9°）
int percent_to_degrees(int percent) {
    const int degrees = 90 + percent * 9 / 10;
    return degrees < 0 ? 0 : (degrees > 180 ? 180 : degrees);
}

// 摇杆 0~255 → -100~100
int axis_percent(uint8_t axis) {
    const int delta = static_cast<int>(axis) - 128;
    return delta < 0 ? delta * 100 / 128 : delta * 100 / 127;
}

bool pressed_edge(const Ps2Frame &frame, Ps2Button button) {
    const bool pressed = ps2_pressed(frame, button);
    const bool was_pressed = (last_buttons & (1U << static_cast<int>(button))) != 0;
    return pressed && !was_pressed;
}

void load_pose(const int pose[4]) {
    for (int i = 0; i < 4; ++i) target_pose[i] = pose[i];
}
} // namespace

// 急停：瞬间到 STOW（冲击较大；想平滑就把赋值那行删掉，改走插值）
ArmCommand arm_emergency_stop() {
    load_pose(kPoseSTOW);
    for (int i = 0; i < 4; ++i) current_pose[i] = target_pose[i];
    last_buttons = 0;

    ArmCommand cmd;
    for (int i = 0; i < 4; ++i) cmd.servo[i] = current_pose[i];
    return cmd;
}

ArmCommand arm_from_ps2(const Ps2Frame &frame) {
    if (!frame.connected) return arm_emergency_stop();

    // R3 切换“全部舵机停止”（自锁），卡住时按一下舵机立刻卸力
    if (pressed_edge(frame, Ps2Button::R3)) {
        servos_released = !servos_released;
        if (!servos_released) {
            // 恢复时锁在当前角度，不要继续往原目标顶
            for (int i = 0; i < 4; ++i) target_pose[i] = current_pose[i];
        }
    }
    if (servos_released) {
        last_buttons = frame.buttons;
        ArmCommand cmd;
        for (int i = 0; i < 4; ++i) cmd.servo[i] = current_pose[i];
        return cmd;
    }

    // 1. 预设姿态
    if (pressed_edge(frame, Ps2Button::Square)) load_pose(kPosePICK);
    if (pressed_edge(frame, Ps2Button::Triangle)) load_pose(kPoseCARRY);
    if (pressed_edge(frame, Ps2Button::Start)) {
        is_place_platform = !is_place_platform;
        load_pose(is_place_platform ? kPosePLACE_PLATFORM : kPosePLACE_GROUND);
    }

    // 2. 夹爪：36° 闭合 / 144° 张开（按实际改）
    if (pressed_edge(frame, Ps2Button::Cross)) target_pose[kServoGripper] = 36;
    if (pressed_edge(frame, Ps2Button::Circle)) target_pose[kServoGripper] = 144;

    // 3. 右摇杆：左右=底座，上下=肩；按住 L3 时上下=肘
    if (frame.analog) {
        const int rx = axis_percent(frame.right_x);
        const int ry = axis_percent(frame.right_y);
        if (ps2_pressed(frame, Ps2Button::L3)) {
            if (ry < -10 || ry > 10) target_pose[kServoElbow] = percent_to_degrees(ry);
        } else {
            if (rx < -10 || rx > 10) target_pose[kServoBase] = percent_to_degrees(rx);
            if (ry < -10 || ry > 10) target_pose[kServoShoulder] = percent_to_degrees(ry);
        }
    }

    last_buttons = frame.buttons;

    // 4. 插值：每 20ms 最多 3°（约 150°/s）
    const uint32_t now = HAL_GetTick();
    if (now - last_interp_ms >= 20) {
        last_interp_ms = now;
        for (int i = 0; i < 4; ++i) {
            int delta = target_pose[i] - current_pose[i];
            if (delta > 3) delta = 3;
            else if (delta < -3) delta = -3;
            current_pose[i] += delta;
            if (current_pose[i] < kServoMin[i]) current_pose[i] = kServoMin[i];
            if (current_pose[i] > kServoMax[i]) current_pose[i] = kServoMax[i];
        }
    }

    ArmCommand cmd;
    for (int i = 0; i < 4; ++i) cmd.servo[i] = current_pose[i];
    return cmd;
}

bool arm_servos_released() { return servos_released; }