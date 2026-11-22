#include "arm_control.hpp"
#include "main.h"

namespace {
// 舵机通道对应关节
enum ServoIndex {
    kServoBase = 0,     // CH1 底座
    kServoShoulder = 1, // CH2 肩关节
    kServoElbow = 2,    // CH3 肘/腕关节
    kServoGripper = 3   // CH4 夹爪
};

// ==================== 待填参数区（调参时修改这里） ====================
// 假设 0 为中位(1500us)。初始给个保守的限幅，防止一开机就把自己打死。
constexpr int kServoMin[4] = {-90, -90, -90, -90}; 
constexpr int kServoMax[4] = { 90,  90,  90,  90};

// 预设姿态表（先用全0占位，待会儿实车调参填入具体数值）
constexpr int kPoseSTOW[4]          = {0, -70, 90, 0};
constexpr int kPosePICK[4]          = {0, -80, 20, 80};
constexpr int kPoseCARRY[4]         = {0, -20, 70, -80};
constexpr int kPosePLACE_GROUND[4]  = {0, -80, 20, 80};
constexpr int kPosePLACE_PLATFORM[4]= {0, 20, 40, 80};
// =====================================================================

int target_pose[4] = {0, 0, 0, 0};
int current_pose[4] = {0, 0, 0, 0};
uint16_t last_buttons = 0;
uint32_t last_interp_ms = 0;
bool is_place_platform = false; // Start键的循环切换状态

// 将手柄 0~255 的轴值映射到 -100~100
int axis_percent(uint8_t axis) {
    const int delta = static_cast<int>(axis) - 128;
    return delta < 0 ? delta * 100 / 128 : delta * 100 / 127;
}

// 按键边沿检测：只响应刚按下的那一瞬间，按住不连发
bool pressed_edge(const Ps2Frame &frame, Ps2Button button) {
    const bool pressed = ps2_pressed(frame, button);
    const bool was_pressed = (last_buttons & (1U << static_cast<int>(button))) != 0;
    return pressed && !was_pressed;
}

// 加载预设到目标位置
void load_pose(const int pose[4]) {
    for (int i = 0; i < 4; ++i) target_pose[i] = pose[i];
}
} // namespace

// 急停函数：瞬间拉满到 STOW，清空按键状态
ArmCommand arm_emergency_stop() {
    load_pose(kPoseSTOW);
    for (int i = 0; i < 4; ++i) current_pose[i] = target_pose[i];
    last_buttons = 0;
    
    ArmCommand cmd;
    for (int i = 0; i < 4; ++i) cmd.servo[i] = current_pose[i];
    return cmd;
}

ArmCommand arm_from_ps2(const Ps2Frame &frame) {
    // 如果手柄断链，自动回安全姿态
    if (!frame.connected) return arm_emergency_stop();

    // 1. 预设姿态切换
    if (pressed_edge(frame, Ps2Button::Square))   load_pose(kPosePICK);
    if (pressed_edge(frame, Ps2Button::Triangle)) load_pose(kPoseCARRY);
    if (pressed_edge(frame, Ps2Button::Start)) {
        is_place_platform = !is_place_platform;
        load_pose(is_place_platform ? kPosePLACE_PLATFORM : kPosePLACE_GROUND);
    }

    // 2. 夹爪控制（覆盖目标姿态的夹爪项）
    if (pressed_edge(frame, Ps2Button::Cross))  target_pose[kServoGripper] = -60; // 闭合
    if (pressed_edge(frame, Ps2Button::Circle)) target_pose[kServoGripper] = 60;  // 张开

    // 3. 摇杆手动微调（右摇杆控制底座和肩部，死区 5）
    if (frame.analog) {
        const int rx = axis_percent(frame.right_x);
        const int ry = axis_percent(frame.right_y);
        
        // 按住左摇杆（L3）时，右摇杆上下控制肘关节
        if (ps2_pressed(frame, Ps2Button::L3)) {
            if (ry < -10 || ry > 10) target_pose[kServoElbow] = ry;
        } else {
            // 没按 L3 时，右摇杆正常控制底座和肩部
            if (rx < -10 || rx > 10) target_pose[kServoBase] = rx;
            if (ry < -10 || ry > 10) target_pose[kServoShoulder] = ry;
        }
    }

    last_buttons = frame.buttons;

    // 4. 平滑插值（每 20ms 更新一次，每次移动 3%）
    const uint32_t now = HAL_GetTick();
    if (now - last_interp_ms >= 20) {
        last_interp_ms = now;
        for (int i = 0; i < 4; ++i) {
            int delta = target_pose[i] - current_pose[i];
            const int step = 3; 
            if (delta > step) delta = step;
            else if (delta < -step) delta = -step;
            
            current_pose[i] += delta;
            
            // 安全限幅
            if (current_pose[i] < kServoMin[i]) current_pose[i] = kServoMin[i];
            if (current_pose[i] > kServoMax[i]) current_pose[i] = kServoMax[i];
        }
    }

    // 返回当前输出指令
    ArmCommand cmd;
    for (int i = 0; i < 4; ++i) cmd.servo[i] = current_pose[i];
    return cmd;
}