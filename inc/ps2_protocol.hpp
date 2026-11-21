#pragma once

// ==================== Competition button map contract (whole team) ====================
// Left stick          Chassis forward / strafe
// L1 / R1             Rotate left / right
// Right stick         Arm joints (base + wrist)
// R2 (hold)           Turbo mode (race)                  -- drive_control
// L2 (hold)           Precision creep mode               -- drive_control
// Select (hold)       Park: zero chassis output while held; arm keeps working so
//                     the driver can brake mid-place     -- drive_control (already in base code)
// Cross / Circle      Gripper close / open               -- arm_control
// Triangle            CARRY pose                         -- arm_control (freed from wheel test)
// Square              PICK pose                          -- arm_control
// Start               PLACE pose / cycle variants        -- arm_control (freed from calibration)
// Select + Start      Hold 2 s during boot window to enter service mode (see service_mode.hpp)
// Onboard key         Emergency stop toggle
// Service mode only: Triangle wheel self-test, Start+D-pad single-motor calibration.
// Note: the base drive code also brakes on Cross; P1 removes Cross from that stop
// condition when the gripper lands, or braking will fire on every gripper close.
// Negotiate with the team before changing this table; keep the bilingual plan docs in sync.

// ==================== 比赛按键映射契约（全员遵守） ====================
// 左摇杆              底盘前进 / 平移
// L1 / R1             左旋 / 右旋
// 右摇杆              机械臂关节（底座 + 腕部）
// R2（按住）          涡轮模式（竞速）                   —— drive_control
// L2（按住）          精细蠕动模式                       —— drive_control
// Select（按住）      驻车：按住期间底盘输出清零；机械臂保持工作，
//                     便于放置过程中刹车                 —— drive_control（基础代码已实现）
// Cross / Circle      夹爪闭合 / 张开                    —— arm_control
// Triangle            CARRY 姿态                         —— arm_control（已从单轮自检释放）
// Square              PICK 姿态                          —— arm_control
// Start               PLACE 姿态 / 循环切换              —— arm_control（已从标定释放）
// Select + Start      开机窗口内按住 2 秒进入维修模式（见 service_mode.hpp）
// 板载按键            急停切换
// 仅维修模式保留：Triangle 单轮自检、Start+方向键 单电机标定。
// 注意：基础驱动代码中 Cross 也会触发刹车；夹爪功能合并后，程序员 1 需将 Cross
// 从刹车条件中移除，否则每次闭合夹爪都会同时刹车。
// 修改本表前先与全队协商，并保持 kimi_plan 中的双语计划文档同步。

#include <cstddef>
#include <cstdint>

enum class Ps2Button : uint8_t {
    Select = 0,
    L3,
    R3,
    Start,
    Up,
    Right,
    Down,
    Left,
    L2,
    R2,
    L1,
    R1,
    Triangle,
    Circle,
    Cross,
    Square
};

struct Ps2Frame {
    bool connected = false;
    bool analog = false;
    uint16_t buttons = 0; // 1 表示按下；仅在解码入口转换协议的低有效位。
    uint8_t right_x = 128;
    uint8_t right_y = 128;
    uint8_t left_x = 128;
    uint8_t left_y = 128;
};

// 返回模式对应的完整报文长度；未知模式返回 0。
std::size_t ps2_packet_size(uint8_t mode);
Ps2Frame decode_ps2_frame(const uint8_t *bytes, std::size_t length);
bool ps2_pressed(const Ps2Frame &frame, Ps2Button button);
