#pragma once

namespace car_config {
enum class MecanumLayout { X, O };
// 俯视时四个麦轮的滚子组成 O 型。
constexpr MecanumLayout kMecanumLayout = MecanumLayout::O;
enum class Wheel : unsigned { FrontLeft = 0, FrontRight, RearLeft, RearRight };
// M1～M4 的轮位已经通过单电机校准确认。
constexpr Wheel kMotorWheel[4] = {Wheel::FrontLeft, Wheel::FrontRight, Wheel::RearRight,
                                  Wheel::RearLeft};
// +1 表示 A=1、B=0 时车轮朝车头方向滚动，否则使用 -1。
// 已按实车测试反转 M1、M2；所有电机的正方向现使用 +1。
constexpr int kMotorPolarity[4] = {1, 1, 1, 1};
constexpr int kMaxDutyPercent = 100;
constexpr int kButtonDutyPercent = 80;
constexpr int kCalibrationDutyPercent = 35;
constexpr int kAxisDeadzone = 16;
static_assert(kMaxDutyPercent > 0 && kMaxDutyPercent <= 100, "Invalid PWM limit");
static_assert(kButtonDutyPercent > 0 && kButtonDutyPercent <= kMaxDutyPercent,
              "Invalid button speed");
static_assert(kAxisDeadzone >= 0 && kAxisDeadzone < 127, "Invalid stick deadzone");
static_assert(kCalibrationDutyPercent > 0 && kCalibrationDutyPercent <= 100,
              "Invalid calibration duty");

}
