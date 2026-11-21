#pragma once

namespace car_config {
enum class MecanumLayout { X, O };
// 本车经团队确认使用 X 型麦轮。
constexpr MecanumLayout kMecanumLayout = MecanumLayout::X;

// 枚举值同时是混控数组的下标，按左前、右前、左后、右后排列。
enum class Wheel : unsigned { FrontLeft = 0, FrontRight, RearLeft, RearRight };
// 以电机通道为下标。公版的 M3 为右后，M4 为左后，改装车以实测为准。
constexpr Wheel kMotorWheel[4] = {Wheel::FrontLeft, Wheel::FrontRight, Wheel::RearRight,
                                  Wheel::RearLeft};
// +1 表示 A=1、B=0 时车轮朝车头方向滚动，否则使用 -1。
constexpr int kMotorPolarity[4] = {1, 1, 1, 1};

// 历史变量名带 Duty。这里定义的是混控送往速度闭环的目标比例上限。
constexpr int kMaxDutyPercent = 100;
// 方向键平移和 L1/R1 旋转各有独立的固定目标幅值。
constexpr int kButtonDutyPercent = 80;
constexpr int kRotationButtonPercent = 60;
// 公版单通道标定目标，最终 PWM 由速度闭环决定。
constexpr int kCalibrationDutyPercent = 35;
// 以 128 为中点的原始摇杆刻度，当前 112～144 都进入死区。
constexpr int kAxisDeadzone = 16;
// 驾驶档位相对于混控完整输出的比例，实车调校后可调整。
constexpr int kPrecisionScalePercent = 30;
constexpr int kNormalScalePercent = 70;
constexpr int kTurboScalePercent = 100;
static_assert(kPrecisionScalePercent > 0 &&
                  kPrecisionScalePercent < kNormalScalePercent &&
                  kNormalScalePercent < kTurboScalePercent && kTurboScalePercent <= 100,
              "Invalid drive mode scales");
// 编译期检查目标上限和死区配置。
static_assert(kMaxDutyPercent > 0 && kMaxDutyPercent <= 100, "Invalid PWM limit");
static_assert(kRotationButtonPercent > 0 && kRotationButtonPercent <= kMaxDutyPercent,
              "Invalid rotation command limit");
static_assert(kButtonDutyPercent > 0 && kButtonDutyPercent <= kMaxDutyPercent,
              "Invalid button speed");
static_assert(kAxisDeadzone >= 0 && kAxisDeadzone < 127, "Invalid stick deadzone");
static_assert(kCalibrationDutyPercent > 0 && kCalibrationDutyPercent <= 100,
              "Invalid calibration duty");

} // namespace car_config
