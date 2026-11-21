#pragma once

#include "ps2_protocol.hpp"
#include "car_config.hpp"

struct DriveCommand {
    // motor[0]～motor[3] 对应接线通道 M1～M4。
    // 混控输出的符号表示方向，幅值表示轮速目标百分比。
    int motor[4] = {};
};

// 前进、向右平移、顺时针旋转分别为对应轴的正方向。
// 输入是归一化指令。旋转项表示左右差速的量，实车轮速由下游闭环控制。
DriveCommand mix_drive(int forward, int strafe, int rotation,
                       car_config::MecanumLayout layout = car_config::kMecanumLayout);

// 读取已解码的手柄输入，按当前模式输出驾驶或标定目标。断联时返回全零目标。
DriveCommand drive_from_ps2(const Ps2Frame &frame);

// 供维修路径调用，按 Start 和一个方向键生成单通道标定目标。
DriveCommand drive_calibration_from_ps2(const Ps2Frame &frame);
