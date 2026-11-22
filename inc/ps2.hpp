#pragma once

#include "main.h"
#include "ps2_protocol.hpp"

// 单个接收器，由主循环调用；不在中断中读写。
// 调试统计只用于观察，不作为控制逻辑的输入。
struct Ps2Diagnostics {
    uint32_t polls = 0;
    uint32_t valid_frames = 0;
    uint32_t invalid_frames = 0;
    uint32_t spi_errors = 0;
    uint32_t input_changes = 0;
    uint32_t last_valid_ms = 0;
    uint32_t last_hal_status = 0;
    uint32_t raw_length = 0;
    uint32_t config_attempts = 0;
    uint8_t mode = 0;
    uint8_t raw[24] = {};
};

extern Ps2Diagnostics ps2_debug;
void ps2_init(SPI_HandleTypeDef &spi);
void ps2_update();
// 主循环已更新电机输出后，才允许尝试恢复模拟模式。
void ps2_recover_analog();
// 机械臂、维修模式和自检读取最新一帧，坏帧立即视为断线。
Ps2Frame ps2_read();
// 正常驾驶可以跨过一次坏帧，未收到有效帧时最多保持上次指令 40 ms。
Ps2Frame ps2_read_drive();
