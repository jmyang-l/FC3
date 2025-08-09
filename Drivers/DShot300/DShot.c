#include "DShot.h"
#include "tim.h"        // CubeMX 生成的 tim.h
#include "dma.h"
#include <string.h>

#define DSHOT_BUF_LEN 16
#define DSHOT_division (16+60)

static uint16_t dshotBuf[4][DSHOT_division];   // 4 路电机，每路 1 帧

/* 打包单帧 DShot300 到 dshotBuf[motor][]
 * 参数：
 *   motor     : 电机编号 0~3
 *   throttle  : 共有2048个不同的值。0保留为上锁命令，1-47被保留为特殊命令 / 48-2047（共2000步）用于实际的油门值
 *   telemetry : 遥测请求位 0/1
 *   X         : 指示是否需要拉低
 * 说明：
 *   1. 把 11-bit 油门左移 1 位，再拼 1-bit 遥测 → 12-bit value
 *   2. 计算 4-bit CRC：value ^ (value>>4) ^ (value>>8)
 *   3. 组合成 16-bit 帧：12-bit value + 4-bit CRC
 *   4. 按 MSB 先出顺序，把每位转成占空比：
 *        逻辑 1 → 300 个定时器 tick (≈ 2.5 µs)
 *        逻辑 0 → 150 个定时器 tick (≈ 1.25 µs)
 *   5. 结果写入 dshotBuf[motor][0..15]，供 DMA 循环发送
 */
static void dshot_pack_one(uint8_t motor, uint16_t throttle, uint8_t telemetry)
{
    /* 步骤 1：组装 12 位有效数据 */
    uint16_t value = ((throttle & 0x07FF) << 1) | (telemetry & 0x01);

    /* 步骤 2：计算 4 位 CRC */
    uint16_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;

    /* 步骤 3：构成 16 位完整帧 */
    uint16_t frame = (value << 4) | crc;

    /* 步骤 4：逐位映射到占空比数组，MSB 先出 */
    for (int i = 0; i < DSHOT_BUF_LEN; ++i) {
        dshotBuf[motor][i] = (frame & 0x8000) ? 300 : 150;   // 0--150 tick = 1.25us, 1--300 ticks = 2.5us
        frame <<= 1;                                    // 左移取下一位
    }
    /* 添加帧间隔 */
    for (int j = 16; j < DSHOT_division; ++j) {
        dshotBuf[motor][j] = 0;
    }

}

/* 更新指定电机油门值（0…2047）并立即刷新到 DMA 缓冲区
 * 注意：
 *   - 电机编号 0~3，超界直接返回
 *   - telemetry 固定为 0（不请求遥测）
 *   - 函数内部调用 dshot_pack_one 立即覆盖 DMA 循环缓冲区
 *   - 由于 DMA 为 Circular 模式，下一帧即使用新数据
 */
void DSHOT_Update(uint8_t motor, uint16_t throttle)
{
    if (motor >= 4) return;           // 边界保护
    dshot_pack_one(motor, throttle, 0);   // 打包并刷新缓冲区
}

/* 一次启动 4 路 DMA 循环发送（CubeMX 已生成 htim1）
 * 流程：
 *   1. 先给 4 路电机填充 0 油门，防止上电瞬间乱转
 *   2. 依次启动 TIM1_CH1~CH4 的 DMA 循环传输
 *   3. 后续只需调用 DSHOT_Update() 更新油门即可，无需再停/启 DMA
 */
void DSHOT_Start(void)
{
    /* 预置 0 油门，ESC 上电初始化更安全 */
    for (uint8_t m = 0; m < 4; ++m) {
        DSHOT_Update(m, 0);
    }

    /* 启动 4 路 DMA，Circular 模式，永不停止 */
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)dshotBuf[0], DSHOT_division);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t *)dshotBuf[1], DSHOT_division);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t *)dshotBuf[2], DSHOT_division);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, (uint32_t *)dshotBuf[3], DSHOT_division);
}
