#include "headfile.h"

/* ══════════════════ 灰度传感器读取 ══════════════════ */
/* 黑色输出高电平 */

uint8_t GraySensor_Read(void)
{
    uint8_t val = 0;
    if (!DL_GPIO_readPins(GRAY_PORT, GRAY_S1_PIN))
        val |= (1 << 0);
    if (!DL_GPIO_readPins(GRAY_PORT, GRAY_S2_PIN))
        val |= (1 << 1);
    if (!DL_GPIO_readPins(GRAY_PORT, GRAY_S3_PIN))
        val |= (1 << 2);
    if (!DL_GPIO_readPins(GRAY_PORT, GRAY_S4_PIN))
        val |= (1 << 3);
    if (!DL_GPIO_readPins(GRAY_PORT, GRAY_S5_PIN))
        val |= (1 << 4);
    if (!DL_GPIO_readPins(GRAY_PORT, GRAY_S6_PIN))
        val |= (1 << 5);
    if (!DL_GPIO_readPins(GRAY_PORT, GRAY_S7_PIN))
        val |= (1 << 6);
    if (!DL_GPIO_readPins(GRAY_PORT, GRAY_S8_PIN))
        val |= (1 << 7);
    return val;
}

/* ══════════════════ 循迹策略 (原 track_strategy) ══════════════════ */

/* 权重: bit0=S1(最左)~bit7=S8(最右), 放大 2.5x 增强 PID 纠偏力度 */
static const float s_weights[8] = {-10.0f, -7.5f, -5.0f, -2.5f, 2.5f, 5.0f, 7.5f, 10.0f};

static float s_last_error = 0.0f;

float Track_GetError(uint8_t sensor_data)
{
    if (sensor_data == 0xFF)
        return 0.0f;
    if (sensor_data == 0x00)
        return s_last_error;

    float sum = 0.0f;
    uint8_t cnt = 0;

    for (int i = 0; i < 8; i++)
    {
        if (sensor_data & (1 << i))
        {
            sum += s_weights[i];
            cnt++;
        }
    }
    if (cnt < 2)
        return s_last_error;

    s_last_error = sum / (float)cnt;
    return s_last_error;
}

TrackState_t Track_GetState(uint8_t sensor_data)
{
    if (sensor_data == 0xFF)
        return TRACK_ALL_BLACK;
    if (sensor_data == 0x00)
        return TRACK_ALL_WHITE;
    return TRACK_NORMAL;
}
