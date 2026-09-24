#ifndef __GRAY_SENSOR_H
#define __GRAY_SENSOR_H

#include "headfile.h"

/* ── 灰度传感器读取 ── */
/* 返回值: bit0=S1, bit1=S2, ..., bit7=S8; 1=黑线, 0=白底 */
uint8_t GraySensor_Read(void);

#define GRAY_S1(val) (((val) >> 0) & 0x01)
#define GRAY_S2(val) (((val) >> 1) & 0x01)
#define GRAY_S3(val) (((val) >> 2) & 0x01)
#define GRAY_S4(val) (((val) >> 3) & 0x01)
#define GRAY_S5(val) (((val) >> 4) & 0x01)
#define GRAY_S6(val) (((val) >> 5) & 0x01)
#define GRAY_S7(val) (((val) >> 6) & 0x01)
#define GRAY_S8(val) (((val) >> 7) & 0x01)

/* ── 循迹策略 (原 track_strategy) ── */
typedef enum
{
    TRACK_NORMAL = 0,
    TRACK_ALL_BLACK = 1,
    TRACK_ALL_WHITE = 2,
} TrackState_t;

float Track_GetError(uint8_t sensor_data);
TrackState_t Track_GetState(uint8_t sensor_data);

#endif /* __GRAY_SENSOR_H */
