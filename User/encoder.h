#ifndef _ENCODER_DRV_H_
#define _ENCODER_DRV_H_

#include "headfile.h"

/*编码器参数 */
//编码器线数，编码器每转一圈产生的脉冲数。
#define ENC_PPR 500
//编码器减速比1:28
#define ENC_GEAR_RATIO 28
#define PI 3.1416f
#define MOTOR_WHEEL_DIA 65 //mm
//编码器一圈总脉冲数
#define ENC_CNT_ONE_REV (ENC_PPR * 1 * ENC_GEAR_RATIO) /* 14000 */

extern float E_rpm[2];
extern float E_speed[2];

/*电机编号*/

#ifndef MOTOR_LEFT
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
#endif

void Encoder_Init(void);
// int32_t  Encoder_GetCount(uint8_t motor);
// float    Encoder_GetSpeedRPM(uint8_t motor);
// void     Encoder_Reset(uint8_t motor);
void Encoder_PrintStatus(void);
void Encoder_SpeedCalc(void);
void Encoder_LeftA_IRQ(void);
void Encoder_RightA_IRQ(void);

#endif
