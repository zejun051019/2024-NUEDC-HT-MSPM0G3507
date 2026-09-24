#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "headfile.h"

/* 电机编号 */
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

/* Motor_SetSingle 内部用的方向常量 */
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 0
#define MOTOR_STOPS 2

/*============================================================
 * API
 *============================================================*/

/** 初始化 — 停止两路电机, 设方向引脚为低 */
void Motor_Init(void);

/**
 * @brief 控制单路电机速度 & 方向
 * @param motor  MOTOR_LEFT / MOTOR_RIGHT
 * @param speed  正值=正转, 负值=反转, 0=惰行停止
 *               范围: -1600 ~ +1600 (对应 0~100% 占空比)
 */
void Motor_Setduty(uint8_t motor, int16_t speed);

/** 两路同时惰行停止 (IN1=L, IN2=L, 电机可自由转动) */
void Motor_Stop(void);

/** 两路同时短路刹车 (IN1=H, IN2=H, 电机锁轴有阻力) */
void Motor_Brake(void);

#endif
