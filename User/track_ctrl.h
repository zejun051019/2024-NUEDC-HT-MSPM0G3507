#ifndef _TRACK_CTRL_H_
#define _TRACK_CTRL_H_

#include "headfile.h"

/* 速度 */
#define SPEED_STRAIGHT 500.0f /* 盲走 mm/s */
#define SPEED_ARC 400.0f      /* 弧线 mm/s */

/* 角度环 */
#define ANGLE_P 17.0f
#define ANGLE_D 5.0f
#define ANGLE_K 0.5f

/* 循迹环 */
#define TRACE_P 13.0f
#define TRACE_D 4.0f
#define TRACE_K 1.0f

/* 声光提示 (满分代码模式: 宏 + 定时器标志位) */
#define BUZZER_ON DL_GPIO_setPins(ALARM_PORT, ALARM_BUZZER_PIN)
#define BUZZER_OFF DL_GPIO_clearPins(ALARM_PORT, ALARM_BUZZER_PIN)
#define ALED_ON DL_GPIO_setPins(ALARM_PORT, ALARM_ALED_PIN)
#define ALED_OFF DL_GPIO_clearPins(ALARM_PORT, ALARM_ALED_PIN)
#define ALARM_ON                                                                                   \
    do                                                                                             \
    {                                                                                              \
        BUZZER_ON;                                                                                 \
        ALED_ON;                                                                                   \
    } while (0)
#define ALARM_OFF                                                                                  \
    do                                                                                             \
    {                                                                                              \
        BUZZER_OFF;                                                                                \
        ALED_OFF;                                                                                  \
    } while (0)

/* 线检测 */
#define BLACK_THRES 15 /* 弧线连续黑: 确认在线 */
#define WHITE_THRES 5  /* 弧线连续白: 确认丢线 */
#define HIT_THRES 1    /* 盲走撞线: 连续黑 N 次 */

/* 距离保护 (防止弧线中途误判丢线 / 盲走误判撞线) */
#define ARC_DIST_MIN 94.5f /* 弧线最低距离(cm), π×40≈125.6, 取73% */

/* Task3: 8字型对角线盲走 */
#define SPEED_TASK3 400.0f
#define DIAG_DIST_PHASE1_T3 80.0f /* Phase1 距离(cm) */
#define DIAG_ANGLE_AC1_T3 -58.0f  /* A→C Phase1: 大角度右下 */
#define DIAG_ANGLE_AC2_T3 0.0f    /* A→C Phase2: 回正车头 */
#define DIAG_ANGLE_BD1_T3 -126.0f /* B→D Phase1: 大角度左下 */
#define DIAG_ANGLE_BD2_T3 -180.0f /* B→D Phase2: 完全反方向 */

/* Task4 对角线参数 (独立于Task3, 可单独调整) */
#define DIAG_DIST_PHASE1_AC_T4 74.0f
#define DIAG_DIST_PHASE1_BD_T4 65.0f
#define DIAG_ANGLE_AC1_T4 -61.0f //绝对值调小就是往赛道中心调
#define DIAG_ANGLE_AC2_T4 0.0f
#define DIAG_ANGLE_BD1_T4 -120.0f //绝对值调大就是往赛道中心调
#define DIAG_ANGLE_BD2_T4 -180.0f

/* Task4: 8字型 × 4圈连续多圈抗扰测试 */
#define TOTAL_LAPS 4       /* 连续跑 4 圈 */
#define SPEED_TASK4 750.0f /* Task4 基础速度(同Task3保证稳定性, 可酌情提速) */

/* ── Task4 专用 PID 参数 (独立于Task1-3, 提速后只改此处) ── */
/* 电机速度环 */
#define MOTOR_P_T4 5.5 /* 等速下同Task3, 提速后若轮速跟不上先动此项 */
#define MOTOR_I_T4 0.45f
#define MOTOR_D_T4 0.50f
/* 角度环 (对角线航向保持) */
#define ANGLE_P_T4 20.0f /* 同Task3起点, 提速后逐步加大 */
#define ANGLE_D_T4 5.0f
/* 循迹环 (弧线跟踪) */
#define ANGLE_K_T4 0.5
#define TRACE_P_T4 15.5f /* 同Task3起点, 提速后逐步加大 */
#define TRACE_D_T4 5.75f
#define TRACE_K_T4 1.0f

/* 全局 */
extern float g_base_speed;
extern bool g_running;
extern uint8_t g_task;
extern uint8_t g_workstep;
extern uint8_t g_black_cnt;
extern uint8_t g_white_cnt;
extern float g_ref_yaw; /* 基准航向 (A点) */
extern uint8_t g_grace; /* 撞线保护: 弧线出来后 N 周期内不检测线 */
extern uint8_t g_ramp;  /* 速度爬坡: 剩余步数, 出弯后平滑加速 */
extern uint8_t g_lap;   /* Task4: 当前圈数 (0起始) */

void Tracking_SystemInit(void);
void Task1_Start(void);
void Task2_Start(void);
void Task3_Start(void);
void Task4_Start(void); /* 8字型 × 4圈连续多圈 */
void Tracking_Stop(void);
void Alarm_Beep(void);           /* 声光 300ms */
void Alarm_Beep_ms(uint16_t ms); /* 声光 可调时长 */
void PID_Reset(void);            /* 全部清零 (启停用) */
void PID_Reset_Upper(void);      /* 只清角度+循迹 (模式切换用, 不动速度环) */

/* 封装的核心函数 */
bool Straight_Run(void); /* 盲走: 角度环航向保持 + 撞线, 返回 true=撞线 */
bool Arc_Run(void);      /* 循迹: 灰度 PID 循迹 + 丢线, 返回 true=丢线 */

#endif
