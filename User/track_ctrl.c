#include "track_ctrl.h"

float g_base_speed = SPEED_STRAIGHT;
bool g_running = false;
uint8_t g_task = 0;
uint8_t g_workstep = 0;
uint8_t g_black_cnt = 0;
uint8_t g_white_cnt = 0;
float g_ref_yaw = 0.0f;
uint8_t g_grace = 0; /* 弧线出弯后保护周期 */
uint8_t g_ramp = 0;  /* 速度爬坡剩余步数 */
uint8_t g_lap = 0;   /* Task4 圈数计数 */

uint8_t g_alarm_flag = 0; /* 1=触发, 10ms ISR 计时关 */
uint8_t g_alarm_timer = 0;

void Tracking_SystemInit(void)
{
    Motor_Init();
    Encoder_Init();
    Kinematics_Init();

    // pid_init(&motorA, DELTA_PID, 3.5f, 1.0f, 0.0f);
    // pid_init(&motorB, DELTA_PID, 3.5f, 1.0f, 0.0f);

    pid_init(&motorA, DELTA_PID, 3.5f, 0.4f, 0.5f); // Ki降,加Kd
    pid_init(&motorB, DELTA_PID, 3.5f, 0.4f, 0.5f);

    pid_init(&angle, POSITION_PID, ANGLE_P, 0.0f, ANGLE_D);
    pid_init(&trace_pid, POSITION_PID, TRACE_P, 0.0f, TRACE_D);
    trace_pid.target = 0.0f;

    ISR_Enable();
}

void PID_Reset(void)
{
    motorA.error[0] = 0;
    motorA.error[1] = 0;
    motorA.error[2] = 0;
    motorA.out = 0;
    motorA.iout = 0;
    motorB.error[0] = 0;
    motorB.error[1] = 0;
    motorB.error[2] = 0;
    motorB.out = 0;
    motorB.iout = 0;
    angle.error[0] = 0;
    angle.error[1] = 0;
    angle.error[2] = 0;
    angle.out = 0;
    angle.iout = 0;
    trace_pid.error[0] = 0;
    trace_pid.error[1] = 0;
    trace_pid.error[2] = 0;
    trace_pid.out = 0;
    trace_pid.iout = 0;
}

/* 只清角度+循迹, 不动速度环 (模式切换用, 保证速度平滑过渡) */
void PID_Reset_Upper(void)
{
    angle.error[0] = 0;
    angle.error[1] = 0;
    angle.error[2] = 0;
    angle.out = 0;
    angle.iout = 0;
    trace_pid.error[0] = 0;
    trace_pid.error[1] = 0;
    trace_pid.error[2] = 0;
    trace_pid.out = 0;
    trace_pid.iout = 0;
}

/**
 * @brief   盲走直线段：航向角度环闭环 + 灰度撞线检测
 *
 * @details 本函数运行在 main loop 调度上下文（~2ms/次），
 *          与 20ms ISR 速度环构成前后台零阶保持模型。
 *
 *          控制逻辑：
 *          (1) 读取 MPU6050 DMP 解算的全局变量 yaw（硬件三轴陀螺仪，
 *              通过 I2C + GPIO 中断更新，周期约 10ms）。
 *          (2) pid_cal_angle() 计算航向偏差 → 差速纠偏量 s。
 *              偏差 < 1.5° 时强制死区（s = 0），防止角度量化噪声
 *              引发 PWM 高频抖动。
 *          (3) 左 target = g_base_speed - s，右 target = g_base_speed + s，
 *              限幅 [0, +∞) 后写入 motorA/B.target，供 ISR 速度环消费。
 *          (4) 灰度传感器 8 路 GPIO 并行读取（sensor != 0x00 表示
 *              至少一个通道检测到黑线），HIT_THRES=1 即判定撞线。
 *
 * @note    出弯保护期 g_grace：以主循环周期（~2ms）为单位的计数缓冲。
 *          g_grace > 0 期间灰度传感器被屏蔽，g_black_cnt 强制归零，
 *          防止出弯瞬间传感器处于边界线附近产生误触发。
 *
 * @warning 航向基准依赖 MPU6050 DMP 的 yaw 值。若 DMP 初始化失败
 *          或长时间运行后 yaw 发生温漂（典型值 0.5°~3°/min），
 *          角度环将持续输出固定方向纠偏量，实际行驶轨迹为弧线，
 *          最终无法命中目标黑线。嵌套调用限深：本函数不自旋，
 *          无递归，单次执行 < 100μs，不阻塞主循环调度。
 *
 * @param   无（所有输入通过全局变量传递）
 * @return  true  = 撞线到达，调用者应执行状态跃迁
 *          false = 仍在途中，调用者继续轮询
 */
bool Straight_Run(void)
{
    /* 速度爬坡: 出弯后逐步加速, 避免猛冲 */
    float speed = g_base_speed;
    // if (g_ramp) {
    //     g_ramp--;
    //     speed = SPEED_ARC + (g_base_speed - SPEED_ARC) * (1.0f - (float)g_ramp / 30.0f);
    //     if (speed > g_base_speed) speed = g_base_speed;
    // }

    /* 出弯保护 + 距离保护: 走满最低距离才允许检测线 */
    if (g_grace)
    {
        g_grace--;
        g_black_cnt = 0;
        // /* 里程不足时只读传感器清计数, 不检测 */
        // uint8_t sensor = GraySensor_Read();
        // (void)sensor;
    }
    else
    {
        uint8_t sensor = GraySensor_Read();
        if (sensor != 0x00)
        {
            g_black_cnt++;
            if (g_black_cnt >= HIT_THRES)
            {
                g_black_cnt = 0;
                return true;
            }
        }
        else
            g_black_cnt = 0;
    }

    angle.now = yaw;
    pid_cal_angle(&angle);
    float s = angle.out * ((g_task == 4) ? ANGLE_K_T4 : ANGLE_K);

    /* 死区: 偏差 < 1° 直走, 避免小角度抖动 (满分代码方案1) */
    float err = angle.error[0];
    if (err < 0)
        err = -err;
    if (err < 1.5f)
        s = 0;

    float tL = speed - s;
    float tR = speed + s;
    if (tL < 0)
        tL = 0;
    if (tR < 0)
        tR = 0;
    motorA.target = tL;
    motorB.target = tR;
    return false;
}

/**
 * @brief   弧线循迹段：灰度传感器线位置 PID + 丢线到达检测
 *
 * @details 运行在 main loop 上下文（~2ms/次），与 Straight_Run 构成
 *          任务级状态机切换。核心区别于反馈源切换：
 *          Straight_Run 用 MPU6050 yaw（角度环），Arc_Run 用灰度
 *          传感器线位置偏差（循迹环），两个 pid_t 实例独立。
 *
 *          控制逻辑：
 *          (1) GraySensor_Read() 读取 8 路 GPIO 电平（原始位图），
 *              经 Track_GetError() 加权平均得到连续线位置偏差。
 *          (2) pid_cal_trace() 循迹位置式 PID → 差速纠偏量 s。
 *          (3) 左右 target 合成与 Straight_Run 同构。
 *
 *          退出条件（双重保护）：
 *          条件 A：g_white_cnt >= WHITE_THRES（=5，连续 ~10ms 全白）
 *          条件 B：Car_Odom.distance_center >= ARC_DIST_MIN（=94.5cm）
 *          两者同时满足才返回 true，防止弯道中途传感器短暂丢失
 *          或线宽不足导致的误退出。
 *
 * @note    g_white_cnt 清零规则：任一主循环周期读到非 0x00 即归零。
 *          所以"连续 5 次全白"要求丢线状态不可逆，持续约 10ms。
 *
 * @warning 距离保护 ARC_DIST_MIN 依赖 20ms ISR 中的 Kinematics_Update()
 *          更新里程计。若该 ISR 被阻塞（如 I2C 挂死），distance_center
 *          不更新，条件 B 永假，函数永不退出，车将无限循迹。
 *
 * @param   无
 * @return  true  = 丢线到达弧线终点，调用者应切换状态
 *          false = 仍在弧线上
 */
bool Arc_Run(void)
{
    uint8_t sensor = GraySensor_Read();
    if (sensor == 0x00)
        g_white_cnt++;
    else
        g_white_cnt = 0;

    trace_pid.now = Track_GetError(sensor);
    pid_cal_trace(&trace_pid);
    float s = trace_pid.out * ((g_task == 4) ? TRACE_K_T4 : TRACE_K);
    float tL = g_base_speed - s;
    float tR = g_base_speed + s;
    if (tL < 0)
        tL = 0;
    if (tR < 0)
        tR = 0;
    motorA.target = tL;
    motorB.target = tR;

    /* 距离保护: 走满弧线最低距离才允许丢线退出 (防弧线中途误判) */
    if (g_white_cnt >= WHITE_THRES && Car_Odom.distance_center >= ARC_DIST_MIN)
    {
        g_white_cnt = 0;
        return true;
    }
    return false;
}

/* =============== Task1: A→B 盲跑 → 撞线停车 =============== */
void Task1_Start(void)
{
    g_task = 1;
    g_running = true;
    g_workstep = 0;
    g_black_cnt = 0;
    g_grace = 0;
    g_ramp = 0;
    g_base_speed = SPEED_STRAIGHT;
    g_ref_yaw = yaw;
    angle.target = g_ref_yaw;
    Kinematics_Clear_Distance();
    PID_Reset();
    motor_target_set(g_base_speed, g_base_speed);
}

/* =============== Task2: A→B→C→D→A =============== */
void Task2_Start(void)
{
    g_task = 2;
    g_running = true;
    g_workstep = 0;
    g_black_cnt = 0;
    g_white_cnt = 0;
    g_grace = 0;
    g_ramp = 0;
    g_ref_yaw = yaw;
    angle.target = g_ref_yaw;
    Kinematics_Clear_Distance(); /* A→B 里程从零计 */
    PID_Reset();
    motor_target_set(SPEED_STRAIGHT, SPEED_STRAIGHT);
    g_base_speed = SPEED_STRAIGHT;
}

/* =============== Task3: 8字型 A→C→B→D→A (两阶段对角线) =============== */
/*
 * 策略: 每条128cm对角线分两段走
 *   Phase1(粗略靠岸): 大角度+编码器里程保护, 走到约100cm强制退出
 *   Phase2(精确捕获): 回正/反方向, 纯靠灰度传感器检测黑线精准停车
 * 目的: 即使MPU6050漂移2-3°, Phase2仍能在目标点±5cm范围内扫到黑线
 */
void Task3_Start(void)
{
    g_task = 3;
    g_running = true;
    g_workstep = 0;
    g_black_cnt = 0;
    g_white_cnt = 0;
    g_grace = 0;
    g_ramp = 0;
    g_ref_yaw = yaw; /* 只在A点锁一次, 所有角度偏移以此为准 */
    g_base_speed = SPEED_TASK3;
    angle.target = g_ref_yaw;
    Kinematics_Clear_Distance();
    PID_Reset();
    motor_target_set(g_base_speed, g_base_speed);
}

/* =============== Task4: 8字型 × 4圈 (连续多圈抗扰) =============== */
/*
 * 策略: 复用 Task3 两阶段对角线满分方案, 外加:
 *   1. 圈数循环 (4圈)
 *   2. 每圈回A点时重置 g_ref_yaw = yaw (地标航向重校准, 抑制MPU6050累积漂移)
 *   3. 每段出发前清PID上层积分 (防累积饱和)
 */
void Task4_Start(void)
{
    g_task = 4;
    g_running = true;
    g_workstep = 0;
    g_black_cnt = 0;
    g_white_cnt = 0;
    g_grace = 0;
    g_ramp = 0;
    g_lap = 0;
    g_ref_yaw = yaw;
    g_base_speed = SPEED_TASK4;
    angle.target = g_ref_yaw;
    Kinematics_Clear_Distance();
    PID_Reset();
    /* ── Task4 专用 PID 参数 (独立于Task1-3, 提速后只改 track_ctrl.h) ── */
    pid_init(&motorA, DELTA_PID, MOTOR_P_T4, MOTOR_I_T4, MOTOR_D_T4);
    pid_init(&motorB, DELTA_PID, MOTOR_P_T4, MOTOR_I_T4, MOTOR_D_T4);
    pid_init(&angle, POSITION_PID, ANGLE_P_T4, 0.0f, ANGLE_D_T4);
    pid_init(&trace_pid, POSITION_PID, TRACE_P_T4, 0.0f, TRACE_D_T4);
    trace_pid.target = 0.0f;
    motor_target_set(g_base_speed, g_base_speed);
}

void Tracking_Stop(void)
{
    Alarm_Beep_ms(200); /* 停车声光 */
    g_running = false;
    g_task = 0;
    g_workstep = 0;
    motor_target_set(0, 0);
    Motor_Brake();
    delay_ms(50);
    Motor_Stop();
    PID_Reset();
}

/* 满分代码模式: 设 flag + 定时器, 10ms ISR 自动倒计时关 */
void Alarm_Beep(void)
{
    ALARM_ON;
    g_alarm_flag = 1;
    g_alarm_timer = 30; /* 300ms 蜂鸣+LED, 10ms ISR 倒计时 */
}

/* 可调时长版本 */
void Alarm_Beep_ms(uint16_t ms)
{
    ALARM_ON;
    g_alarm_flag = 1;
    g_alarm_timer = ms / 10;
}
