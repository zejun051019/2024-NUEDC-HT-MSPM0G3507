#include "headfile.h"

#define PWM_MAX 1600 // PWM 周期上限
#define IOUT_MAX 800 // 积分项上限 (防积分饱和)
#define IOUT_MIN -800

pid_t motorA;
pid_t motorB;
pid_t angle;
pid_t trace_pid;

// ======================== PID 核心 ========================

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
    pid->pid_mode = mode;
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->target = 0;
    pid->now = 0;
    pid->error[0] = 0;
    pid->error[1] = 0;
    pid->error[2] = 0;
    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out = 0;
}

/**
 * @brief   速度闭环核心 — 编码器脉冲差分 → 增量式 PID → PWM 寄存器
 *
 * @details 本函数运行在 Timer_Speed_INST_IRQHandler (TIMG7) 的 ISR 上下文中，
 *          固定 20ms 周期触发，与 Encoder_SpeedCalc() 同一中断处理函数内
 *          顺序执行，确保 E_speed[] 数据消费不发生跨周期竞争。
 *
 *          数据通路（单周期内）：
 *          GPIO 编码器中断 → E_cnt[]（硬件计数器，上升沿触发）
 *          → Encoder_SpeedCalc() 差分得 E_speed[]（RPM + mm/s）
 *          → pid_control() 以 E_speed[n] 为 now, motorX.target 为参考
 *          → pid_cal() 增量式 PID 得 out（自带累加记忆）
 *          → pidout_limit() 钳位 ±PWM_MAX（=1600，对应 100% 占空比）
 *          → Motor_Setduty() 写 TIMG6 CCP0/CCP1 寄存器（立即更新模式）
 *
 * @note    增量式 PID 的 out 自带历史累加，不依赖独立 iout 记忆。
 *          目标速度归零后 out 保持当前值不会瞬间回零，
 *          需调用方 Motor_Brake() 硬件短路刹车主动消除驱动。
 *
 * @warning 故障场景 — 编码器断线或 GPIO 中断失效：
 *          E_speed 为零 → PID 误差恒为正 → iout 持续累加
 *          → pidout_limit 钳位 ±1600（全速输出）。
 *          系统不会失控但行为异常，依赖调用方 Track_Stop()
 *          将 target 置零 + Motor_Brake() 硬件锁定才能消除。
 *
 *          ISR 执行时间约 50μs，远小于 20ms 周期，时序安全。
 *          严禁在此上下文中插入阻塞式 I2C 或软件延时，
 *          否则将导致定时器溢出，PID 采样周期失效。
 *
 * @param   无（通过全局变量 motorA/B.now, .target, .out 传递）
 * @return  无
 */
void pid_control(void)
{
    motorA.now = E_speed[MOTOR_LEFT];
    motorB.now = E_speed[MOTOR_RIGHT];

    pid_cal(&motorA);
    pid_cal(&motorB);

    pidout_limit(&motorA);
    pidout_limit(&motorB);

    Motor_Setduty(MOTOR_LEFT, (int16_t)motorA.out);
    Motor_Setduty(MOTOR_RIGHT, (int16_t)motorB.out);
}

/**
 * @brief   通用 PID 计算（增量式 / 位置式双模）
 *
 * @details 根据 pid_mode 分两条路径：
 *
 *          DELTA_PID —— 速度环专用
 *          iout 每次覆盖（i * e0），out 自身累加。
 *          钳位时同步限制 iout 防下一周期再溢出。
 *          out 自带记忆，目标归零后保持当前值，不会回弹。
 *
 *          POSITION_PID —— 角度/循迹环专用
 *          iout 独立累加（iout += i * e0），out 每次重算。
 *          本分支无钳位，调用者（pid_cal_angle/trace）
 *          需自行确保输出不会失控。
 *
 * @note    两种模式下 error[] 移位逻辑一致：
 *          error[1] → error[2], error[0] → error[1]
 *          每次调用后完成，为下周期差分做准备。
 *
 * @warning 调用者必须保证 pid->now 在本周期已更新。
 *          复用旧 now → 增量式积分重复注入，位置式 iout 额外累加。

 * @param   [in/out]  pid   PID 实例指针
 *                       pid->now  必须在本周期已更新（物理量纲：mm/s 或 °）
 *                       pid->target  目标值（同一量纲）
 *                       pid->pid_mode  DELTA_PID（0）或 POSITION_PID（1）

 * @return 无（计算结果写入 pid->out，调用者自行消费）
 */

void pid_cal(pid_t *pid)
{
    pid->error[0] = pid->target - pid->now;

    if (pid->pid_mode == DELTA_PID)
    {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
        pid->out += pid->pout + pid->iout + pid->dout;
        /* 抗积分饱和: 超限时钳位, 限制 iout 防下次越界 */
        if (pid->out > PWM_MAX)
        {
            pid->out = PWM_MAX;
            pid->iout = IOUT_MAX;
        }
        if (pid->out < -PWM_MAX)
        {
            pid->out = -PWM_MAX;
            pid->iout = IOUT_MIN;
        }
    }
    else // POSITION_PID
    {
        pid->pout = pid->p * pid->error[0];
        pid->iout += pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        pid->out = pid->pout + pid->iout + pid->dout;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

/*
 * 角度环 PID — 使用 Yaw_error_zzk 处理 ±180° 环绕问题
 */
void pid_cal_angle(pid_t *pid)
{
    pid->error[0] = Yaw_error_zzk(pid->target, pid->now);

    if (pid->pid_mode == DELTA_PID)
    {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
        pid->out += pid->pout + pid->iout + pid->dout;
        if (pid->out > PWM_MAX)
        {
            pid->out = PWM_MAX;
            pid->iout = IOUT_MAX;
        }
        if (pid->out < -PWM_MAX)
        {
            pid->out = -PWM_MAX;
            pid->iout = IOUT_MIN;
        }
    }
    else
    {
        pid->pout = pid->p * pid->error[0];
        pid->iout += pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        pid->out = pid->pout + pid->iout + pid->dout;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

/*
 * 循迹 PID — 直接差值 (线位置偏差), 不做环绕
 */
void pid_cal_trace(pid_t *pid)
{
    pid->error[0] = pid->target - pid->now;

    if (pid->pid_mode == DELTA_PID)
    {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
        pid->out += pid->pout + pid->iout + pid->dout;
        if (pid->out > PWM_MAX)
        {
            pid->out = PWM_MAX;
            pid->iout = IOUT_MAX;
        }
        if (pid->out < -PWM_MAX)
        {
            pid->out = -PWM_MAX;
            pid->iout = IOUT_MIN;
        }
    }
    else
    {
        pid->pout = pid->p * pid->error[0];
        pid->iout += pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        pid->out = pid->pout + pid->iout + pid->dout;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

// ======================== 输出限幅 ========================

void pidout_limit(pid_t *pid)
{
    if (pid->out > PWM_MAX)
        pid->out = PWM_MAX;
    if (pid->out < -PWM_MAX)
        pid->out = -PWM_MAX;
}

//目标速度设置

void motor_target_set(float spe1, float spe2)
{
    motorA.target = spe1;
    motorB.target = spe2;
}

// ======================== 角度误差计算 (航向环) ========================

/*
 * Yaw_error_zzk — 最短角度误差 (归一化到 [-180, +180])
 */
float Yaw_error_zzk(float Target, float Now)
{
    float error = Target - Now;
    while (error > 180.0f)
        error -= 360.0f;
    while (error < -180.0f)
        error += 360.0f;
    return error;
}
