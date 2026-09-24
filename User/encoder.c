#include "headfile.h"
/*========================== 常量 ==========================*/

/* 一圈总计数 = 500PPR × 1(1x倍频) × 28减速比 */
#define CPR 14000
/* RPM = delta / CPR × 60 / 0.05 = delta × 1200 / CPR / 60s-min */
#define RPM_FACTOR (3000.0f / (float)CPR)

static volatile int32_t E_cnt[2]; /* 脉冲计数 */
static int32_t E_last[2];         /* 上次采样值(用于差值法) */
float E_rpm[2];   /* 当前转速(RPM) 是一个角速度的量度，单位是 转/分钟。 */
float E_speed[2]; /* 当前线速度(mm/s) */
static int32_t d0;
static int32_t d1;

void Encoder_Init(void)
{
    /* 计数器清0 */
    E_cnt[0] = 0;
    E_last[0] = 0;
    E_rpm[0] = 0.0f;
    E_speed[0] = 0.0f;
    E_cnt[1] = 0;
    E_last[1] = 0;
    E_rpm[1] = 0.0f;
    E_speed[1] = 0.0f;
    d0 = 0;
    d1 = 0;

    /* 编码器 GPIO 中断使能 */
    DL_GPIO_enableInterrupt(ENCODER_E1A_PORT, ENCODER_E1A_PIN);
    NVIC_ClearPendingIRQ(ENCODER_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_GPIOA_INT_IRQN);

    DL_GPIO_enableInterrupt(ENCODER_E2A_PORT, ENCODER_E2A_PIN);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);

    /* 启动 50ms 测速定时器 */
    DL_TimerG_enableInterrupt(Timer_Speed_INST, DL_TIMER_IIDX_ZERO);
    DL_TimerG_clearInterruptStatus(Timer_Speed_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(Timer_Speed_INST_INT_IRQN);
    DL_TimerG_startCounter(Timer_Speed_INST);
}

void Encoder_LeftA_IRQ(void)
{
    if (DL_GPIO_readPins(ENCODER_E1B_PORT, ENCODER_E1B_PIN))
        E_cnt[0]++; /* B=高 → B领先A → 反转 */
    else
        E_cnt[0]--; /* B=低 → A领先B → 正转 */
}

void Encoder_RightA_IRQ(void)
{
    if (DL_GPIO_readPins(ENCODER_E2B_PORT, ENCODER_E2B_PIN))
        E_cnt[1]--; /* B=高 → 反转 */
    else
        E_cnt[1]++; /* B=低 → 正转 */
}

void Encoder_SpeedCalc(void)
{
    /* 清中断标志 */
    DL_TimerG_clearInterruptStatus(Timer_Speed_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    /* 左电机: 差值 → RPM */
    d0 = E_cnt[0] - E_last[0];
    E_last[0] = E_cnt[0];
    E_rpm[0] = (float)d0 * RPM_FACTOR;
    /* 右电机 */
    d1 = E_cnt[1] - E_last[1];
    E_last[1] = E_cnt[1];
    E_rpm[1] = (float)d1 * RPM_FACTOR;
    //计算速度
    /*
     * 线速度 (mm/s): d/14000=转数 × π×65mm=每转距离 × 20 (1/0.05s) x 50(1/0.02)
     */
    E_speed[0] = (float)d0 / ENC_CNT_ONE_REV * PI * MOTOR_WHEEL_DIA * 50.0f; // 20→50
    E_speed[1] = (float)d1 / ENC_CNT_ONE_REV * PI * MOTOR_WHEEL_DIA * 50.0f;

    /* 里程计更新: 脉冲差值 → 行驶距离(cm) */
    Kinematics_Update((float)d0, (float)d1);
}

void Encoder_PrintStatus(void)
{
    char buf[128];
    sprintf(buf, "L:%+5d %+5.1fRPM %5.1fmm/s | R:%+7d %+7.1fRPM %5.1fmm/s\r\n", d0, E_rpm[0],
            E_speed[0], d1, E_rpm[1], E_speed[1]);
    //uart_send_string(PRINT_INST,buf);
}
