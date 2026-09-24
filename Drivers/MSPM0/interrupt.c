#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "mpu6050.h"
#include "headfile.h"

uint8_t enable_group1_irq = 0;
volatile uint8_t g_key_pressed = 0; /* 旧 KEY(B21), 保留兼容 */
volatile uint8_t g_key_t1 = 0;      /* T1 按键 */
volatile uint8_t g_key_t2 = 0;      /* T2 按键 */
volatile uint8_t g_key_t3 = 0;      /* T3 按键 */
volatile uint8_t g_key_t4 = 0;      /* T4 按键: 声光测试 */

void Interrupt_Init(void)
{
    if (enable_group1_irq)
    {
        NVIC_EnableIRQ(GPIOB_INT_IRQn);
        NVIC_EnableIRQ(GPIOA_INT_IRQn); /* E1A 编码器 */
    }
}

void SysTick_Handler(void)
{
    tick_ms++;
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
    case ENCODER_GPIOA_INT_IIDX:
        switch (DL_GPIO_getPendingInterrupt(GPIOA))
        {
        case ENCODER_E1A_IIDX:
            DL_GPIO_clearInterruptStatus(ENCODER_E1A_PORT, ENCODER_E1A_PIN);
            Encoder_LeftA_IRQ();
            break;
        default:
            break;
        }
        break;

    case GPIO_MULTIPLE_GPIOB_INT_IIDX:
        switch (DL_GPIO_getPendingInterrupt(GPIOB))
        {
        case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
            DL_GPIO_clearInterruptStatus(GPIO_MPU6050_PORT, GPIO_MPU6050_PIN_MPU6050_INT_PIN);
            Read_Quad();
            break;
        case KEY_PIN_21_IIDX:
            DL_GPIO_clearInterruptStatus(KEY_PORT, KEY_PIN_21_PIN);
            g_key_pressed = 1;
            break;
        case KEY_T1_IIDX:
            DL_GPIO_clearInterruptStatus(GPIOB, KEY_T1_PIN);
            g_key_t1 = 1;
            break;
        case KEY_T2_IIDX:
            DL_GPIO_clearInterruptStatus(GPIOB, KEY_T2_PIN);
            g_key_t2 = 1;
            break;
        case KEY_T3_IIDX:
            DL_GPIO_clearInterruptStatus(GPIOB, KEY_T3_PIN);
            g_key_t3 = 1;
            break;
        case KEY_T4_IIDX:
            DL_GPIO_clearInterruptStatus(GPIOB, KEY_T4_PIN);
            g_key_t4 = 1;
            break;
        case ENCODER_E2A_IIDX:
            DL_GPIO_clearInterruptStatus(ENCODER_E2A_PORT, ENCODER_E2A_PIN);
            Encoder_RightA_IRQ();
            break;
        default:
            break;
        }
        break;

    default:
        break;
    }
}

void ISR_Enable(void)
{
    DL_TimerG_startCounter(PWM_0_INST);
    DL_TimerG_clearInterruptStatus(Timer_10ms_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerG_setLoadValue(Timer_10ms_INST, Timer_10ms_INST_LOAD_VALUE);
    NVIC_EnableIRQ(Timer_10ms_INST_INT_IRQN);
    DL_TimerG_startCounter(Timer_10ms_INST);
}

void Timer_10ms_INST_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(Timer_10ms_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    if (g_running)
        DL_GPIO_togglePins(GPIOB, LED_LED0_PIN);

    /* 声光提示定时器: 超时自动关 */
    if (g_alarm_flag && g_alarm_timer)
    {
        if (--g_alarm_timer == 0)
        {
            g_alarm_flag = 0;
            DL_GPIO_clearPins(ALARM_PORT, ALARM_BUZZER_PIN | ALARM_ALED_PIN);
        }
    }
}
