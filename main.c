#include "ti_msp_dl_config.h"
#include "headfile.h"

/* ══════════════════ main ══════════════════ */
int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    delay_ms(50);
    MPU6050_Init();

    /* OLED 先初始化, 显示7校准进度 */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"MPU Calibrating", 8);
    OLED_ShowString(0, 2, (uint8_t *)"Wait 8-10s...", 8);

    Interrupt_Init(); /* 必须在 MPU6050_Init 之后, 才会使能 GPIOB 中断 */

    Tracking_SystemInit();

    uint8_t oled_buf[16];
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task 0", 16);
    sprintf((char *)oled_buf, "Yaw:%5.1f", (double)yaw);
    OLED_ShowString(0, 2, oled_buf, 16);

    if (!enable_group1_irq)
        OLED_ShowString(0, 4, (uint8_t *)"MPU FAIL", 8);

    static uint8_t yaw_tick = 0;
    static uint8_t g_key_lockout = 0; /* 发车锁定期: 防按键回弹误触发停车 */

    while (1)
    {
        /* 锁定期每 10ms 减 1 (300ms = 30 个主循环) */
        if (g_key_lockout)
            g_key_lockout--;

        /* ── 四键扫描 ── */
        uint8_t key = Key_Scan();
        if (key == 1)
        {
            if (g_running && g_key_lockout == 0)
                Tracking_Stop();
            if (!g_running)
            {
                Task1_Start();
                g_key_lockout = 30;
                Alarm_Beep_ms(200);
                OLED_Clear();
                OLED_ShowString(0, 0, (uint8_t *)"Task 1", 16);
                OLED_ShowString(0, 2, (uint8_t *)"Yaw:", 16);
            }
        }
        else if (key == 2)
        {
            if (g_running && g_key_lockout == 0)
                Tracking_Stop();
            if (!g_running)
            {
                Task2_Start();
                g_key_lockout = 30;
                Alarm_Beep_ms(200);
                OLED_Clear();
                OLED_ShowString(0, 0, (uint8_t *)"Task 2", 16);
                OLED_ShowString(0, 2, (uint8_t *)"Yaw:", 16);
            }
        }
        else if (key == 3)
        {
            if (g_running && g_key_lockout == 0)
                Tracking_Stop();
            if (!g_running)
            {
                Task3_Start();
                g_key_lockout = 30;
                Alarm_Beep_ms(200);
                OLED_Clear();
                OLED_ShowString(0, 0, (uint8_t *)"Task 3", 16);
                OLED_ShowString(0, 2, (uint8_t *)"Yaw:", 16);
            }
        }
        else if (key == 4)
        {
            if (g_running && g_key_lockout == 0)
                Tracking_Stop();
            else if (!g_running)
            {
                Task4_Start();
                g_key_lockout = 30;
                Alarm_Beep_ms(200);
                OLED_Clear();
                OLED_ShowString(0, 0, (uint8_t *)"Task 4", 16);
                OLED_ShowString(0, 2, (uint8_t *)"Yaw:", 16);
            }
        }

        /* 每 300ms 整行刷新 yaw */
        if (++yaw_tick >= 30)
        {
            yaw_tick = 0;
            uint8_t oled_buf[16];
            sprintf((char *)oled_buf, "Yaw:%5.1f", (double)yaw);
            OLED_ShowString(0, 2, oled_buf, 16);
        }

        /* ── 任务调度 (互斥: else if) ── */
        if (g_running && g_task == 1)
            Task1_Loop();
        else if (g_running && g_task == 2)
            Task2_Loop();
        else if (g_running && g_task == 3)
            Task3_Loop();
        else if (g_running && g_task == 4)
            Task4_Loop();

        delay_ms(2);
    }
}

void Timer_Speed_INST_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(Timer_Speed_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    Encoder_SpeedCalc();
    if (g_running)
        pid_control();
}
