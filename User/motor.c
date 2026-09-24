#include "headfile.h"

/* 与 SysConfig → PWM_0 → timerCount 对齐 */
#define PWM_PERIOD 1600

static void Motor_SetSingle(uint8_t motor, uint8_t dir, uint16_t speed)
{
    if (speed > PWM_PERIOD)
        speed = PWM_PERIOD;

    if (motor == MOTOR_LEFT)
    {
        switch (dir)
        {
        case MOTOR_FORWARD:
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN2_PIN);
            break;
        case MOTOR_BACKWARD:
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN1_PIN);
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN2_PIN);
            break;
        default: /* MOTOR_STOP: IN1=L, IN2=L */
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN2_PIN);
            speed = 0;
            break;
        }
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C0_IDX);
    }
    else /* MOTOR_RIGHT */
    {
        switch (dir)
        {
        case MOTOR_FORWARD:
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN2_PIN);
            break;
        case MOTOR_BACKWARD:
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN1_PIN);
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN2_PIN);
            break;
        default:
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN2_PIN);
            speed = 0;
            break;
        }
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C1_IDX);
    }
}

void Motor_Init(void)
{
    Motor_Stop();
}

/* speed: +=正转, -=反转, 0=惰行停止, speed范围 ±1600 */
void Motor_Setduty(uint8_t motor, int16_t duty)
{
    if (duty > 0)
        Motor_SetSingle(motor, MOTOR_FORWARD, (uint16_t)duty);
    else if (duty < 0)
        Motor_SetSingle(motor, MOTOR_BACKWARD, (uint16_t)(-duty));
    else
        Motor_SetSingle(motor, MOTOR_STOPS, 0);
}

void Motor_Stop(void)
{
    Motor_SetSingle(MOTOR_LEFT, MOTOR_STOPS, 0);
    Motor_SetSingle(MOTOR_RIGHT, MOTOR_STOPS, 0);
}

void Motor_Brake(void)
{
    /* 短路刹车: 两路 IN1=IN2=H, 占空比归零 */
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
}
