#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include "headfile.h"

extern uint8_t enable_group1_irq;
extern volatile uint8_t g_key_t1; /* T1: Task1 */
extern volatile uint8_t g_key_t2; /* T2: Task2 */
extern volatile uint8_t g_key_t3; /* T3: Stop */
extern volatile uint8_t g_key_t4; /* T4: 声光测试 */

/* 声光提示: 满分代码 shengguang_flag 模式 */
extern uint8_t g_alarm_flag;  /* 1=蜂鸣+LED亮, 0=关 */
extern uint8_t g_alarm_timer; /* 10ms 倒计时 */

void Interrupt_Init(void);
void ISR_Enable(void);

#endif
