#ifndef DELAY_H
#define DELAY_H

#include "headfile.h"
/* delay_ms 由 oled_hardware_i2c.c 定义 (内部调用 mspm0_delay_ms) */
void delay_ms(uint32_t ms);

#endif // DELAY_H