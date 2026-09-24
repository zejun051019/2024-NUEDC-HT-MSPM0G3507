#ifndef _HEADFILE_H_
#define _HEADFILE_H_

#include "ti_msp_dl_config.h"
#include "stdio.h"
#include <stdint.h>
#include <stdbool.h>

/* 原 main.h 内容直接内联 */
#include "clock.h"
#include "interrupt.h"
#include "mpu6050.h"
#include "oled_hardware_i2c.h"

#include "delay.h"
#include "motor.h"
#include "encoder.h"
#include "gray.sensor.h"
#include "kinematics.h"
#include "pid.h"
#include "track_ctrl.h"
#include "key.h"
#include "control.h"

/* 按键全局标志 (interrupt.c 置1, main 读取后清零) */
extern volatile uint8_t g_key_pressed;

#endif /* _HEADFILE_H_ */
