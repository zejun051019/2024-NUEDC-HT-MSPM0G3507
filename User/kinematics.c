#include "kinematics.h"

// ==========================================
// 硬件物理参数配置区 (必须根据你的车实测修改！)
// ==========================================
#define PI 3.1416f

// 1. 轮子直径 (单位：cm，常见的黄轴电机轮胎是 6.5cm)
#define WHEEL_DIAMETER_CM 6.5f
// 计算出轮子周长 (转一圈走多远)
#define WHEEL_PERIMETER_CM (WHEEL_DIAMETER_CM * PI)

// 2. 轮子转整整一圈，单片机能捕获到的脉冲总数
// 假设：减速比 28，500线霍尔编码器，单相单边沿捕获 = 28 * 500 = 14000
#define PULSES_PER_REV 14000.0f

// 3. 终极转换系数：1 个脉冲等于多少厘米 (cm/pulse)
//    转一圈走多远 / 轮子转一整圈捕获的脉冲数
#define CM_PER_PULSE (WHEEL_PERIMETER_CM / PULSES_PER_REV)

// 实例化全局里程计
Odometry_TypeDef Car_Odom;

void Kinematics_Init(void)
{
    Kinematics_Clear_Distance();
}

// ==========================================
// 核心更新引擎：把脉冲积分成现实距离
// ==========================================
// pulse_L / pulse_R: 本周期内左右轮各自产生的脉冲数
//   (由 encoder.c 的 encoder_pulse_delta[] 提供,
//    在 Encoder_SpeedCalc 每个 50ms 周期更新一次)
void Kinematics_Update(float pulse_L, float pulse_R)
{
    // 1. 计算本周期内，左右轮分别走了多少厘米
    float step_L_cm = pulse_L * CM_PER_PULSE;
    float step_R_cm = pulse_R * CM_PER_PULSE;

    // 2. 积分累加：更新总行程
    Car_Odom.distance_L += step_L_cm;
    Car_Odom.distance_R += step_R_cm;

    // 3. 计算车体中心里程 (左右轮里程的平均值，最能代表车身的真实位移)
    Car_Odom.distance_center = (Car_Odom.distance_L + Car_Odom.distance_R) / 2.0f;
}

// ==========================================
// 里程清零：每段新任务开始前调用
// ==========================================
void Kinematics_Clear_Distance(void)
{
    Car_Odom.distance_L = 0.0f;
    Car_Odom.distance_R = 0.0f;
    Car_Odom.distance_center = 0.0f;
}
