#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include "headfile.h"

// 里程计结构体：记录小车在真实世界中跑了多远
typedef struct
{
    float distance_L;      // 左轮累计行程 (cm)
    float distance_R;      // 右轮累计行程 (cm)
    float distance_center; // 车体中心点累计行程 (cm) -> 盲走定距最核心的数据！
} Odometry_TypeDef;

// 对外暴露的全局变量
extern Odometry_TypeDef Car_Odom;

// 模块初始化
void Kinematics_Init(void);

// 核心更新函数：丢入本周期左右轮产生的脉冲数 (来自 encoder_pulse_delta[])
void Kinematics_Update(float pulse_L, float pulse_R);

// 里程清零：每次到达节点（比如到达 A、B、C、D 点时）必须调用！
void Kinematics_Clear_Distance(void);

#endif // __KINEMATICS_H