# 软件架构与任务调用

## 范围

本文依据仓库当前 `main.c`、`User/`、`Drivers/` 和 `.syscfg` 梳理微控制器端结构。它描述源码调用关系，不表示各模块是并发 RTOS 任务，也不代替实机时序测量。

## 启动与运行链

```text
main()
  ├─ SYSCFG_DL_init / SysTick / MPU6050 / OLED 初始化
  ├─ Interrupt_Init / Tracking_SystemInit
  └─ 前台循环
       ├─ 扫描按键并选择 Task 1–4 的启动/停止入口
       ├─ 低频刷新 OLED 航向与任务状态
       ├─ 按 g_running、g_task 互斥调用对应 Task*_Loop
       └─ 短延时后进入下一轮

Timer_Speed ISR（由 SysConfig 配置为 20 ms）
  ├─ Encoder_SpeedCalc()
  └─ g_running 时调用 pid_control()

GPIO / SysTick / 10 ms timer ISR
  ├─ GPIO：处理 MPU6050 数据就绪、编码器边沿和按键事件
  ├─ SysTick：递增毫秒时基
  └─ 10 ms timer：运行态 LED 翻转与蜂鸣器超时关闭
```

中断函数及其数据访问应以 `Drivers/MSPM0/interrupt.c`、`main.c` 和生成配置逐项核实。重构时应保持初始化先后、任务互斥、定时器控制节拍和现有中断/前台交互不变。

## 模块职责

| 模块 | 责任 | 主要关系 |
| --- | --- | --- |
| `main.c` | 初始化系统、扫描任务按键、路由任务启动/停止、前台互斥调度与 OLED 刷新 | 调用任务入口、跟踪初始化和显示 API |
| `User/control.c` | 实现 Task 1–4 的阶段状态推进与阶段切换 | 调用直行、弧线跟踪、运动学清零、电机及 PID 辅助接口 |
| `User/track_ctrl.c` | 初始化跟踪状态、提供直行/弧线控制和任务启停入口 | 使用灰度、航向、里程和运动控制状态 |
| `User/pid.c` | 根据目标速度与反馈更新电机控制量 | 由速度定时器中断路径调用 |
| `User/encoder.c`、`User/kinematics.c` | 编码器计数/速度及车辆运动量相关计算 | 为速度闭环与路线阶段提供反馈 |
| `User/motor.c` | 电机方向、PWM、停车/制动接口 | 接收控制模块输出 |
| `User/key.c`、灰度相关模块 | 读取人机输入与循迹传感器 | 为主循环与路线控制提供输入 |
| `Drivers/MPU6050/` | MPU6050/DMP 访问及 I²C 适配 | 启动阶段初始化，GPIO/数据事件路径读取 |
| `Drivers/OLED_Hardware_I2C/` | OLED 命令、绘制和刷新 | 前台显示调用 |
| `Drivers/MSPM0/` | 时钟、中断及板级支持 | 提供 MCU 运行基础 |

## 行为保护边界

- Task 1–4 的状态转移、补偿常量、速度/PID 数据和运行顺序属于功能行为；文档整理不对其作数值重写。
- 控制器状态存在跨文件访问时，不能仅为“减少全局变量”而改变其生命周期或更新时序。
- 中断中不应增加阻塞串口、OLED 刷新或其他长耗时操作。
- `.syscfg` 是外设管脚和定时器配置的来源；生成文件不应手工编辑。
