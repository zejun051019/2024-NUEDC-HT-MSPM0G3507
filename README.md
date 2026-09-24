# 2024 年全国大学生电子设计竞赛 H 题

基于 TI MSPM0G3507 的嵌入式小车控制固件。项目围绕循迹、编码器速度反馈、航向控制、任务状态机和 MPU6050 姿态数据组织，保留 Code Composer Studio 工程与 SysConfig 外设配置。

> 本仓库介绍的是微控制器端控制工程。本文档整理不修改控制代码、控制参数或比赛功能；仓库源码整理本身不等同于重新烧录或实机复测。实测结论以可追溯的原始记录和实际固件版本为准。

## 项目亮点

- **分层的裸机控制程序**：主循环负责按键与任务调度，独立模块承担循迹、运动学、编码器、PID、电机和传感器驱动。
- **多题目流程复用控制底座**：Task 1–4 由独立启动/运行状态流程组合直行、弧线循迹、航向保持和停车动作。
- **双轮速度反馈链路**：编码器周期采样与速度计算进入 PID 控制，再由电机驱动模块输出 PWM/方向控制。
- **传感器与故障恢复实践**：MPU6050 I²C 适配层包含传输超时检测与 SDA 总线解锁流程；OLED 使用独立硬件 I²C 外设。
- **板级配置可追溯**：引脚、定时器、PWM、I²C 与灰度/编码器输入以仓库内 `.syscfg` 为依据。

## 系统结构

```text
main.c（初始化、按键选择、主循环调度）
  ├─ Task 1–4 状态流程（User/control.c）
  │    ├─ 直行 / 弧线循迹 / 航向保持（User/track_ctrl.c）
  │    ├─ 运动学与里程（User/kinematics.c）
  │    └─ 速度控制（User/pid.c）
  ├─ 输入：灰度传感器、双路编码器、MPU6050、按键
  ├─ 输出：双电机 PWM/方向、OLED、蜂鸣器与指示灯
  └─ 周期中断：编码器速度更新及运行态 PID 调用

Drivers/
  ├─ MSPM0：时钟与中断支持
  ├─ MPU6050：传感器、DMP 与 I²C 适配
  └─ OLED_Hardware_I2C：OLED 硬件 I²C 驱动
```

主循环和中断的实际先后关系以 `main.c`、`Drivers/MSPM0/interrupt.c` 及当前 SysConfig 为准。模块文件图不代表独立 RTOS 任务。

## 题目流程与源码入口

| 任务 | 当前源码体现的流程 | 主要入口 |
| --- | --- | --- |
| Task 1 | A→B 直行段，检测到终点后停车并结束任务 | `Task1_Start` / `Task1_Loop` |
| Task 2 | A→B→C→D→A，直行段与弧线循迹段组合 | `Task2_Start` / `Task2_Loop` |
| Task 3 | 8 字路径，由对角线分段、航向目标与弧线循迹状态推进 | `Task3_Start` / `Task3_Loop` |
| Task 4 | 多圈 8 字路径，并包含分段航向补偿参数 | `Task4_Start` / `Task4_Loop` |

启动按键、互斥调度和 OLED 状态显示由 `main.c` 统一协调。具体按键接线、有效电平及外设管脚请以 [硬件与外设说明](docs/hardware.md) 及 `.syscfg` 配置核对，不应仅依赖本文推断实际接线。

## 模块导航

| 路径 | 职责 |
| --- | --- |
| `main.c`、`headfile.h` | 启动初始化、任务输入与顶层调度、应用接口汇总 |
| `User/control.c` | Task 1–4 状态流程 |
| `User/track_ctrl.c` | 直行、弧线循迹、任务启动/停止及路径状态辅助 |
| `User/pid.c`、`User/encoder.c`、`User/kinematics.c` | 速度反馈、编码器采样与运动学/里程计算 |
| `User/motor.c`、`User/key.c`、`User/gray.c` | 电机、按键和灰度输入适配 |
| `Drivers/MPU6050/` | MPU6050/DMP 与 MSPM0 I²C 适配 |
| `Drivers/OLED_Hardware_I2C/` | OLED 硬件 I²C 驱动与字库 |
| `Drivers/MSPM0/` | 时钟、中断等 MCU 支持 |
| `mpu6050-oled-hardware-i2c.syscfg` | 外设和管脚配置源文件 |

更多信息：

- [软件架构与任务调用](docs/architecture.md)
- [硬件与外设映射](docs/hardware.md)
- [构建与验证说明](docs/testing.md)

## 开发环境与构建

当前 CCS 工程配置记录为 TI Arm Clang `4.0.4.LTS`、MSPM0 SDK `2.10.0.04` 和 SysConfig `1.27.0`。这些是工程元数据中的配置，不保证与任意本机安装完全一致。仓库不打包 TI SDK。

1. 安装与工程配置兼容的 Code Composer Studio、MSPM0 SDK、SysConfig 与 TI Arm Clang。
2. 在 CCS 中导入仓库根目录工程，并检查产品依赖路径。
3. 由 CCS/SysConfig 按 `.syscfg` 配置生成所需初始化代码，再构建 `Debug` 配置。
4. 上板前复核器件、调试探针、电源和实际接线；首次构建成功不代表固件已烧录或实机功能已验证。

## 代码风格与验证边界

仓库提供 `.clang-format`、`.editorconfig` 与 `.gitattributes` 作为文本和格式约定。厂商生成文件、IDE 状态与本机编译产物不作为人工格式化目标。

此仓库快照没有在本次文档整理中重新构建、烧录或实机运行。若用于复现，请记录代码提交、工具链版本、目标板和实际测试条件；不要把历史口述结果泛化为当前快照的验证结论。验证范围见 [构建与验证说明](docs/testing.md)。
