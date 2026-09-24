# 2024 年全国大学生电子设计竞赛 H 题

基于 MSPM0G3507 的嵌入式控制工程。仓库保留比赛工程的 CCS 项目配置、SysConfig 配置和应用源码；不包含本机生成的编译产物及 IDE 缓存。

> 本仓库整理了项目源码与工程结构。此次整理没有修改控制参数或控制逻辑，也没有替代实机复测；功能表现仍以原项目的实测结果及后续复测为准。

## 工程组成

| 路径 | 职责 |
| --- | --- |
| main.c、headfile.h | 系统初始化、主循环与应用接口汇总 |
| User/ | 控制与运动学、编码器、循迹传感器、按键、电机、PID 和延时等应用模块 |
| Drivers/MPU6050/ | MPU6050、DMP 与 MSPM0 I²C 适配 |
| Drivers/OLED_Hardware_I2C/ | OLED 硬件 I²C 显示驱动与字库 |
| Drivers/MSPM0/ | MSPM0 时钟与中断相关代码 |
| mpu6050-oled-hardware-i2c.syscfg | SysConfig 外设配置源文件 |
| .project、.cproject、.ccsproject | Code Composer Studio 工程与构建配置 |
| targetConfigs/ | MSPM0G3507 调试目标配置 |

总体执行链为：板级与外设初始化 → 传感器和输入采集 → 运动状态/控制计算 → 电机输出与 OLED 状态显示。模块保持现有 C 接口和调用方式，整理重点是源码格式与仓库边界，不改变比赛行为。

## 硬件与软件环境

- MCU：TI MSPM0G3507，Cortex-M0+
- IDE / 构建：Code Composer Studio（CCS）项目
- TI Arm Clang：工程当前配置为 4.0.4.LTS
- MSPM0 SDK：工程当前配置为 2.10.0.04
- SysConfig：工程当前配置为 1.27.0
- 外设与引脚：以仓库内 SysConfig 配置为准；更换板卡或接线时请先核对实际硬件

上述版本来自当前 .cproject 配置。构建机需单独安装 TI 工具链与 SDK；仓库不打包厂商 SDK。

## 在 CCS 中构建

1. 安装与工程配置匹配的 CCS、TI Arm Clang、MSPM0 SDK 和 SysConfig。
2. 在 CCS 中导入本仓库根目录的 CCS 工程。
3. 检查 CCS 产品依赖是否已指向本机安装的 SDK 与 SysConfig。
4. 由 CCS 根据 .syscfg 生成配置代码，再执行 Debug 配置构建。
5. 烧录前确认目标器件、调试器和板卡接线与当前硬件一致。

Debug/、.settings/、.vscode/ 等本机生成或个人 IDE 状态不纳入版本控制。若 CCS 未自动生成配置代码，请检查 SysConfig 产品安装和工程产品依赖，不要手工编辑生成文件。

## 代码风格

- C 源码使用仓库提供的 .clang-format。
- .editorconfig 统一文本文件的缩进与换行约定。
- 格式整理不包含 Debug/ 中的 SysConfig 生成代码和编译输出。

## 验证范围

- 工程源码和配置已按原工程保留；编译验证与实机验证是不同关口。
- 本次发布整理不代表重新完成整车/赛题实机验收。
- 首次在新环境构建后，请在目标板上按原流程验证传感器、执行器、按键与整题运行表现。
