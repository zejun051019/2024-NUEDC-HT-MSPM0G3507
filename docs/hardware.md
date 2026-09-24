# 硬件与外设说明

## 平台

- MCU：TI MSPM0G3507，Cortex-M0+。
- 工程：Code Composer Studio 工程；配置中的工具版本见仓库根目录 `.cproject` 与 [README](../README.md)。
- 外设依据：`mpu6050-oled-hardware-i2c.syscfg`。下表记录 SysConfig 中的分配，实际板卡版本和线束仍需现场核对。

## SysConfig 外设映射

| 资源 / 信号 | SysConfig 分配 | 用途（按配置命名） |
| --- | --- | --- |
| `I2C_MPU6050` | I2C0，SDA PA28，SCL PA31 | MPU6050 I²C 控制器 |
| `I2C_OLED` | I2C1，SDA PB3，SCL PB2 | OLED 硬件 I²C |
| `PWM_0` | TIMG6，CCP0 PB6，CCP1 PB7 | 双通道 PWM 输出 |
| `Timer_Speed` | TIMG7，周期 20 ms | 编码器速度计算与运行态 PID 调用入口 |
| `Timer_10ms` | TIMG0，周期 10 ms | 定时中断服务 |
| `E1A/E1B` | PA16 / PA17 | 编码器通道输入 |
| `E2A/E2B` | PB0 / PB1 | 编码器通道输入 |
| `S1`–`S8` | PA14、PA15、PA21、PA22、PA24–PA27 | 八路灰度输入 |
| 按键组 | PB21、PB10、PB11、PB14、PB19 | GPIO 输入；具体按键号以代码与目标板接线共同确认 |
| `BUZZER` / `ALED` | PB17 / PB18 | 蜂鸣器与指示灯 GPIO |
| PWM 相关控制 GPIO | PA8、PA9、PA0、PA1、PA11 | 以 `.syscfg` 信号命名和电路板接线为准 |

此表是配置快照而非通用接线图。工程存在多个板卡/线束版本时，应结合原理图和目标板丝印复核；不要仅凭 PA/PB 管脚表直接接线。

## MPU6050 I²C 适配

`Drivers/MPU6050/mspm0_i2c.c` 在发送/接收过程包含 10 ms 超时检测；超时后尝试将控制器切换为 GPIO 并脉冲 SCL（最多 100 个周期），检查 SDA 后再恢复 I²C 外设。总线解锁包含同步延时，因此不能将其描述为非阻塞操作；该机制也不构成对传感器、供电、上拉或接线故障的完整诊断。

## 电气与上板检查

1. 核对 MSPM0G3507 板卡型号、供电电压、共地关系与电机驱动电源。
2. 断电状态下按目标板原理图检查 PWM、方向、编码器、灰度传感器和 I²C 连线。
3. 上电前确认车轮架空或底盘已固定，具备可立即断电的措施。
4. 首次运行不要默认电机方向、编码器极性、灰度阈值与传感器地址符合其他版本；先按项目既有流程低风险确认。
