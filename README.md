# 2025_E MSPM0G 智能车工程说明

本工程是基于 TI MSPM0G 系列、MSPM0 SDK 2.10.00.04 和 CCS Theia 的 2025_E 小车控制工程。当前 SysConfig 目标为 `MSPM0G3505`，封装为 `LQFP-64(PM)`，外设与引脚以 [empty.syscfg](empty.syscfg) 为准。

工程当前包含双电机 PWM 驱动、AB 编码器测速、8 路数字循迹、JY61P 陀螺仪、OLED 选题界面、按键启停、UART/VOFA 调参，以及 1-6 档题目模式控制。

## 工程结构

| 路径 | 说明 |
|---|---|
| [empty.c](empty.c) | 主程序入口；完成 SysConfig 初始化、OLED 初始化、PWM/编码器/定时器/UART 中断使能、按键扫描和 20 ms 控制调度 |
| [empty.syscfg](empty.syscfg) | TI SysConfig 配置文件；定义 ADC、GPIO、I2C、PWM、TIMER、UART 等外设 |
| [Hardware/board.c](Hardware/board.c) | 板级公共函数、运行中断开关、OLED 选题界面、`printf` 串口重定向、延时函数 |
| [Hardware/control.c](Hardware/control.c) | 速度环 PID、循迹差速、原地转向、角速度转向、直行航向保持和控制状态清零 |
| [Hardware/mode.c](Hardware/mode.c) | 题目模式 1-6 的状态机 |
| [Hardware/motor.c](Hardware/motor.c) | 左右电机方向控制和 PWM 输出 |
| [Hardware/encoder.c](Hardware/encoder.c) | 双路 AB 编码器中断计数和 RPM 计算 |
| [Hardware/xunji.c](Hardware/xunji.c) | 8 路数字循迹输入读取，低电平判定为黑线 |
| [Hardware/error.c](Hardware/error.c) | 循迹误差计算 |
| [Hardware/bsp_gyro.c](Hardware/bsp_gyro.c) | JY61P 陀螺仪 I2C 通信和角度读取 |
| [Hardware/oled_software_i2c.c](Hardware/oled_software_i2c.c) | SSD1306 OLED 软件 I2C 驱动 |
| [Hardware/key.c](Hardware/key.c) | A/B 按键消抖、选题和启动/急停 |
| [Hardware/uart_vofa.c](Hardware/uart_vofa.c) | UART 接收解析与 VOFA+ 数据/参数接口 |
| [Hardware/track_logic.c](Hardware/track_logic.c) | 复杂循迹图案识别：丢线、普通线、十字、锐角、环岛入口、全黑 |
| [Hardware/track_mode.c](Hardware/track_mode.c) | 增强循迹状态机，目前未接入正常 1-6 档调度 |
| [vofa.md](vofa.md) | VOFA+ 调试说明 |
| [BACK.md](BACK.md) | 备份/历史说明 |

## 运行流程

1. `main()` 调用 `SYSCFG_DL_init()` 初始化 SysConfig 生成的外设。
2. 初始化 JY61P、OLED，并启动 `PWM_0` 定时器。
3. 使能编码器中断、`TIMER_0` 20 ms 周期中断、`UART_0`/`UART_1` RX 中断。
4. 上电默认 `car_started = 0`，业务中断关闭，主循环只扫描按键并刷新 OLED 选题界面。
5. A 键只在未发车时切换 `task_mode`，范围为 1-6。
6. B 键用于启动/急停。启动时清空控制运行状态并打开业务中断；急停时立即清零左右 PWM、关闭业务中断并清空 PID/编码器状态。
7. `TIMER_0_INST_IRQHandler()` 每 20 ms 置位 `control_tick_pending`。
8. 主循环检测到控制节拍后，在 `car_started == 1` 时调用 `control()`。
9. `control()` 根据 `task_mode` 调用对应模式/控制函数，然后统一计算编码器 RPM、更新速度环 PID，并输出左右 PWM。

## 当前模式入口

当前 [Hardware/control.c](Hardware/control.c) 中的调度入口如下：

| 档位 | 当前调用 | 状态 |
|---|---|---|
| 1 | `Turn_In_Place(90)` | 临时用于原地 90 度转向调试，原 `mode_1()` 已被注释 |
| 2 | `Xunji_Speed()` | 临时用于普通循迹调试，原 `mode_2()` 已被注释 |
| 3 | `mode_3()` | 斜线/圆弧组合路线 |
| 4 | `mode_4()` | 模式 3 路线累计 3 圈 |
| 5 | `mode_5()` | 正方形路线：直行一段时间后原地转 90 度 |
| 6 | `mode_6()` | 普通循迹，左侧黑线触发短直行和左转 90 度 |

注意：`Track_Mode6Enhanced()` 已实现复杂路况识别，但当前 `control()` 会先把非法档位钳位到 1-6，因此 `default` 分支实际不会进入。若要使用增强循迹，需要显式接入某个档位，例如替换 `case 6`。

## 控制逻辑

| 功能 | 入口 | 说明 |
|---|---|---|
| 速度闭环 | `PID_Update()` | 增量式 PID，目标速度来自 `Left_Speed` / `Right_Speed`，输出最终 PWM |
| 循迹差速 | `Xunji_Speed()` | 根据 `Error_Calculate()` 得到循迹偏差，经位置式 PD 输出左右目标速度差 |
| 原地转向 | `Turn_In_Place()` | 读取 JY61P yaw，使用角度 PD 生成左右反向目标速度 |
| 角速度转向 | `Turn_In_Place_Rate()` | 角度外环生成目标角速度，角速度内环估算 yaw 差分并修正输出 |
| 航向直行 | `Keep_Angle_Straight()` | 锁定目标 yaw，根据航向误差修正左右轮目标速度 |
| 状态复位 | `control_reset_runtime_state()` | 清空速度、编码器计数、滤波缓存、PID 运行量和 yaw 缓存 |

编码器速度每 20 ms 计算一次，并经过 5 点滑动平均滤波。当前 RPM 计算按 13 线编码器、4 倍频、20:1 减速比处理。

## 主要参数

| 参数 | 当前值 | 位置 |
|---|---:|---|
| 控制周期 | 20 ms | `empty.syscfg` 的 `TIMER_0` |
| 基础循迹速度 | 80 | `Basic_Speed` |
| PWM 输出限幅 | +/-100 | `DRIVE_PWM_LIMIT` |
| 普通速度环 | `Kp=0.165, Ki=0.045, Kd=0` | `speed_left` / `speed_right` |
| 转向速度环 | `Kp=0.28, Ki=0.045, Kd=0` | `turn_speed_left` / `turn_speed_right` |
| 循迹位置环 | `Kp=3.0, Kd=1.5` | `Xunji` |
| 原地转向环 | `Kp=2.0, Kd=1.0` | `Turn` |
| 航向直行环 | `Kp=1.5, Kd=3.0` | `Straight` |

## 引脚分配

### 电机驱动

| 功能 | 信号 | MCU 引脚 | SysConfig 名称 | 代码接口 |
|---|---|---|---|---|
| 右电机 PWM | PWM_R | PB2 | `PWM_0` C0 / `TIMA1_CCP0` | `Set_PWM_R()` |
| 左电机 PWM | PWM_L | PB3 | `PWM_0` C1 / `TIMA1_CCP1` | `Set_PWM_L()` |
| 左电机方向 | AIN1 | PA12 | `AIN_AIN1` | `Set_PWM_L()` |
| 左电机方向 | AIN2 | PA13 | `AIN_AIN2` | `Set_PWM_L()` |
| 右电机方向 | BIN1 | PA17 | `BIN_BIN1` | `Set_PWM_R()` |
| 右电机方向 | BIN2 | PA16 | `BIN_BIN2` | `Set_PWM_R()` |

### 编码器

| 功能 | 信号 | MCU 引脚 | SysConfig 名称 | 配置 |
|---|---|---|---|---|
| 编码器 A 相 | E1A | PA25 | `ENCODERA_E1A` | 上拉输入，上升沿中断 |
| 编码器 A 相 | E1B | PA24 | `ENCODERA_E1B` | 上拉输入，上升沿中断 |
| 编码器 B 相 | E2A | PB19 | `ENCODERB_E2A` | 上拉输入，上升沿中断 |
| 编码器 B 相 | E2B | PB20 | `ENCODERB_E2B` | 上拉输入，上升沿中断 |

当前闭环映射：

| 轮子 | 编码器计数 | RPM 变量 | PID | PWM 输出 |
|---|---|---|---|---|
| 左轮 | `Get_Encoder_countA` | `MA_RPM` | `speed_left` | `Set_PWM_L()` |
| 右轮 | `Get_Encoder_countB` | `MB_RPM` | `speed_right` | `Set_PWM_R()` |

联调时必须确认 `Set_PWM_L()` 转动的实际轮子对应 `MA_RPM` 变化，`Set_PWM_R()` 转动的实际轮子对应 `MB_RPM` 变化。

### 8 路数字循迹

| 通道 | 信号 | MCU 引脚 | SysConfig 名称 | 说明 |
|---|---|---|---|---|
| CH1 | XUNJI0 | PB16 | `XUNJI_XUNJI0` | 低电平判定为黑线 |
| CH2 | XUNJI1 | PB0 | `XUNJI_XUNJI1` | 低电平判定为黑线 |
| CH3 | XUNJI2 | PB6 | `XUNJI_XUNJI2` | 低电平判定为黑线 |
| CH4 | XUNJI3 | PB7 | `XUNJI_XUNJI3` | 低电平判定为黑线 |
| CH5 | XUNJI4 | PB8 | `XUNJI_XUNJI4` | 低电平判定为黑线 |
| CH6 | XUNJI5 | PB15 | `XUNJI_XUNJI5` | 低电平判定为黑线 |
| CH7 | XUNJI6 | PB17 | `XUNJI_XUNJI6` | 低电平判定为黑线 |
| CH8 | XUNJI7 | PB12 | `XUNJI_XUNJI7` | 低电平判定为黑线 |

### 传感器与显示

| 功能 | 信号 | MCU 引脚 | SysConfig 名称 | 说明 |
|---|---|---|---|---|
| ADC 输入 | ADC_IN | PA15 | `ADC12_0` C0 / `ADC1` CH0 | 10 bit ADC |
| OLED SCL | SCL | PA8 | `GPIO_OLED_PIN_OLED_SCL` | 软件 I2C |
| OLED SDA | SDA | PA26 | `GPIO_OLED_PIN_OLED_SDA` | 软件 I2C |
| JY61P SDA | SDA | PA28 | `I2C_GYRO` SDA / `I2C0_SDA` | 硬件 I2C，Fast，内部上拉 |
| JY61P SCL | SCL | PA31 | `I2C_GYRO` SCL / `I2C0_SCL` | 硬件 I2C，Fast，内部上拉 |

### 通信与人机交互

| 功能 | 信号 | MCU 引脚 | SysConfig 名称 | 说明 |
|---|---|---|---|---|
| UART0 TX | TX | PA10 | `UART_0_TX` | 115200 8N1，`printf` 默认同时输出到 UART0/UART1 |
| UART0 RX | RX | PA11 | `UART_0_RX` | RX 中断调用 `vofa_set_data()` |
| UART1 TX | TX | PB4 | `UART_1_TX` | 115200 8N1 |
| UART1 RX | RX | PA9 | `UART_1_RX` | RX 中断调用 `vofa_set_data()` |
| 按键 A | KEY | PB21 | `KEY_key` | 上拉输入，按下为低电平；未发车时切换题目模式 |
| 按键 B | KEY_2 | PA18 | `KEY_key_2` | 上电自适应空闲电平；启动/急停 |
| 指示灯 | LED | PA0 | `LED_led` | 低电平点亮 |

### 定时与调试

| 功能 | 外设/引脚 | SysConfig 名称 | 说明 |
|---|---|---|---|
| 控制定时器 | TIMG0 | `TIMER_0` | 20 ms 周期中断 |
| PWM 定时器 | TIMA1 | `PWM_0` | 计数 100，C0/C1 输出 |
| SWDIO | PA19 | DEBUGSS | 下载/调试接口，建议保留 |
| SWCLK | PA20 | DEBUGSS | 下载/调试接口，建议保留 |

## 快速调试清单

1. 修改 `empty.syscfg` 后，先确认能重新生成 `Debug/ti_msp_dl_config.*`。
2. 上电后观察 OLED：应显示 `TASK SELECT`、当前 `MODE` 和 `STATE:READY`。
3. 未发车时按 A 键，确认模式在 1-6 循环。
4. 按 B 键启动前，确认左右电机 PWM 为 0；运行中再按 B 键应立即急停。
5. 单独读取 `digital(1)` 到 `digital(8)`，确认黑线为 1、白底为 0。
6. 单独测试左右电机方向和编码器计数，确认电机极性、编码器方向、闭环映射一致。
7. 单独测试 JY61P yaw，确认顺/逆时针角度变化方向与 `Turn_In_Place()` 的控制方向一致。
8. 调 VOFA+ 参数时先低速、限幅、小步调整，先保证速度环稳定，再调循迹和转向。

## 维护注意

1. 引脚表以 `empty.syscfg` 为准。改 SysConfig 后必须同步更新本文档。
2. 不建议手动修改 `Debug/ti_msp_dl_config.*`，这些文件由 SysConfig 生成。
3. 修改左右电机归属时，需要同时检查 `Hardware/motor.c`、`Hardware/control.c`、编码器接线和实车轮子方向。
4. 修改控制周期时，需要同步检查 `TIMER_0` 周期、`Calculate_Motor_RPM()` 采样时间和 PID 参数。
5. `mode.c` 中的状态机使用静态变量保存进度；如果需要在重新选择模式或重新发车后从头开始，应补充对应状态清零逻辑。
6. 当前 README 只描述 `README.md`；根目录的 `README.html` 若用于展示，需要在 Markdown 更新后重新生成。
