# 2025_E MSPM0G3507 智能车工程说明

本工程基于 TI MSPM0G3507 / MSPM0G3505、MSPM0 SDK 2.10.00.04 和 CCS Theia 开发，面向 2025_E 题目小车控制。代码包含双电机驱动、编码器测速、8 路数字循迹、JY61P 陀螺仪、OLED 显示、按键选题、UART/VOFA 调参和多模式任务控制。

当前 SysConfig 目标为 `MSPM0G350X / MSPM0G3505`，封装为 `LQFP-64(PM)`。外设与引脚以 `empty.syscfg` 为准，生成文件位于 `Debug/ti_msp_dl_config.*`。

## 工程结构

| 路径 | 说明 |
|---|---|
| `empty.c` | 主程序入口，完成外设初始化、按键扫描、OLED 显示、运行状态切换和 20 ms 控制调度 |
| `empty.syscfg` | TI SysConfig 配置文件，定义 GPIO、PWM、UART、I2C、ADC、TIMER 等外设 |
| `Hardware/board.*` | 公共头文件、基础类型、板级函数声明 |
| `Hardware/control.*` | 速度环、循迹 PD、转向控制、直行保持和运行状态复位 |
| `Hardware/mode.*` | 题目模式 1-6 的状态机 |
| `Hardware/motor.*` | 左右电机方向控制和 PWM 输出 |
| `Hardware/encoder.*` | AB 编码器中断计数与 RPM 计算 |
| `Hardware/xunji.*` | 8 路数字循迹读取，低电平判定为黑线 |
| `Hardware/error.*` | 循迹误差计算 |
| `Hardware/bsp_gyro.*` | JY61P 陀螺仪 I2C 读写与角度获取 |
| `Hardware/oled_software_i2c.*` | 软件 I2C OLED 显示 |
| `Hardware/key.*` | A/B 按键消抖、选题和启动/急停 |
| `Hardware/uart_vofa.*` | UART 接收解析和 VOFA+ 调参/数据输出 |
| `vofa.md` | VOFA+ 调试说明 |
| `BACK.md` | 备份/历史说明 |

## 运行流程

1. `main()` 调用 `SYSCFG_DL_init()` 初始化 SysConfig 外设。
2. 初始化 JY61P、OLED、PWM、编码器中断、TIMG0 定时器中断和 UART0/UART1 接收中断。
3. 上电默认不启动业务中断，主循环持续扫描按键并刷新 OLED 选题界面。
4. A 键在未发车时切换 `task_mode`，范围为 1-6。
5. B 键用于启动/急停。启动时清空控制器运行状态，急停时立即清零 PWM。
6. `TIMER_0_INST_IRQHandler()` 每 20 ms 置位 `control_tick_pending`。
7. 主循环检测到控制节拍后，在已启动状态下调用 `control()`。
8. `control()` 根据 `task_mode` 进入 `mode_1()` 到 `mode_6()`，再计算编码器速度、更新速度 PID。

注意：当前 `Hardware/control.c` 中 `case 1` 临时调用的是 `Turn_In_Place(90)`，原来的 `mode_1()` 被注释。若要恢复题目 1 的完整状态机，需要把该分支改回 `mode_1()`。

## 控制逻辑

| 功能 | 入口 | 说明 |
|---|---|---|
| 速度闭环 | `PID_Update()` | 增量式 PID，目标速度来自 `Left_Speed` / `Right_Speed` |
| 循迹控制 | `Xunji_Speed()` | 根据 `Error_Calculate()` 得到偏差，位置式 PD 输出差速 |
| 原地转向 | `Turn_In_Place()` | 读取 JY61P yaw，使用转向 PD 和左右轮反向速度实现定角转向 |
| 直行保持 | `Keep_Angle_Straight()` | 锁定目标航向，根据 yaw 偏差修正左右轮速度 |
| 状态清零 | `control_reset_runtime_state()` | 清空速度、编码器计数、滤波缓存和 PID 运行量 |

编码器速度每 20 ms 计算一次，并经过 5 点移动平均滤波。当前参数按 13 线编码器、2 倍频、30:1 减速比计算 RPM。

## 题目模式

| 模式 | 函数 | 当前意图 |
|---|---|---|
| 1 | `mode_1()` | A 点直行到 b 点，检测黑线后停车 |
| 2 | `mode_2()` | A->B 直行，B->C 右半圆循迹，C->D 直行，D->A 左半圆循迹 |
| 3 | `mode_3()` | 斜线与圆弧组合路线，包含原地转向到 D 方向 |
| 4 | `mode_4()` | 执行模式 3 路线并累计 3 圈 |
| 5 | `mode_5()` | 自动走正方形，直行一段时间后原地转向 90 度 |
| 6 | `mode_6()` | 普通循迹，左侧检测到黑线后短直行、左转 90 度，再回到循迹 |

模式状态机大量依赖 `any_black()`、`no_black()`、`digital(1)` 等循迹判断，以及 JY61P yaw 角。调试时建议先单独验证循迹电平、陀螺仪方向和电机极性，再联调完整模式。

## 引脚分配

### 电机驱动

| 功能 | 信号 | MCU 引脚 | SysConfig 名称 | 代码使用 |
|---|---|---|---|---|
| 右电机 PWM | PWM_R | PB2 | `PWM_0` C0 / `TIMA1_CCP0` | `Set_PWM_R()` |
| 左电机 PWM | PWM_L | PB3 | `PWM_0` C1 / `TIMA1_CCP1` | `Set_PWM_L()` |
| 左电机方向 | AIN1 | PA12 | `AIN_AIN1` | `Set_PWM_L()` |
| 左电机方向 | AIN2 | PA13 | `AIN_AIN2` | `Set_PWM_L()` |
| 右电机方向 | BIN1 | PA17 | `BIN_BIN1` | `Set_PWM_R()` |
| 右电机方向 | BIN2 | PA16 | `BIN_BIN2` | `Set_PWM_R()` |

### 编码器

| 功能 | 信号 | MCU 引脚 | SysConfig 名称 | 配置 | 代码使用 |
|---|---|---|---|---|---|
| 编码器 A 相 | E1A | PA25 | `ENCODERA_E1A` | 上拉输入，上升沿中断 | `Get_Encoder_countA` |
| 编码器 A 相 | E1B | PA24 | `ENCODERA_E1B` | 上拉输入，上升沿中断 | `Get_Encoder_countA` |
| 编码器 B 相 | E2A | PB19 | `ENCODERB_E2A` | 上拉输入，上升沿中断 | `Get_Encoder_countB` |
| 编码器 B 相 | E2B | PB20 | `ENCODERB_E2B` | 上拉输入，上升沿中断 | `Get_Encoder_countB` |

当前闭环映射：

| 轮子 | 编码器计数 | 转速变量 | PID | PWM 输出 |
|---|---|---|---|---|
| 左轮 | `Get_Encoder_countA` | `MA_RPM` | `speed_left` | `Set_PWM_L()` |
| 右轮 | `Get_Encoder_countB` | `MB_RPM` | `speed_right` | `Set_PWM_R()` |

联调时应确认 `Set_PWM_L()` 转动的电机对应 `MA_RPM` 变化，`Set_PWM_R()` 转动的电机对应 `MB_RPM` 变化。

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
| JY61P SDA | SDA | PA28 | `I2C_GYRO` SDA / `I2C0_SDA` | 硬件 I2C，400 kHz，上拉 |
| JY61P SCL | SCL | PA31 | `I2C_GYRO` SCL / `I2C0_SCL` | 硬件 I2C，400 kHz，上拉 |

`Hardware/ganv_hw.*` 中保留了灰度模拟采样相关代码，但当前 SysConfig 只配置了 PA15 单路 ADC，未配置 3 位地址选择 GPIO。若要恢复 8 路模拟灰度复用采样，需要先在 SysConfig 中补齐地址线并同步代码。

### 通信与人机交互

| 功能 | 信号 | MCU 引脚 | SysConfig 名称 | 配置/说明 |
|---|---|---|---|---|
| UART0 TX | TX | PA10 | `UART_0_TX` | 115200 8N1，调试/VOFA |
| UART0 RX | RX | PA11 | `UART_0_RX` | 接收中断调用 `vofa_set_data()` |
| UART1 TX | TX | PB4 | `UART_1_TX` | 115200 8N1 |
| UART1 RX | RX | PA9 | `UART_1_RX` | 接收中断调用 `vofa_set_data()` |
| 按键 A | KEY | PB21 | `KEY_key` | 上拉输入，按下为低电平；未发车时切换题目模式 |
| 按键 B | KEY_2 | PA18 | `KEY_key_2` | 下拉输入，代码按上电空闲电平自适应；启动/急停 |
| 指示灯 | LED | PA0 | `LED_led` | 低电平点亮 |

### 定时与调试

| 功能 | 外设/引脚 | SysConfig 名称 | 配置/说明 |
|---|---|---|---|
| 控制定时器 | TIMG0 | `TIMER_0` | 20 ms 周期中断 |
| PWM 定时器 | TIMA1 | `PWM_0` | 计数 100，C0/C1 输出 |
| SWDIO | PA19 | DEBUGSS | 下载/调试，建议保留 |
| SWCLK | PA20 | DEBUGSS | 下载/调试，建议保留 |

## 快速调试清单

1. 编译并下载前，确认 `empty.syscfg` 能正常生成 `ti_msp_dl_config.*`。
2. 上电后先观察 OLED 选题界面，确认 A 键能在 1-6 模式循环。
3. 按 B 键启动前，小车应保持电机停止；运行中再按 B 键应急停并清零 PWM。
4. 单独测试 `digital(1)` 到 `digital(8)`，确认黑线为 1、白底为 0。
5. 单独测试左右电机方向和编码器计数，确认电机极性、编码器方向与闭环映射一致。
6. 测试 JY61P yaw，确认顺/逆时针转动时角度变化方向与 `Turn_In_Place()` 的控制方向一致。
7. 使用 VOFA+ 调 PID 时，先低速、限幅、小步调整，优先保证速度闭环稳定，再调循迹和转向。

## 维护注意事项

1. 引脚表以 `empty.syscfg` 为准。修改 SysConfig 后，应重新生成配置并同步更新本文档。
2. 不建议手改 `Debug/ti_msp_dl_config.*`，这些文件由 SysConfig 生成。
3. 修改左右电机归属时，需要同时检查 `Hardware/motor.c`、`Hardware/control.c`、编码器接线和实际轮子方向。
4. 修改控制周期时，需要同步检查 `TIMER_0` 周期、`Calculate_Motor_RPM()` 的采样时间和 PID 参数。
5. 模式状态机使用静态变量保存进度，若需要在切换模式后从头开始，需补充对应状态清零逻辑。
