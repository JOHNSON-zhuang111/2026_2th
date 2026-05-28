#include "mode.h"
#include <math.h>

extern u8 task_mode;

#define MODE1_STRAIGHT_SPEED 30
#define MODE6_STRAIGHT_BEFORE_TURN_TICKS 30U

u8 allwhite(void)
{
    // 注意：digital() 返回 1 表示检测到黑线，所以这里实际表示“8 路全黑”。
    if (digital(1) + digital(2) + digital(3) + digital(4) +
        digital(5) + digital(6) + digital(7) + digital(8) == 8)
    {
        return 1;
    }
    else return 0;
}

static u8 middle_black(void)
{
    // 中间探头压到黑线，用来判断是否到达中间标志线。
    return ((digital(3) && digital(4)) ||
            (digital(4) && digital(5)) ||
            (digital(5) && digital(6)));
}

static u8 no_black(void)
{
    // 8 路探头都没有检测到黑线，常用于判断已经离开圆弧/标志线。
    return (digital(1) + digital(2) + digital(3) + digital(4) +
            digital(5) + digital(6) + digital(7) + digital(8) == 0);
}

static u8 any_black(void)
{
    // 任意一路检测到黑线，用于判断进入某个目标点或赛道区域。
    return (digital(1) || digital(2) || digital(3) || digital(4) ||
            digital(5) || digital(6) || digital(7) || digital(8));
}

static void point_prompt_once(void)
{
    // 到达关键点时翻转 LED，作为一次提示。
    LED_Toggle();
}

static float normalize_angle(float angle)
{
    // 将角度限制到 -180~180，避免陀螺仪跨越边界时误差突变。
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float angle_diff(float target_angle, float current_angle)
{
    // 计算最短方向的角度误差。
    return normalize_angle(target_angle - current_angle);
}

// 模式 1：A 点直行到 b 点，检测到黑线后停车。
void mode_1(void)
{
    static int state = 0;
    static uint32_t delay_cnt = 0;   // 连续检测计数，防止瞬时误判。
    
    float target_angle_b = 0.0f;     // A -> b 的直行航向角。

    switch (state) {
        case 0:
            // 沿 target_angle_b 直行；连续检测到黑线后认为到达 b 点。
            Keep_Angle_Straight(target_angle_b, MODE1_STRAIGHT_SPEED);
            if (any_black()) {
                delay_cnt++;
                if (delay_cnt > 3) {
                    state = 1;
                    delay_cnt = 0;
                }
            } else {
                delay_cnt = 0;
            }
            break;
            
        case 1:
            // 到达 b 点后清空控制器状态并停车。
            control_reset_runtime_state();
            Set_PWM_L(0);
            Set_PWM_R(0);
            break;
    }
}

// 模式 2：A->B 直行，B->C 右半圆循迹，C->D 直行，D->A 左半圆循迹。
void mode_2(void)
{
    static int state = 0;
    static uint32_t delay_cnt = 0;   // 进入新路段后的保护延时。
    static uint32_t stable_cnt = 0;  // 连续满足切换条件的计数。
    
    float angle_ab = 0.0f;
    float angle_cd = 180.0f;

    switch (state) {
        case 0:
            // A -> B：按 0 度航向直行，检测到黑线后进入圆弧循迹。
            Keep_Angle_Straight(angle_ab, 80);
            if (any_black()) {
                stable_cnt++;
                if (stable_cnt > 3) {
                    point_prompt_once();
                    state = 1;
                    delay_cnt = 0;
                    stable_cnt = 0;
                }
            } else {
                stable_cnt = 0;
            }
            break;
            
        case 1:
            // B -> C：右半圆循迹；先延时一段时间，避免刚入弧时误判离线。
            Xunji_Speed();
            if (delay_cnt < 50) {
                delay_cnt++;
                stable_cnt = 0;
            } else if (no_black()) {
                stable_cnt++;
                if (stable_cnt > 3) {
                    point_prompt_once();
                    state = 2;
                    delay_cnt = 0;
                    stable_cnt = 0;
                }
            } else {
                stable_cnt = 0;
            }
            break;
            
        case 2:
            // C -> D：按 180 度航向直行，检测到黑线后进入下一段圆弧。
            Keep_Angle_Straight(angle_cd, 80);
            if (any_black()) {
                stable_cnt++;
                if (stable_cnt > 3) {
                    point_prompt_once();
                    state = 3;
                    delay_cnt = 0;
                    stable_cnt = 0;
                }
            } else {
                stable_cnt = 0;
            }
            break;
            
        case 3:
            // D -> A：左半圆循迹；离开黑线后认为回到 A 点附近。
            Xunji_Speed();
            if (delay_cnt < 50) {
                delay_cnt++;
                stable_cnt = 0;
            } else if (no_black()) {
                stable_cnt++;
                if (stable_cnt > 3) {
                    point_prompt_once();
                    state = 4;
                    stable_cnt = 0;
                }
            } else {
                stable_cnt = 0;
            }
            break;
            
        case 4:
            // 一圈完成，清空控制器状态并停车。
            control_reset_runtime_state();
            Set_PWM_L(0);
            Set_PWM_R(0);
            break;
    }
}

// 模式 3：斜线/圆弧组合路线：A->C，C->B，原地转向 D，B->D，D->A。
void mode_3(void)
{
    static int state = 0;
    static uint32_t delay_cnt = 0;   // 保护延时，避免刚进入路段就触发切换。
    static uint32_t stable_cnt = 0;  // 条件稳定计数，用来滤掉抖动。

    float target_angle_C = 0.0f;
    float target_angle_D = -141.3f;

    switch (state) {
        case 0:
            // A -> C：按 target_angle_C 直行，延时后检测黑线作为到达 C 的依据。
            Keep_Angle_Straight(target_angle_C, 100);
            delay_cnt++;
            if (delay_cnt > 100 && any_black()) {
                stable_cnt++;
                if (stable_cnt > 2) {
                    point_prompt_once();
                    state = 1;
                    delay_cnt = 0;
                    stable_cnt = 0;
                }
            } else if (delay_cnt > 100) {
                stable_cnt = 0;
            }
            break;

        case 1:
            // C -> B：圆弧循迹，延时后检测无黑线作为离开圆弧的依据。
            Xunji_Speed();
            delay_cnt++;
            if (delay_cnt > 100 && no_black()) {
                stable_cnt++;
                if (stable_cnt > 2) { 
                    point_prompt_once();
                    state = 2;
                    delay_cnt = 0;
                    stable_cnt = 0;
                }
            } else if (delay_cnt > 50) {
                stable_cnt = 0;
            }
            break;

        case 2:
            // 原地转向 D：陀螺仪角度误差连续稳定在阈值内后进入直行。
            Turn_In_Place(target_angle_D);
            {
                Gyro_Struct *JY61P_Data = get_angle();
                float diff = angle_diff(target_angle_D, JY61P_Data->z);

                if (diff > -3.0f && diff < 3.0f) {
                    stable_cnt++;
                    if (stable_cnt > 2) {
                        state = 3;
                        delay_cnt = 0;
                        stable_cnt = 0;
                    }
                } else {
                    stable_cnt = 0;
                }
            }
            break;

        case 3:
            // B -> D：按 target_angle_D 直行，延时后检测黑线作为到达 D 的依据。
            Keep_Angle_Straight(target_angle_D, 80);
            delay_cnt++;
            if (delay_cnt > 100 && any_black()) {
                stable_cnt++;
                if (stable_cnt > 2) {
                    point_prompt_once();
                    state = 4;
                    delay_cnt = 0;
                    stable_cnt = 0;
                }
            } else if (delay_cnt > 50) {
                stable_cnt = 0;
            }
            break;

        case 4:
            // D -> A：圆弧循迹，离开黑线后认为回到 A 点附近。
            Xunji_Speed();
            delay_cnt++;
            if (delay_cnt > 100 && no_black()) {
                stable_cnt++;
                if (stable_cnt > 2) {
                    point_prompt_once();
                    state = 5;
                    stable_cnt = 0;
                }
            } else if (delay_cnt > 100) {
                stable_cnt = 0;
            }
            break;

        case 5:
            // 路线完成，清空控制器状态并停车。
            control_reset_runtime_state();
            Set_PWM_L(0);
            Set_PWM_R(0);
            break;

        default:
            // 异常状态保护：回到起始状态。
            state = 0;
            delay_cnt = 0;
            stable_cnt = 0;
            break;
    }
}

// 模式 4：执行模式 3 的路线并累计 3 圈，完成后停车。
void mode_4(void)
{
    static int state = 0;
    static uint32_t delay_cnt = 0;   // 路段保护延时。
    static uint32_t stable_cnt = 0;  // 稳定检测计数。
    static uint8_t lap_count = 0;    // 已完成圈数。
    
    float target_angle_C = 0.0f;
    float target_angle_D = -141.3f;
    
    switch (state) {
        case 0:
            // 1. A -> C：直行到 C，检测到黑线后切换到圆弧。
            Keep_Angle_Straight(target_angle_C, 80);
            delay_cnt++;
            if (delay_cnt > 50 && any_black()) {
                stable_cnt++;
                if (stable_cnt > 2) {
                    point_prompt_once();
                    state = 1;
                    delay_cnt = 0;
                    stable_cnt = 0;
                }
            } else if (delay_cnt > 50) {
                stable_cnt = 0;
            }
            break;
            
        case 1:
            // 2. C -> B：圆弧循迹，离开黑线后准备转向。
            Xunji_Speed();
            delay_cnt++;
            if (delay_cnt > 50 && no_black()) {
                stable_cnt++;
                if (stable_cnt > 2) {
                    point_prompt_once();
                    state = 2;
                    delay_cnt = 0;
                    stable_cnt = 0;
                }
            } else if (delay_cnt > 50) {
                stable_cnt = 0;
            }
            break;
            
        case 2:
            // 3. 原地转向 D，角度稳定后进入 B -> D 直行段。
            Turn_In_Place(target_angle_D);
            {
                Gyro_Struct *JY61P_Data = get_angle();
                float diff = angle_diff(target_angle_D, JY61P_Data->z);
                
                if (diff > -3.0f && diff < 3.0f) {
                    stable_cnt++;
                    if (stable_cnt > 2) {
                        state = 3;
                        delay_cnt = 0;
                        stable_cnt = 0;
                    }
                } else {
                    stable_cnt = 0;
                }
            }
            break;
            
        case 3:
            // 4. B -> D：沿 target_angle_D 直行到 D。
            Keep_Angle_Straight(target_angle_D, 80);
            delay_cnt++;
            if (delay_cnt > 50 && any_black()) {
                stable_cnt++;
                if (stable_cnt > 2) {
                    point_prompt_once();
                    state = 4;
                    delay_cnt = 0;
                    stable_cnt = 0;
                }
            } else if (delay_cnt > 50) {
                stable_cnt = 0;
            }
            break;
            
        case 4:
            // 5. D -> A：圆弧循迹回到 A，完成一圈后累计 lap_count。
            Xunji_Speed();
            delay_cnt++;
            if (delay_cnt > 50 && no_black()) {
                stable_cnt++;
                if (stable_cnt > 2) {
                    point_prompt_once();
                    lap_count++;
                    delay_cnt = 0;
                    stable_cnt = 0;
                    if (lap_count >= 3) {
                        state = 5;
                    } else {
                        state = 0;
                    }
                }
            } else if (delay_cnt > 50) {
                stable_cnt = 0;
            }
            break;
            
        case 5:
            // 三圈完成，清空控制器状态并停车。
            control_reset_runtime_state();
            Set_PWM_L(0);
            Set_PWM_R(0);
            break;

        default:
            // 异常状态保护：计数全部清零并重新开始。
            state = 0;
            delay_cnt = 0;
            stable_cnt = 0;
            lap_count = 0;
            break;
    }
}

// 模式 5：自动走正方形。每条边直行一段时间，然后原地转向 90 度。
void mode_5(void)
{
    static int state = 0;
    static uint32_t delay_cnt = 0;   // 直行计时或转向稳定计数。
    static int current_edge = 0;     // 当前正在走第几条边，0~3 循环。
    
    float target_angle = current_edge * 90.0f;
    float next_angle = (current_edge + 1) * 90.0f;

    // 将目标角度限制在 -180~180，方便和陀螺仪角度比较。
    while (target_angle > 180.0f) target_angle -= 360.0f;
    while (target_angle < -180.0f) target_angle += 360.0f;
    while (next_angle > 180.0f) next_angle -= 360.0f;
    while (next_angle < -180.0f) next_angle += 360.0f;

    // 获取当前陀螺仪 yaw 角。
    Gyro_Struct *JY61P_Data = get_angle();
    float current_yaw = JY61P_Data->z;

    // 将目标角度拉到离当前 yaw 最近的位置，避免跨越 180 度边界时误差跳变。
    if (target_angle - current_yaw >  180.0f) target_angle -= 360.0f;
    if (target_angle - current_yaw < -180.0f) target_angle += 360.0f;
    if (next_angle - current_yaw >  180.0f) next_angle -= 360.0f;
    if (next_angle - current_yaw < -180.0f) next_angle += 360.0f;

    switch (state) {
        case 0:
            // 1. 直行阶段：沿当前边的目标角度前进固定时间。
            Keep_Angle_Straight(target_angle, 80);
            delay_cnt++;
            if (delay_cnt > 75) { 
                // 时间到，切换到原地转向阶段。
                state = 1;
                delay_cnt = 0;
            }
            break;
            
        case 1:
            // 2. 转向阶段：转到下一条边的角度。
            Turn_In_Place(next_angle);
            {
                float diff = next_angle - current_yaw;
                
                // 将角度误差归一化到 -180~180，防止跨边界时误差突变。
                while (diff > 180.0f) diff -= 360.0f;
                while (diff < -180.0f) diff += 360.0f;
                
                // 角度误差连续落入阈值内，认为本次 90 度转向完成。
                if (diff > -2.0f && diff < 2.0f) {
                    delay_cnt++;
                    if (delay_cnt > 3) {
                        current_edge++; 
                        if (current_edge >= 4) {
                            current_edge = 0;
                        }
                        // 切换回直行阶段，开始下一条边。
                        state = 0;
                        delay_cnt = 0;
                    }
                } else {
                    delay_cnt = 0;
                }
            }
            break;
    }
}

// 模式 6：普通循迹，检测到左侧黑线后先短直行，再左转 90 度，最后回到循迹。
void mode_6(void)
{
    static uint8_t state = 0;
    static uint8_t left_black_cnt = 0;         // 左侧探头连续检测到黑线的计数。
    static uint8_t stable_cnt = 0;             // 转向完成的稳定计数。
    static uint16_t straight_cnt = 0;          // 检测到左侧黑线后的短直行计时。
    static uint16_t cooldown_cnt = 0;          // 转弯后冷却计时，避免立刻重复触发。
    static float straight_target_angle = 0.0f; // 触发转弯前锁定的直行航向。
    static float turn_target_angle = 0.0f;     // 需要转到的目标航向。

    Gyro_Struct *JY61P_Data = get_angle();
    float current_yaw = JY61P_Data->z;

    switch (state) {
        case 0:
            // 正常循迹；左侧探头连续检测到黑线后，进入短直行阶段。
            Xunji_Speed();

            if (digital(1)) {
                left_black_cnt++;
                if (left_black_cnt > 2) {
                    straight_target_angle = current_yaw;
                    state = 1;
                    left_black_cnt = 0;
                    stable_cnt = 0;
                    straight_cnt = 0;
                }
            } else {
                left_black_cnt = 0;
            }
            break;

        case 1:
            // 保持触发瞬间的航向短直行一段距离，避免直接原地转弯压在线上。
            Keep_Angle_Straight(straight_target_angle, Basic_Speed);
            straight_cnt++;
            if (straight_cnt >= MODE6_STRAIGHT_BEFORE_TURN_TICKS) {
                turn_target_angle = normalize_angle(straight_target_angle + 90.0f);
                state = 2;
                straight_cnt = 0;
                stable_cnt = 0;
            }
            break;

        case 2:
            // 原地左转 90 度；角度连续稳定后进入冷却循迹阶段。
            Turn_In_Place(turn_target_angle);
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if (diff > -3.0f && diff < 3.0f) {
                    stable_cnt++;
                    if (stable_cnt > 3) {
                        state = 3;
                        stable_cnt = 0;
                        cooldown_cnt = 0;
                    }
                } else {
                    stable_cnt = 0;
                }
            }
            break;

        case 3:
            // 转弯后继续循迹，等左侧探头离开黑线后再允许下一次触发。
            Xunji_Speed();
            cooldown_cnt++;
            if (cooldown_cnt > 30 && !digital(1)) {
                state = 0;
                cooldown_cnt = 0;
            }
            break;

        default:
            // 异常状态保护：所有计数清零并回到普通循迹。
            state = 0;
            left_black_cnt = 0;
            stable_cnt = 0;
            straight_cnt = 0;
            cooldown_cnt = 0;
            break;
    }
}
