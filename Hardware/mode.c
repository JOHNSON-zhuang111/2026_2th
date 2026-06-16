#include "mode.h"
#include <math.h>

extern volatile u8 task_mode;
extern volatile u8 car_started;
extern volatile u8 set_quanshu;

#define MODE1_STRAIGHT_SPEED 30
#define MODE6_STRAIGHT_BEFORE_TURN_TICKS 30U
#define MODE1_TURNS_PER_LAP 4U
#define MODE4_TURNS_PER_LAP 4U
#define MODE4_OBSTACLE_DISTANCE_CM 18.0f
#define MODE4_OBSTACLE_HIT_COUNT 2U
#define MODE4_LINE_SEARCH_MIN_TICKS 20U

static uint8_t mode4_target_laps(void)
{
    if (set_quanshu < 1U) {
        return 1U;
    }
    if (set_quanshu > 5U) {
        return 5U;
    }
    return set_quanshu;
}

static uint8_t mode4_obstacle_detected(uint16_t *sample_ticks, uint8_t *hit_count)
{
    (void) sample_ticks;
    float distance_cm = SR04_GetLengthNonBlocking();
    if ((distance_cm > 0.0f) && (distance_cm <= MODE4_OBSTACLE_DISTANCE_CM)) {
        if (*hit_count < MODE4_OBSTACLE_HIT_COUNT) {
            (*hit_count)++;
        }
    } else {
        *hit_count = 0U;
    }

    return (*hit_count >= MODE4_OBSTACLE_HIT_COUNT) ? 1U : 0U;
}

static uint8_t mode4_turn_done(float target_angle, float current_yaw, uint8_t *stable_cnt, uint8_t stable_need)
{
    Turn_In_Place(target_angle);
    float diff = angle_diff(target_angle, current_yaw);
    if ((diff > -1.0f) && (diff < 1.0f)) {
        (*stable_cnt)++;
        if (*stable_cnt > stable_need) {
            *stable_cnt = 0U;
            return 1U;
        }
    } else {
        *stable_cnt = 0U;
    }

    return 0U;
}

static void mode4_reset_state(uint8_t *left_black_cnt,
                              uint8_t *stable_cnt,
                              uint8_t *turn_count,
                              uint8_t *lap_count,
                              uint8_t *obstacle_hit_cnt,
                              uint16_t *straight_cnt,
                              uint16_t *cooldown_cnt,
                              uint16_t *sr04_sample_cnt)
{
    *left_black_cnt = 0U;
    *stable_cnt = 0U;
    *turn_count = 0U;
    *lap_count = 0U;
    *obstacle_hit_cnt = 0U;
    *straight_cnt = 0U;
    *cooldown_cnt = 0U;
    *sr04_sample_cnt = 0U;
}

static u8 allwhite(void)//全白
{
    // 注意：digital() 返回 1 表示检测到黑线，所以这里实际表示“8 路全黑”。
    if (digital(1) + digital(2) + digital(3) + digital(4) +
        digital(5) + digital(6) + digital(7) + digital(8) == 8)
    {
        return 1;
    }
    else return 0;
}

static u8 middle_black(void)//中间黑
{
    // 中间探头压到黑线，用来判断是否到达中间标志线。
    return ((digital(3) && digital(4)) ||
            (digital(4) && digital(5)) ||
            (digital(5) && digital(6)));
}

static u8 any_black(void)//任意黑
{
    // 任意一路检测到黑线，用于判断进入某个目标点或赛道区域。
    return (digital(1) || digital(2) || digital(3) || digital(4) ||
            digital(5) || digital(6) || digital(7) || digital(8));
}

float normalize_angle(float angle)//角度归一化
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

float angle_diff(float target_angle, float current_angle)//计算最短方向的角度误差
{
    
    return normalize_angle(target_angle - current_angle);
}




// 模式 1：普通循迹，检测到左侧黑线后先短直行，再左转 90 度，最后回到循迹。
void mode_1(void)
{
    static uint8_t state = 0;
    static uint8_t left_black_cnt = 0;         // 左侧探头连续检测到黑线的计数。
    static uint8_t stable_cnt = 0;             // 转向完成的稳定计数。
    static uint16_t straight_cnt = 0;          // 检测到左侧黑线后的短直行计时。
    static uint16_t cooldown_cnt = 0;          // 转弯后冷却计时，避免立刻重复触发。
    static uint8_t turn_count = 0;             // 当前圈已完成的转弯次数。
    static uint8_t lap_count = 0;              // 已完成圈数。
    static float straight_target_angle = 0.0f; // 触发转弯前锁定的直行航向。
    static float turn_target_angle = 0.0f;     // 需要转到的目标航向。

    Gyro_Struct *JY61P_Data = get_angle();
    float current_yaw = JY61P_Data->z;

    if (state == 4U) {
        state = 0;
        left_black_cnt = 0;
        stable_cnt = 0;
        straight_cnt = 0;
        cooldown_cnt = 0;
        turn_count = 0;
        lap_count = 0;
    }

    switch (state) {
        case 0:
            // 正常循迹；左侧探头连续检测到黑线后，进入短直行阶段。
            Xunji_Speed();

            if (digital(1)) {
                left_black_cnt++;
                if (left_black_cnt > 1) {
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
            Keep_Angle_Straight(straight_target_angle, 80);
            straight_cnt++;
            if (straight_cnt >= 30) {
                turn_target_angle = normalize_angle(straight_target_angle + 70.0f);
                state = 2;
                straight_cnt = 0;
                stable_cnt = 0;
            }
            break;

        case 2:
            // 原地左转 70 度；角度连续稳定后进入冷却循迹阶段。
            Turn_In_Place(turn_target_angle);
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if (diff > -1.0f && diff < 1.0f) {
                    stable_cnt++;
                    if (stable_cnt > 0) {
                        turn_count++;
                        if (turn_count >= 4) {
                            turn_count = 0;
                            lap_count++;
                        }

                        uint8_t target_laps = (set_quanshu == 0U) ? 1U : set_quanshu;
                        if (lap_count >= target_laps) {
                            control_reset_runtime_state();
                            Set_PWM_L(0);
                            Set_PWM_R(0);
                            car_started = 0U;
                            state = 4;
                            stable_cnt = 0;
                            cooldown_cnt = 0;
                            break;
                        }

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

        case 4:
            // 目标圈数完成，保持停车；下一次重新启动时从 default/复位路径恢复。
            control_reset_runtime_state();
            Set_PWM_L(0);
            Set_PWM_R(0);
            break;

        default:
            // 异常状态保护：所有计数清零并回到普通循迹。
            state = 0;
            left_black_cnt = 0;
            stable_cnt = 0;
            straight_cnt = 0;
            cooldown_cnt = 0;
            turn_count = 0;
            lap_count = 0;
            break;
    }
}

void mode_2(void)
{
    static uint8_t state = 0;
    static uint8_t left_black_cnt = 0;         // 左侧探头连续检测到黑线的计数。
    static uint8_t right_black_cnt = 0;        // 右侧探头连续检测到黑线的计数。
    static uint8_t stable_cnt = 0;             // 转向完成的稳定计数。
    static uint16_t straight_cnt = 0;          // 检测到左侧黑线后的短直行计时。
    static uint16_t cooldown_cnt = 0;          // 转弯后冷却计时，避免立刻重复触发。
    static float straight_target_angle = 0.0f; // 触发转弯前锁定的直行航向。
    static float turn_target_angle = 0.0f;     // 需要转到的目标航向。
    static uint16_t lap_count = 0;             // 圈数计数。
    Gyro_Struct *JY61P_Data = get_angle();
    float current_yaw = JY61P_Data->z;

    switch (state) {
        case 0:
            // 正常循迹；左侧探头连续检测到黑线后，进入短直行阶段。A-B
            Xunji_Speed();
            stable_cnt++;
            if (stable_cnt>120 && (digital(1)||digital(2))) {
                left_black_cnt++;
                if (left_black_cnt > 0) {
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
            Keep_Angle_Straight(straight_target_angle, 50);
            straight_cnt++;
            if (straight_cnt >= 50) {
                turn_target_angle = normalize_angle(straight_target_angle + 136.0f);
                state = 2;
                straight_cnt = 0;
                stable_cnt = 0;
            }
            break;

        case 2:
            // 原地左转 135 度；角度连续稳定后进入冷却循迹阶段。B-D
            Turn_In_Place(turn_target_angle);
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if (diff > -1.0f && diff < 1.0f) {
                    straight_target_angle = turn_target_angle;
                    stable_cnt++;
                    if (stable_cnt > 1) {
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
            Keep_Angle_Straight(straight_target_angle, 150);//B-D
            straight_cnt++;
            if (straight_cnt >= 150 && any_black()) {
                left_black_cnt++;
                if (left_black_cnt > 2) {
                    straight_target_angle = current_yaw;
                    state = 4;
                    left_black_cnt = 0;
                    stable_cnt = 0;
                    straight_cnt = 0;
                }
            } else {
                left_black_cnt = 0;
            }
            break;
        case 4:
        // 保持触发瞬间的航向短直行一段距离，避免直接原地转弯压在线上。
            Keep_Angle_Straight(straight_target_angle, 50);//D-C
            straight_cnt++;
            if (straight_cnt >= 50) {
                turn_target_angle = normalize_angle(straight_target_angle - 135.0f);
                state = 5;
                straight_cnt = 0;
                stable_cnt = 0;
            }
            break;
        case 5:
            Turn_In_Place(turn_target_angle);//D-C
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if (diff > -1.0f && diff < 1.0f) {
                    stable_cnt++;
                    if (stable_cnt > 1) {
                        straight_target_angle = current_yaw;
                        state = 6;
                        stable_cnt = 0;
                        cooldown_cnt = 0;
                    }
                } else {
                    stable_cnt = 0;
                }
            }
            break;
        case 6:
            Xunji_Speed();//D-C
            stable_cnt++;
            if (stable_cnt > 150 && (digital(8) || digital(7))) {
                right_black_cnt++;
                if (right_black_cnt > 1) {
                    straight_target_angle = current_yaw;
                    state = 7;
                    right_black_cnt = 0;
                    stable_cnt = 0;
                    straight_cnt = 0;
                }
            } else {
                right_black_cnt = 0;
            }
            break;
            
            
        case 7:
        // 保持触发瞬间的航向短直行一段距离，避免直接原地转弯压在线上。
            Keep_Angle_Straight(straight_target_angle, 50);
            straight_cnt++;
            if (straight_cnt >= 50) {
                turn_target_angle = normalize_angle(straight_target_angle - 135.0f);
                state = 8;
                straight_cnt = 0;
                stable_cnt = 0;
            }
            break;
        case 8:
            Turn_In_Place(turn_target_angle);
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if (diff > -1.0f && diff < 1.0f) {
                    stable_cnt++;
                    if (stable_cnt > 1) {
                        straight_target_angle = current_yaw;
                        state = 9;
                        stable_cnt = 0;
                        cooldown_cnt = 0;
                    }
                } else {
                    stable_cnt = 0;
                }
            }
            break;
        case 9:
            Keep_Angle_Straight(straight_target_angle, 150);//停车
            straight_cnt++;
           if (straight_cnt >= 150 && any_black()) {
            lap_count++;

            if (lap_count >= 1) {
                control_reset_runtime_state();
                state = 12;   // 1圈完成，停车
            } else {
                state = 10;    // 没到1圈，重新走下一圈
            }

            straight_cnt = 0;
            stable_cnt = 0;
            }
            break;

        case 10:
            Keep_Angle_Straight(straight_target_angle, 50);
            straight_cnt++;
            if (straight_cnt >= 50) {
                turn_target_angle = normalize_angle(straight_target_angle + 135.0f);
                state = 11;
                straight_cnt = 0;
                stable_cnt = 0;
            }
            break;
            
        case 11:
            Turn_In_Place(turn_target_angle);
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if (diff > -1.0f && diff < 1.0f) {
                    stable_cnt++;
                    if (stable_cnt > 1) {
                        straight_target_angle = current_yaw;
                        state = 0; // 继续下一圈
                        stable_cnt = 0;
                        cooldown_cnt = 0;
                    }
                } else {
                    stable_cnt = 0;
                }
            }
            break;
        case 12:
            control_reset_runtime_state();
            Set_PWM_L(0);
            Set_PWM_R(0);
            break;
        default:
            // // 异常状态保护：所有计数清零并回到普通循迹。
            // state = 0;
            // left_black_cnt = 0;
            // stable_cnt = 0;
            // straight_cnt = 0;
            // cooldown_cnt = 0;
            break;
    }
}
void mode_3(void)
{
    static uint8_t state = 0;
    static uint8_t left_black_cnt = 0;         // 左侧探头连续检测到黑线的计数。
    static uint8_t right_black_cnt = 0;        // 右侧探头连续检测到黑线的计数。
    static uint8_t stable_cnt = 0;             // 转向完成的稳定计数。
    static uint16_t straight_cnt = 0;          // 检测到左侧黑线后的短直行计时。
    static uint16_t cooldown_cnt = 0;          // 转弯后冷却计时，避免立刻重复触发。
    static float straight_target_angle = 0.0f; // 触发转弯前锁定的直行航向。
    static float turn_target_angle = 0.0f;     // 需要转到的目标航向。
    static uint16_t lap_count = 0;             // 圈数计数。
    Gyro_Struct *JY61P_Data = get_angle();
    float current_yaw = JY61P_Data->z;

    switch (state) {
        case 0:
            // 正常循迹；左侧探头连续检测到黑线后，进入短直行阶段。A-B
            Xunji_Speed();
            stable_cnt++;
            if (stable_cnt>120 && (digital(1)||digital(2))) {
                left_black_cnt++;
                if (left_black_cnt > 0) {
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
            Keep_Angle_Straight(straight_target_angle, 80);
            straight_cnt++;
            if (straight_cnt >= 25) {
                turn_target_angle = normalize_angle(straight_target_angle + 136.0f);
                state = 2;
                straight_cnt = 0;
                stable_cnt = 0;
            }
            break;

        case 2:
            // 原地左转 135 度；角度连续稳定后进入冷却循迹阶段。B-D
            Turn_In_Place(turn_target_angle);
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if (diff > -1.0f && diff < 1.0f) {
                    straight_target_angle = turn_target_angle;
                    stable_cnt++;
                    if (stable_cnt > 1) {
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
            Keep_Angle_Straight(straight_target_angle, 160);//B-D
            straight_cnt++;
            if (straight_cnt >= 120 && any_black()) {
                left_black_cnt++;
                if (left_black_cnt > 2) {
                    straight_target_angle = current_yaw;
                    state = 4;
                    left_black_cnt = 0;
                    stable_cnt = 0;
                    straight_cnt = 0;
                }
            } else {
                left_black_cnt = 0;
            }
            break;
        case 4:
        // 保持触发瞬间的航向短直行一段距离，避免直接原地转弯压在线上。
            Keep_Angle_Straight(straight_target_angle, 80);//D-C
            straight_cnt++;
            if (straight_cnt >= 25) {
                turn_target_angle = normalize_angle(straight_target_angle - 135.0f);
                state = 5;
                straight_cnt = 0;
                stable_cnt = 0;
            }
            break;
        case 5:
            Turn_In_Place(turn_target_angle);//D-C
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if (diff > -1.0f && diff < 1.0f) {
                    stable_cnt++;
                    if (stable_cnt > 1) {
                        straight_target_angle = current_yaw;
                        state = 6;
                        stable_cnt = 0;
                        cooldown_cnt = 0;
                    }
                } else {
                    stable_cnt = 0;
                }
            }
            break;
        case 6:
            Xunji_Speed();//D-C
            stable_cnt++;
            if (stable_cnt > 120 && (digital(8) || digital(7))) {
                right_black_cnt++;
                if (right_black_cnt > 1) {
                    straight_target_angle = current_yaw;
                    state = 7;
                    right_black_cnt = 0;
                    stable_cnt = 0;
                    straight_cnt = 0;
                }
            } else {
                right_black_cnt = 0;
            }
            break;
            
            
        case 7:
        // 保持触发瞬间的航向短直行一段距离，避免直接原地转弯压在线上。
            Keep_Angle_Straight(straight_target_angle, 80);
            straight_cnt++;
            if (straight_cnt >= 25) {
                turn_target_angle = normalize_angle(straight_target_angle - 135.0f);
                state = 8;
                straight_cnt = 0;
                stable_cnt = 0;
            }
            break;
        case 8:
            Turn_In_Place(turn_target_angle);
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if (diff > -1.0f && diff < 1.0f) {
                    stable_cnt++;
                    if (stable_cnt > 1) {
                        straight_target_angle = current_yaw;
                        state = 9;
                        stable_cnt = 0;
                        cooldown_cnt = 0;
                    }
                } else {
                    stable_cnt = 0;
                }
            }
            break;
        case 9:
            Keep_Angle_Straight(straight_target_angle, 160);//停车
            straight_cnt++;
           if (straight_cnt >= 120 && any_black()) {
            lap_count++;

            if (lap_count >= 4) {
                control_reset_runtime_state();
                state = 12;   // 4圈完成，停车
            } else {
                state = 10;    // 没到4圈，重新走下一圈
            }

            straight_cnt = 0;
            stable_cnt = 0;
            }
            break;

        case 10:
            Keep_Angle_Straight(straight_target_angle, 80);
            straight_cnt++;
            if (straight_cnt >= 25) {
                turn_target_angle = normalize_angle(straight_target_angle + 135.0f);
                state = 11;
                straight_cnt = 0;
                stable_cnt = 0;
            }
            break;
            
        case 11:
            Turn_In_Place(turn_target_angle);
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if (diff > -1.0f && diff < 1.0f) {
                    stable_cnt++;
                    if (stable_cnt > 1) {
                        straight_target_angle = current_yaw;
                        state = 0; // 继续下一圈
                        stable_cnt = 0;
                        cooldown_cnt = 0;
                    }
                } else {
                    stable_cnt = 0;
                }
            }
            break;
        case 12:
            control_reset_runtime_state();
            Set_PWM_L(0);
            Set_PWM_R(0);
            break;
        default:
            // // 异常状态保护：所有计数清零并回到普通循迹。
            // state = 0;
            // left_black_cnt = 0;
            // stable_cnt = 0;
            // straight_cnt = 0;
            // cooldown_cnt = 0;
            break;
    }
}
void mode_4(void)
{
    static uint8_t state = 0U;
    static uint8_t left_black_cnt = 0U;
    static uint8_t stable_cnt = 0U;
    static uint8_t turn_count = 0U;
    static uint8_t lap_count = 0U;
    static uint8_t obstacle_hit_cnt = 0U;
    static uint16_t straight_cnt = 0U;
    static uint16_t cooldown_cnt = 0U;
    static uint16_t sr04_sample_cnt = 0U;
    static float base_angle = 0.0f;
    static float straight_target_angle = 0.0f;
    static float turn_target_angle = 0.0f;

    Gyro_Struct *JY61P_Data = get_angle();
    float current_yaw = JY61P_Data->z;

    if (state == 99U) {
        state = 0U;
        mode4_reset_state(&left_black_cnt, &stable_cnt, &turn_count, &lap_count,
                          &obstacle_hit_cnt, &straight_cnt, &cooldown_cnt,
                          &sr04_sample_cnt);
    }

    switch (state) {
        case 0U:
            Xunji_Speed();

            if (mode4_obstacle_detected(&sr04_sample_cnt, &obstacle_hit_cnt)) {
                base_angle = current_yaw;
                turn_target_angle = normalize_angle(base_angle - 45.0f);
                control_reset_runtime_state();
                state = 10U;
                stable_cnt = 0U;
                straight_cnt = 0U;
                cooldown_cnt = 0U;
                break;
            }

            if (digital(1)) {
                left_black_cnt++;
                if (left_black_cnt > 1U) {
                    straight_target_angle = current_yaw;
                    state = 1U;
                    left_black_cnt = 0U;
                    stable_cnt = 0U;
                    straight_cnt = 0U;
                }
            } else {
                left_black_cnt = 0U;
            }
            break;

        case 1U:
            Keep_Angle_Straight(straight_target_angle, 70);
            straight_cnt++;
            if (straight_cnt >= 30U) {
                turn_target_angle = normalize_angle(straight_target_angle + 70.0f);
                state = 2U;
                straight_cnt = 0U;
                stable_cnt = 0U;
            }
            break;

        case 2U:
            if (mode4_turn_done(turn_target_angle, current_yaw, &stable_cnt, 0U)) {
                turn_count++;
                if (turn_count >= MODE4_TURNS_PER_LAP) {
                    turn_count = 0U;
                    lap_count++;
                }

                if (lap_count >= mode4_target_laps()) {
                    control_reset_runtime_state();
                    Set_PWM_L(0);
                    Set_PWM_R(0);
                    car_started = 0U;
                    state = 99U;
                    break;
                }

                state = 3U;
                cooldown_cnt = 0U;
            }
            break;

        case 3U:
            Xunji_Speed();
            cooldown_cnt++;
            if (mode4_obstacle_detected(&sr04_sample_cnt, &obstacle_hit_cnt)) {
                base_angle = current_yaw;
                turn_target_angle = normalize_angle(base_angle - 45.0f);
                control_reset_runtime_state();
                state = 10U;
                stable_cnt = 0U;
                straight_cnt = 0U;
                cooldown_cnt = 0U;
                break;
            }
            if ((cooldown_cnt > 30U) && !digital(1)) {
                state = 0U;
                cooldown_cnt = 0U;
            }
            break;

        case 10U:
            if (mode4_turn_done(turn_target_angle, current_yaw, &stable_cnt, 1U)) {
                straight_target_angle = turn_target_angle;
                state = 11U;
                straight_cnt = 0U;
            }
            break;

        case 11U:
            Keep_Angle_Straight(straight_target_angle, 55);
            straight_cnt++;
            if ((straight_cnt >= 80U) ||
                ((straight_cnt > MODE4_LINE_SEARCH_MIN_TICKS) && any_black())) {
                turn_target_angle = base_angle;
                state = 12U;
                stable_cnt = 0U;
                straight_cnt = 0U;
            }
            break;

        case 12U:
            if (mode4_turn_done(turn_target_angle, current_yaw, &stable_cnt, 1U)) {
                state = 3U;
                cooldown_cnt = 0U;
                obstacle_hit_cnt = 0U;
                sr04_sample_cnt = 0U;
            }
            break;

        case 99U:
            Set_PWM_L(0);
            Set_PWM_R(0);
            break;

        default:
            state = 0U;
            mode4_reset_state(&left_black_cnt, &stable_cnt, &turn_count, &lap_count,
                              &obstacle_hit_cnt, &straight_cnt, &cooldown_cnt,
                              &sr04_sample_cnt);
            break;
    }
}
