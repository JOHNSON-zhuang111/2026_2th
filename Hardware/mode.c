#include "mode.h"
#include <math.h>

extern volatile u8 task_mode;
extern volatile u8 car_started;
extern volatile u8 set_quanshu;

#define MODE1_STRAIGHT_SPEED 30
#define MODE6_STRAIGHT_BEFORE_TURN_TICKS 30U
#define MODE1_TURNS_PER_LAP 4U
#define MODE3_TURN_EXIT_DEG 2.0f
#define MODE3_BRAKE_SPEED 80.0f
#define MODE3_BRAKE_TICKS 5U

/* 模式4参数：tick 都按 control() 的调用周期计数，实车上主要调这些值。 */
#define MODE4_TURNS_PER_LAP 4U
#define MODE4_OBSTACLE_DISTANCE_CM 18.0f
#define MODE4_OBSTACLE_HIT_COUNT 2U
#define MODE4_CORNER_STRAIGHT_TICKS 30U
#define MODE4_CORNER_COOLDOWN_TICKS 0U
#define MODE4_LINE_SEARCH_MIN_TICKS 30U
#define MODE4_LINE_SEARCH_TIMEOUT_TICKS 80U
#define MODE4_LINE_AFTER_HIT_TICKS 38U
#define MODE4_AVOID_SHIFT_TICKS 80U
#define MODE4_AVOID_PASS_TICKS 80U
#define MODE4_AVOID_SPEED 90
#define MODE4_AVOID_ANGLE_DEG 45.0f
#define MODE4_SR04_PRINTF_DEBUG 0U
#define MODE4_SR04_PRINTF_PERIOD_TICKS 5U


static uint8_t mode4_target_laps(void)
{
    /* 圈数由按键/VOFA 写入 set_quanshu，这里限制在题目要求的 1~5 圈。 */
    if (set_quanshu < 1U) {
        return 1U;
    }
    if (set_quanshu > 5U) {
        return 5U;
    }
    return set_quanshu;
}

static float mode4_read_distance(uint8_t state)
{
    /* 使用非阻塞超声波接口，避免在控制环里等待 ECHO 导致小车卡顿。 */
    float distance_cm = SR04_GetLengthNonBlocking();

#if MODE4_SR04_PRINTF_DEBUG
    static uint8_t print_cnt = 0U;

    print_cnt++;
    if (print_cnt >= MODE4_SR04_PRINTF_PERIOD_TICKS) {
        printf("M4 state:%u SR04:%.2f cm\r\n", state, distance_cm);
        print_cnt = 0U;
    }
#endif

    return distance_cm;
}

static uint8_t mode4_obstacle_detected(uint8_t *hit_count, float distance_cm)
{
    /* 非阻塞接口返回 0 表示暂时没有新测距结果，不把它当作“无障碍”清零。 */
    if (distance_cm <= 0.0f) {
        return (*hit_count >= MODE4_OBSTACLE_HIT_COUNT) ? 1U : 0U;
    }

    /* 连续多次小于阈值才认为真的有障碍，过滤超声波偶发跳变。 */
    if (distance_cm <= MODE4_OBSTACLE_DISTANCE_CM) {
        if (*hit_count < MODE4_OBSTACLE_HIT_COUNT) {
            (*hit_count)++;
        }
    } else {
        *hit_count = 0U;
    }

    return (*hit_count >= MODE4_OBSTACLE_HIT_COUNT) ? 1U : 0U;
}

static uint8_t mode4_turn_done(float target_angle,
                               float current_yaw,
                               uint8_t *stable_cnt,
                               uint8_t stable_need)
{
    /* 原地转向到目标角度，连续稳定若干个周期后才切换状态。 */
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
                              uint16_t *cooldown_cnt)
{
    /* 模式4重新进入时清掉所有运行计数，防止上一次任务残留状态影响发车。 */
    *left_black_cnt = 0U;
    *stable_cnt = 0U;
    *turn_count = 0U;
    *lap_count = 0U;
    *obstacle_hit_cnt = 0U;
    *straight_cnt = 0U;
    *cooldown_cnt = 0U;
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
            Keep_Angle_Straight(straight_target_angle, 60);
            straight_cnt++;
            if (straight_cnt >= 15) {
                turn_target_angle = normalize_angle(straight_target_angle + 80.0f);
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
                if (diff > -2.0f && diff < 2.0f) {
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
    static float last_good_yaw = 0.0f;         // 循迹居中时记录的稳定航向，避免边线触发瞬间车身歪斜。
    static uint8_t last_good_yaw_valid = 0U;
    static uint16_t lap_count = 0;             // 圈数计数。
    Gyro_Struct *JY61P_Data = get_angle();
    float current_yaw = JY61P_Data->z;

    if (!last_good_yaw_valid) {
        last_good_yaw = current_yaw;
        last_good_yaw_valid = 1U;
    }

    switch (state) {
        case 0:
            // 正常循迹；左侧探头连续检测到黑线后，进入短直行阶段。A-B
            Xunji_Speed();
            stable_cnt++;
            if ((digital(4) || digital(5)) && !digital(1) && !digital(8)) {
                last_good_yaw = current_yaw;
            }
            if (stable_cnt>0 && (digital(1))) {
                left_black_cnt++;
                if (left_black_cnt > 0) {
                    straight_target_angle = last_good_yaw;
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
                    if (stable_cnt > 0) {
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
                if (left_black_cnt > 0) {
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
                    if (stable_cnt > 0) {
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
                if (right_black_cnt > 0) {
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
            if (stable_cnt>60 && (digital(1))) {
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
            if (straight_cnt >= 18) {
                turn_target_angle = normalize_angle(straight_target_angle + 135.0f);
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
                if (diff > -MODE3_TURN_EXIT_DEG && diff < MODE3_TURN_EXIT_DEG) {
                    straight_target_angle = turn_target_angle;
                    state = 3;
                    stable_cnt = 0;
                    cooldown_cnt = 0;
                }
            }
            break;

        case 3:
            Keep_Angle_Straight(straight_target_angle,300);//B-D
            straight_cnt++;
            if (straight_cnt >= 0 && any_black()) {
                left_black_cnt++;
                if (left_black_cnt > 0) {
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
            if (straight_cnt >= 18) {
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
                if (diff > -MODE3_TURN_EXIT_DEG && diff < MODE3_TURN_EXIT_DEG) {
                    straight_target_angle = turn_target_angle;
                    state = 6;
                    stable_cnt = 0;
                    cooldown_cnt = 0;
                }
            }
            break;
        case 6:
            Xunji_Speed();//D-C
            stable_cnt++;
            if (stable_cnt >60 && (digital(8) || digital(7))) {
                right_black_cnt++;
                if (right_black_cnt > 0) {
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
            if (straight_cnt >= 18) {
                turn_target_angle = normalize_angle(straight_target_angle - 133.0f);
                state = 8;
                straight_cnt = 0;
                stable_cnt = 0;
            }
            break;
        case 8:
            Turn_In_Place(turn_target_angle);
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if (diff > -MODE3_TURN_EXIT_DEG && diff < MODE3_TURN_EXIT_DEG) {
                    straight_target_angle = turn_target_angle;
                    state = 9;
                    stable_cnt = 0;
                    cooldown_cnt = 0;
                }
            }
            break;
        case 9:
            Keep_Angle_Straight(straight_target_angle, 300);//停车
            straight_cnt++;
           if (straight_cnt >= 0 && any_black()) {
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
            if (straight_cnt >= 18) {
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
                if (diff > -MODE3_TURN_EXIT_DEG && diff < MODE3_TURN_EXIT_DEG) {
                    straight_target_angle = turn_target_angle;
                    state = 0; // 继续下一圈
                    stable_cnt = 0;
                    cooldown_cnt = 0;
                }
            }
            break;
        case 12:
            if (straight_cnt < MODE3_BRAKE_TICKS) {
                Left_Speed = -MODE3_BRAKE_SPEED;
                Right_Speed = -MODE3_BRAKE_SPEED;
                straight_cnt++;
            } else {
                control_reset_runtime_state();
                Set_PWM_L(0);
                Set_PWM_R(0);
            }
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
    /*
     * state:
     * 0  正常循迹，同时检测障碍和拐角边线
     * 1  拐角前短直行
     * 2  原地左转并累计圈数
     * 3  转弯后继续循迹，等待离开边线
     * 10~17 避障：偏出、通过、回线、回正
     * 99 停车保持
     */
    static uint8_t state = 0U;
    static uint8_t left_black_cnt = 0U;
    static uint8_t stable_cnt = 0U;
    static uint8_t turn_count = 0U;
    static uint8_t lap_count = 0U;
    static uint8_t obstacle_hit_cnt = 0U;
    static uint16_t straight_cnt = 0U;
    static uint16_t cooldown_cnt = 0U;
    static float base_angle = 0.0f;
    static float straight_target_angle = 0.0f;
    static float turn_target_angle = 0.0f;

    Gyro_Struct *JY61P_Data = get_angle();
    float current_yaw = JY61P_Data->z;
    float distance_cm = mode4_read_distance(state);

    if (state == 99U) {
        /* 停车态下一次重新进入时，先恢复到初始循迹状态。 */
        state = 0U;
        mode4_reset_state(&left_black_cnt, &stable_cnt, &turn_count, &lap_count,
                          &obstacle_hit_cnt, &straight_cnt, &cooldown_cnt);
    }

    switch (state) {
        case 0U:
            /* 正常循迹，同时检测前方障碍和左侧拐角边线。 */
            Xunji_Speed();

            if (mode4_obstacle_detected(&obstacle_hit_cnt, distance_cm)) {
                /* 发现障碍：记录当前航向，先向内侧偏转。 */
                base_angle = current_yaw;
                turn_target_angle = normalize_angle(base_angle + MODE4_AVOID_ANGLE_DEG);
                control_reset_runtime_state();
                state = 10U;
                stable_cnt = 0U;
                straight_cnt = 0U;
                break;
            }

            if (digital(1)|| digital(2) ) {
                
                
                    straight_target_angle = current_yaw;
                    state = 1U;
                    left_black_cnt = 0U;
                    stable_cnt = 0U;
                    straight_cnt = 0U;
                
            } else {
                left_black_cnt = 0U;
            }
            break;

        case 1U:
            /* 拐角前短直行一段，避免车身还压在线上就原地转向。 */
            Keep_Angle_Straight(straight_target_angle, MODE4_AVOID_SPEED);
            straight_cnt++;
            if (straight_cnt >= MODE4_CORNER_STRAIGHT_TICKS) {
                turn_target_angle = normalize_angle(straight_target_angle + 90.0f);
                state = 2U;
                straight_cnt = 0U;
                stable_cnt = 0U;
            }
            break;

        case 2U:
            /* 完成一次左转后计入拐角数，4 个拐角为 1 圈。 */
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
            /* 转弯后继续循迹，等待离开刚触发的边线。 */
            Xunji_Speed();
            cooldown_cnt++;

            if (mode4_obstacle_detected(&obstacle_hit_cnt, distance_cm)) {
                base_angle = current_yaw;
                turn_target_angle = normalize_angle(base_angle + MODE4_AVOID_ANGLE_DEG);
                control_reset_runtime_state();
                state = 10U;
                stable_cnt = 0U;
                straight_cnt = 0U;
                cooldown_cnt = 0U;
                break;
            }

            if ((cooldown_cnt > MODE4_CORNER_COOLDOWN_TICKS) &&
                !digital(1) && !digital(2)) {
                state = 0U;
                cooldown_cnt = 0U;
            }
            break;

        case 10U:
            /* 避障第1步：向内侧偏转。 */
            if (mode4_turn_done(turn_target_angle, current_yaw, &stable_cnt, 0U)) {
                straight_target_angle = turn_target_angle;
                state = 11U;
                straight_cnt = 0U;
            }
            break;

        case 11U:
            /* 避障第2步：斜向离开原边线，拉开横向距离。 */
            Keep_Angle_Straight(straight_target_angle, MODE4_AVOID_SPEED);
            straight_cnt++;
            if (straight_cnt >= 45) {
                turn_target_angle = base_angle;
                state = 12U;
                stable_cnt = 0U;
                straight_cnt = 0U;
            }
            break;

        case 12U:
            /* 避障第3步：转回原航向。 */
            if (mode4_turn_done(turn_target_angle, current_yaw, &stable_cnt, 0U)) {
                straight_target_angle = base_angle;
                state = 13U;
                straight_cnt = 0U;
            }
            break;

        case 13U:
            /* 避障第4步：保持原航向，从障碍物侧面通过。 */
            Keep_Angle_Straight(straight_target_angle, MODE4_AVOID_SPEED);
            straight_cnt++;
            if (straight_cnt >= 38) {
                turn_target_angle = normalize_angle(base_angle - MODE4_AVOID_ANGLE_DEG);
                state = 14U;
                stable_cnt = 0U;
                straight_cnt = 0U;
            }
            break;

        case 14U:
            /* 避障第5步：向边线方向偏转，开始找回循迹线。 */
            if (mode4_turn_done(turn_target_angle, current_yaw, &stable_cnt, 0U)) {
                straight_target_angle = turn_target_angle;
                state = 15U;
                straight_cnt = 0U;
            }
            break;

        case 15U:
            /* 避障第6步：斜向回线，任意探头碰线后进入短直行。 */
            Keep_Angle_Straight(straight_target_angle, MODE4_AVOID_SPEED);
            straight_cnt++;
            if (any_black()) {
                state = 16U;
                straight_cnt = 0U;
            }
            break;

        case 16U:
            /* 避障第7步：碰线后继续斜直行一小段，再准备回正。 */
            Keep_Angle_Straight(straight_target_angle, 75);
            straight_cnt++;
            if (straight_cnt >= MODE4_LINE_AFTER_HIT_TICKS) {
                turn_target_angle = base_angle;
                state = 17U;
                stable_cnt = 0U;
                straight_cnt = 0U;
            }
            break;

        case 17U:
            /* 避障第8步：回到原航向，重新交给循迹流程。 */
            if (mode4_turn_done(turn_target_angle, current_yaw, &stable_cnt, 0U)) {
                obstacle_hit_cnt = 0U;
                state = 3U;
                cooldown_cnt = 0U;
                straight_cnt = 0U;
            }
            break;

        case 99U:
            /* 停车保持。 */
            Set_PWM_L(0);
            Set_PWM_R(0);
            break;

        default:
            state = 0U;
            mode4_reset_state(&left_black_cnt, &stable_cnt, &turn_count, &lap_count,
                              &obstacle_hit_cnt, &straight_cnt, &cooldown_cnt);
            break;
    }
}
