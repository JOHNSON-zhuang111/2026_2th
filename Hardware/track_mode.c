#include "board.h"
#include "mode.h"

/* 以下 tick 均按 control() 的周期累计，用来滤除瞬时误判和控制动作持续时间。 */
#define TRACK_EVENT_STABLE_TICKS        3U    /* 路况连续识别到多少个控制周期才算有效，防止单次抖动误触发。 */
#define TRACK_CROSS_PASS_TICKS          25U   /* 检测到十字/全黑线后，锁定当前航向直行穿过的持续周期数。 */
#define TRACK_TURN_STABLE_TICKS         3U    /* 锐角转弯时，角度误差连续稳定多少个周期后认为转弯完成。 */
#define TRACK_COOLDOWN_TICKS            30U   /* 特殊动作完成后的冷却周期数，避免同一个路口/标志线被重复触发。 */
#define TRACK_ROUNDABOUT_MIN_TICKS      60U   /* 进入环岛后的最短循迹周期数，在这之前不允许判断已经出环。 */
#define TRACK_ROUNDABOUT_TIMEOUT_TICKS  220U  /* 环岛状态最长持续周期数，超过后强制退出，避免一直卡在环岛状态。 */
#define TRACK_SHARP_TURN_DEGREE         75.0f /* 锐角急转时相对当前 yaw 增减的目标角度，实际可根据赛道调大/调小。 */

typedef enum {
    TRACK_MODE_NORMAL = 0,  /* 普通循迹，持续调用 Xunji_Speed()。 */
    TRACK_MODE_CROSS_PASS,  /* 十字/全黑线：锁定航向短直行穿过。 */
    TRACK_MODE_SHARP_TURN,  /* 锐角：按陀螺仪目标角原地急转。 */
    TRACK_MODE_ROUNDABOUT,  /* 环岛：保持循迹，等待出口/超时退出。 */
    TRACK_MODE_COOLDOWN     /* 动作结束后的冷却期，避免同一标志反复触发。 */
} TrackModeState;

static uint8_t track_roundabout_finished(TrackPattern pattern, uint16_t ticks)
{
    /* 刚进环岛时先保护一段时间，防止入口形态刚变化就误判已经出环。 */
    if (ticks < TRACK_ROUNDABOUT_MIN_TICKS) {
        return 0U;
    }

    /* 保护时间后恢复普通线形或短暂丢线，都认为已经接近出口。 */
    if ((pattern == TRACK_PATTERN_NORMAL) || (pattern == TRACK_PATTERN_LOST)) {
        return 1U;
    }

    /* 如果一直识别不到出口，超时强制退出，避免卡死在环岛状态。 */
    return (ticks >= TRACK_ROUNDABOUT_TIMEOUT_TICKS) ? 1U : 0U;
}

void Track_Mode6Enhanced(void)
{
    /* 状态变量保持在函数内部，mode 6 每个控制周期调用一次即可。 */
    static TrackModeState state = TRACK_MODE_NORMAL;
    static uint8_t stable_cnt = 0U;
    static uint16_t state_ticks = 0U;
    static float locked_angle = 0.0f;
    static float turn_target_angle = 0.0f;
    static TrackPattern pending_pattern = TRACK_PATTERN_NORMAL;

    Gyro_Struct *gyro = get_angle();
    float current_yaw = gyro->z;
    TrackPattern pattern = Track_GetPattern();

    switch (state) {
        case TRACK_MODE_NORMAL:
            /* 默认仍然使用原来的 PD 循迹，复杂路况只在稳定出现后接管。 */
            Xunji_Speed();

            if ((pattern == TRACK_PATTERN_CROSS) ||
                (pattern == TRACK_PATTERN_FULL_BLACK)) {
                /* 十字和全黑线先连续确认，再锁定当前航向直行穿过。 */
                stable_cnt++;
                if (stable_cnt >= TRACK_EVENT_STABLE_TICKS) {
                    locked_angle = current_yaw;
                    state = TRACK_MODE_CROSS_PASS;
                    state_ticks = 0U;
                    stable_cnt = 0U;
                    pending_pattern = pattern;
                }
            } else if ((pattern == TRACK_PATTERN_LEFT_SHARP) ||
                       (pattern == TRACK_PATTERN_RIGHT_SHARP)) {
                /* 锐角入口连续确认后，计算一个相对当前 yaw 的急转目标角。 */
                stable_cnt++;
                if (stable_cnt >= TRACK_EVENT_STABLE_TICKS) {
                    float delta = (pattern == TRACK_PATTERN_LEFT_SHARP) ?
                                  TRACK_SHARP_TURN_DEGREE :
                                  -TRACK_SHARP_TURN_DEGREE;
                    turn_target_angle = normalize_angle(current_yaw + delta);
                    state = TRACK_MODE_SHARP_TURN;
                    state_ticks = 0U;
                    stable_cnt = 0U;
                    pending_pattern = pattern;
                }
            } else if ((pattern == TRACK_PATTERN_LEFT_ROUNDABOUT_ENTRY) ||
                       (pattern == TRACK_PATTERN_RIGHT_ROUNDABOUT_ENTRY)) {
                /* 环岛入口先进入环岛状态，具体沿线运动仍交给 Xunji_Speed()。 */
                stable_cnt++;
                if (stable_cnt >= TRACK_EVENT_STABLE_TICKS) {
                    state = TRACK_MODE_ROUNDABOUT;
                    state_ticks = 0U;
                    stable_cnt = 0U;
                    pending_pattern = pattern;
                }
            } else {
                stable_cnt = 0U;
            }
            break;

        case TRACK_MODE_CROSS_PASS:
            /* 穿过十字时不要被横线干扰，按进入十字前的航向短直行。 */
            Keep_Angle_Straight(locked_angle, Basic_Speed);
            state_ticks++;
            if (state_ticks >= TRACK_CROSS_PASS_TICKS) {
                state = TRACK_MODE_COOLDOWN;
                state_ticks = 0U;
            }
            break;

        case TRACK_MODE_SHARP_TURN:
            /* 锐角转弯用陀螺仪闭环，角度连续稳定后退出。 */
            Turn_In_Place(turn_target_angle);
            {
                float diff = angle_diff(turn_target_angle, current_yaw);
                if ((diff > -3.0f) && (diff < 3.0f)) {
                    stable_cnt++;
                    if (stable_cnt >= TRACK_TURN_STABLE_TICKS) {
                        state = TRACK_MODE_COOLDOWN;
                        state_ticks = 0U;
                        stable_cnt = 0U;
                    }
                } else {
                    stable_cnt = 0U;
                }
            }
            break;

        case TRACK_MODE_ROUNDABOUT:
            /* 环岛内继续循迹，出口判断由 track_roundabout_finished() 统一处理。 */
            Xunji_Speed();
            state_ticks++;
            if (track_roundabout_finished(pattern, state_ticks)) {
                state = TRACK_MODE_COOLDOWN;
                state_ticks = 0U;
            }
            break;

        case TRACK_MODE_COOLDOWN:
            /* 冷却期间继续循迹，但暂时不响应新的复杂路况，防止刚离开标志线又触发。 */
            Xunji_Speed();
            state_ticks++;
            if (state_ticks >= TRACK_COOLDOWN_TICKS) {
                state = TRACK_MODE_NORMAL;
                state_ticks = 0U;
                stable_cnt = 0U;
                pending_pattern = TRACK_PATTERN_NORMAL;
            }
            break;

        default:
            /* 异常状态保护：回到普通循迹并清空计数。 */
            state = TRACK_MODE_NORMAL;
            stable_cnt = 0U;
            state_ticks = 0U;
            pending_pattern = TRACK_PATTERN_NORMAL;
            break;
    }

    /* 目前 pending_pattern 只用于保留触发来源，避免未使用变量告警。 */
    (void) pending_pattern;
}
