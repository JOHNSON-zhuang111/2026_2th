#include "mode.h"

extern volatile u8 car_started;

#define MODE_COUNT_MAX 6U

#define MODE1_DEFAULT_LAPS 1U
#define MODE1_MIN_LAPS 1U
#define MODE1_MAX_LAPS 5U

/* control() is called every 20 ms in this project. */
#define MODE1_TICKS_PER_SECOND 50U
#define MODE1_MAX_LAP_TICKS (25U * MODE1_TICKS_PER_SECOND)

#define MODE1_CORNERS_PER_LAP 4U
#define MODE1_CORNER_STABLE_TICKS 2U
#define MODE1_CORNER_COOLDOWN_TICKS 18U
#define MODE1_START_IGNORE_TICKS 20U

typedef enum {
    MODE1_STATE_RUN = 0,
    MODE1_STATE_DONE,
    MODE1_STATE_TIMEOUT
} Mode1State;

static u8 mode_reset_pending[MODE_COUNT_MAX + 1U] = {0};
static volatile u8 mode1_target_laps = MODE1_DEFAULT_LAPS;

void mode_reset_runtime_state(void)
{
    for (u8 i = 1U; i <= MODE_COUNT_MAX; i++) {
        mode_reset_pending[i] = 1U;
    }
}

static u8 mode_take_reset(u8 mode)
{
    if ((mode <= MODE_COUNT_MAX) && (mode_reset_pending[mode] != 0U)) {
        mode_reset_pending[mode] = 0U;
        return 1U;
    }

    return 0U;
}

void mode1_set_target_laps(u8 laps)
{
    if (laps < MODE1_MIN_LAPS) {
        laps = MODE1_MIN_LAPS;
    } else if (laps > MODE1_MAX_LAPS) {
        laps = MODE1_MAX_LAPS;
    }

    mode1_target_laps = laps;
}

u8 mode1_get_target_laps(void)
{
    return mode1_target_laps;
}

u8 allwhite(void)
{
    return (digital(1) + digital(2) + digital(3) + digital(4) +
            digital(5) + digital(6) + digital(7) + digital(8) == 8U);
}

static u8 no_black(void)
{
    return (digital(1) + digital(2) + digital(3) + digital(4) +
            digital(5) + digital(6) + digital(7) + digital(8) == 0U);
}

static void mode_stop_car(void)
{
    control_reset_runtime_state();
    Set_PWM_L(0);
    Set_PWM_R(0);
}

float normalize_angle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

float angle_diff(float target_angle, float current_angle)
{
    return normalize_angle(target_angle - current_angle);
}

static u8 mode1_is_corner_pattern(TrackPattern pattern)
{
    return ((pattern == TRACK_PATTERN_CROSS) ||
            (pattern == TRACK_PATTERN_LEFT_SHARP) ||
            (pattern == TRACK_PATTERN_RIGHT_SHARP) ||
            (pattern == TRACK_PATTERN_LEFT_ROUNDABOUT_ENTRY) ||
            (pattern == TRACK_PATTERN_RIGHT_ROUNDABOUT_ENTRY) ||
            (pattern == TRACK_PATTERN_FULL_BLACK));
}

static void mode1_finish(void)
{
    mode_stop_car();
    car_started = 0U;
}

/*
 * Mode 1:
 * Follow the black square track, count one lap after four stable corner events,
 * and stop after the configured lap count N. N is clamped to 1..5.
 */
void mode_1(void)
{
    static Mode1State state = MODE1_STATE_RUN;
    static u8 lap_count = 0U;
    static u8 corner_count = 0U;
    static u8 corner_stable_count = 0U;
    static u16 corner_cooldown = 0U;
    static u16 start_ignore_count = 0U;
    static u16 lap_tick_count = 0U;

    if (mode_take_reset(1U)) {
        state = MODE1_STATE_RUN;
        lap_count = 0U;
        corner_count = 0U;
        corner_stable_count = 0U;
        corner_cooldown = 0U;
        start_ignore_count = 0U;
        lap_tick_count = 0U;
    }

    switch (state) {
        case MODE1_STATE_RUN:
        {
            Xunji_Speed();

            if (start_ignore_count < MODE1_START_IGNORE_TICKS) {
                start_ignore_count++;
                return;
            }

            lap_tick_count++;
            if (lap_tick_count > MODE1_MAX_LAP_TICKS) {
                state = MODE1_STATE_TIMEOUT;
                mode1_finish();
                return;
            }

            if (corner_cooldown > 0U) {
                corner_cooldown--;
                corner_stable_count = 0U;
                return;
            }

            TrackPattern pattern = Track_GetPattern();
            if (mode1_is_corner_pattern(pattern)) {
                corner_stable_count++;
                if (corner_stable_count >= MODE1_CORNER_STABLE_TICKS) {
                    corner_count++;
                    corner_stable_count = 0U;
                    corner_cooldown = MODE1_CORNER_COOLDOWN_TICKS;
                    LED_Toggle();

                    if (corner_count >= MODE1_CORNERS_PER_LAP) {
                        corner_count = 0U;
                        lap_count++;
                        lap_tick_count = 0U;

                        if (lap_count >= mode1_target_laps) {
                            state = MODE1_STATE_DONE;
                            mode1_finish();
                            return;
                        }
                    }
                }
            } else if (no_black()) {
                corner_stable_count = 0U;
            } else {
                corner_stable_count = 0U;
            }
            break;
        }

        case MODE1_STATE_DONE:
        case MODE1_STATE_TIMEOUT:
        default:
            mode1_finish();
            break;
    }
}

void mode_2(void)
{
    if (mode_take_reset(2U)) {
        mode_stop_car();
    }
    mode_stop_car();
}

void mode_3(void)
{
    if (mode_take_reset(3U)) {
        mode_stop_car();
    }
    mode_stop_car();
}

void mode_4(void)
{
    if (mode_take_reset(4U)) {
        mode_stop_car();
    }
    mode_stop_car();
}

void mode_5(void)
{
    if (mode_take_reset(5U)) {
        mode_stop_car();
    }
    mode_stop_car();
}

void mode_6(void)
{
    if (mode_take_reset(6U)) {
        mode_stop_car();
    }
    mode_stop_car();
}
