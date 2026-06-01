#ifndef __MODE_H__
#define __MODE_H__

#include "board.h"

void mode_1(void);
void mode_2(void);
void mode_3(void);
void mode_4(void);
void mode_5(void);
void mode_6(void);
// u8 mode_run_selected(void);
void mode_reset_runtime_state(void);
void mode1_set_target_laps(u8 laps);
u8 mode1_get_target_laps(void);
u8 allwhite(void);
float normalize_angle(float angle);
float angle_diff(float target_angle, float current_angle);
#endif
