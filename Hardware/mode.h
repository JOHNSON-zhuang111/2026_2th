#ifndef __MODE_H__
#define __MODE_H__

#include "board.h"

void mode_1(void);
void mode_2(void);
void mode_3(void);
void mode_4(void);

float normalize_angle(float angle);
float angle_diff(float target_angle, float current_angle);


#endif
