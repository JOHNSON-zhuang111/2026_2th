#ifndef __ERROR_H
#define __ERROR_H


extern uint8_t right_angle_flag;//每次任务进行的圈数
extern int err;
int Error_Calculate(void);
int get_fused_error(int sensor_err, short gyro_z);
float Right_err(void);
int get_fused_error_2(int sensor_err, float gyro_z);

#endif
