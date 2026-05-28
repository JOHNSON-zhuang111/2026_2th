#ifndef __CONTROL_H
#define __CONTROL_H
#include "board.h"

typedef struct {
	float Target;
	float Actual;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float Error2;
	float ErrorInt;
	
	float OutMax;
	float OutMin;
} PID_t;
extern PID_t speed_left;
extern PID_t speed_right;
extern PID_t turn_speed_left;
extern PID_t turn_speed_right;
extern PID_t Turn;
extern PID_t Straight;
extern PID_t Xunji;
void control_reset_runtime_state(void);

void control(void);



void Xunji_Speed(void);
float Place_Control(PID_t *p);
// float PID_Control(float NowPoint, float SetPoint, float *TURN_PID);
void PID_Update(PID_t *p);
void Turn_In_Place(float target_angle);
void Keep_Angle_Straight(float target_angle, int base_speed); 
#endif