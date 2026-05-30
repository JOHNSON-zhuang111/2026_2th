#include "control.h"
#include "bsp_gyro.h"
#include "error.h"
#include "mode.h"
extern u8 task_mode;
int sensor_err=0,final_err=0;
int Basic_Speed=80;    				//基础速度，在这里修改速度，但是元素要先注释掉
#define DRIVE_PWM_LIMIT 100
float Left_Speed=0,Right_Speed=0;
float Turn_factor=1.0;
float Place_PD[2] = {7,4};//本次调试，原来KP值为5,KD值为3
 //转向环PID参数
static float Speed_Out_L=0,Speed_Out_R=0,Place_Out=0;//两个环的输出
float MA_RPM=0,MB_RPM=0;
float yaw=0;

#define  SPEED_FILTER_SIZE 5
static float speed_buffer[2][SPEED_FILTER_SIZE] ={0};
static u8 speed_index[2]={0};
// 1. 循迹速度环PID参数（兼作实际运行状态变量）
PID_t speed_left ={
    .Kp=0.165,    // 继续减小增量式的 Kp（抑制高频突变毛刺）
    .Ki=0.045,    // 保持积分不变
    .Kd=0.0,    
    .OutMax=100, .OutMin=-100,
};
PID_t speed_right ={
    .Kp=0.165,  
    .Ki=0.045,  
    .Kd=0.0,  
    .OutMax=100, .OutMin=-100,
};

// 2. 原地转角速度环PID参数（当启动转圈时临时调用这组参数）
PID_t turn_speed_left ={
    .Kp=0.28, 
    .Ki=0.045,   
    .Kd=0.0,
	.OutMax=50, .OutMin=-50,
};
PID_t turn_speed_right ={
    .Kp=0.28, 
    .Ki=0.045,   
    .Kd=0.0,
	.OutMax=50, .OutMin=-50,
};

// 3. 位置式PID参数
PID_t Xunji ={
    .Kp=3.0,    // 继续减小增量式的 Kp（抑制高频突变毛刺）
    .Ki=0.0,    // 保持积分不变
    .Kd=1.5,    
    .OutMax=30, .OutMin=-30,
	};
PID_t Turn ={
    .Kp=2.0,    // 恢复为较温和的比例
    .Ki=0.0,    
    .Kd=1.0,    // 保留适度微分阻尼
    .OutMax=50, .OutMin=-50, // 核心修改：死死卡住最大允许目标底盘转速，不让它顶到PWM40上限限制速度环暴走
	};
PID_t Straight ={
    .Kp=1.5,    
    .Ki=0.0,    
    .Kd=3.0,    
    .OutMax=50, .OutMin=-50,
	};

// 原地转向角速度内环参数，主要用于锐角急转。
TurnRate_t TurnRate ={
    .DtSec=0.02f,      // 控制周期，当前定时器约 20ms。
    .TargetKp=3.0f,    // 角度误差转换为目标角速度的比例。
    .TargetMax=120.0f, // 目标角速度最大值，防止起转过猛。
    .RateKp=0.22f,     // 角速度内环 P 参数。
    .RateKd=0.04f,     // 角速度内环 D 参数，用于抑制过冲。
    .PwmMax=45.0f,     // 角速度内环输出的最大左右轮差速。
};
static void PID_ResetRuntime(PID_t *p)
{
	p->Target = 0.0f;
	p->Actual = 0.0f;
	p->Out = 0.0f;
	p->Error0 = 0.0f;
	p->Error1 = 0.0f;
	p->Error2 = 0.0f;
	p->ErrorInt = 0.0f;
}

void control_reset_runtime_state(void)
{
	sensor_err = 0;
	final_err = 0;
	Left_Speed = 0;
	Right_Speed = 0;
	Speed_Out_L = 0.0f;
	Speed_Out_R = 0.0f;
	Place_Out = 0.0f;
	MA_RPM = 0.0f;
	MB_RPM = 0.0f;
	yaw = 0.0f;
	memset(speed_buffer, 0, sizeof(speed_buffer));
	speed_index[0] = 0U;
	speed_index[1] = 0U;
	Get_Encoder_countA = 0;
	Get_Encoder_countB = 0;
	PID_ResetRuntime(&speed_left);
	PID_ResetRuntime(&speed_right);
}


void control(void)
{
	u8 mode = task_mode;

	if (mode < 1U || mode > 6U)
	{
		mode = 1U;
	}

	switch (mode)
	{
		case 1U:
			// mode_1();
			Turn_In_Place(90);
			break;
		case 2U:
			// mode_2();
            Xunji_Speed();
			break;
		case 3U:
			// mode_3();
			Keep_Angle_Straight(0.0f, 50);
			break;
		case 4U:
			mode_4();
			break;
		case 5U:
			mode_5();
			break;
		case 6U:
			mode_6();
			break;
		default:
			Track_Mode6Enhanced();
			break;
	}
    
    
	

// Keep_Angle_Straight(0.0f, Basic_Speed);
// Turn_In_Place(45.0) ;
// Xunji_Speed() ;


 	// 1. 计算电机转速
// 使用Calculate_Motor_RPM函数计算左右轮的转速
// 参数1: 编码器计数，参数2: 采样时间（毫秒）
MA_RPM = Calculate_Motor_RPM(Get_Encoder_countA, 20); // 获取左轮转速 (单位: RPM)
MB_RPM = Calculate_Motor_RPM(Get_Encoder_countB, 20); // 获取右轮转速 (单位: RPM)


// 2. 速度滤波处理
// 对左轮速度进行移动平均滤波
// 将当前速度值存入缓冲区
	speed_buffer[0][speed_index[0]] = MA_RPM;
// 更新缓冲区索引（循环缓冲区）
	speed_index[0] = (speed_index[0] + 1) % SPEED_FILTER_SIZE;
// 计算缓冲区中所有值的总和
	float sum1 = 0;
	for(int j = 0; j < SPEED_FILTER_SIZE; j++)
	{
		sum1 += speed_buffer[0][j];
	}
// 计算平均值作为滤波后的速度
	MA_RPM = sum1 / SPEED_FILTER_SIZE;

// 对右轮速度进行移动平均滤波
// 将当前速度值存入缓冲区
	speed_buffer[1][speed_index[1]] = MB_RPM;
// 更新缓冲区索引（循环缓冲区）
	speed_index[1] = (speed_index[1] + 1) % SPEED_FILTER_SIZE;
// 计算缓冲区中所有值的总和
	float sum2 = 0;
	for(int j = 0; j < SPEED_FILTER_SIZE; j++)
	{
		sum2 += speed_buffer[1][j];
	}
// 计算平均值作为滤波后的速度
	MB_RPM = sum2 / SPEED_FILTER_SIZE;

// 3. 将滤波后的真实RPM转速作为当前实际速度喂给PID
	speed_left.Actual = MA_RPM; // 将转速转换为每分钟转数（RPM），假设编码器计数是每20ms的增量
	speed_right.Actual = MB_RPM; // 将转速转换为每分钟转数（RPM），假设编码器计数是每20ms的增量
	//printf("Actual:%.2f, %.2f\n\r", speed_left.Actual, speed_right.Actual);
// 4. 重置编码器计数
// 将编码器计数重置为0，为下一次测量做准备
	Get_Encoder_countB = Get_Encoder_countA = 0;


    speed_left.Target=Left_Speed; //转速目标值
	PID_Update(&speed_left);
 	Speed_Out_L=speed_left.Out;
	if(Speed_Out_L>DRIVE_PWM_LIMIT){Speed_Out_L=DRIVE_PWM_LIMIT;}
	if(Speed_Out_L<-DRIVE_PWM_LIMIT){Speed_Out_L=-DRIVE_PWM_LIMIT;}
	
	Set_PWM_L(Speed_Out_L);
	
		
	
	 speed_right.Target=Right_Speed;//转速目标值
	 PID_Update(&speed_right);
	 Speed_Out_R=speed_right.Out;
	 if(Speed_Out_R>DRIVE_PWM_LIMIT){Speed_Out_R=DRIVE_PWM_LIMIT;}
	 if(Speed_Out_R<-DRIVE_PWM_LIMIT){Speed_Out_R=-DRIVE_PWM_LIMIT;}
	//printf("%.2f,%.2f,%.2f\n",speed_right.Out,Speed_Out_R,speed_right.Actual);
	Set_PWM_R(Speed_Out_R);
	//printf("%.2f,%.2f\n",speed_left.Out,speed_right.Out);
	printf("speed: %.2f, %.2f, %.2f, %.2f\n", speed_left.Target, speed_right.Target, speed_left.Actual, speed_right.Actual);
	//printf("%.3f, %.3f, %.3f, %.3f\n", speed_left.Kp, speed_left.Ki,speed_right.Kp, speed_right.Ki);
	//printf("%.3f, %.3f\n", Turn.Kp, Turn.Kd);
	//printf("%d\n",Basic_Speed );
}













/************************位置式转向环pd***************************/
float Place_Control(PID_t *p) //PD控制位置环
{
    float KP, KD; 
    float NowError, Out; 
    NowError = p->Target - p->Actual; // 当前误差 
    KP = p->Kp; 
    KD = p->Kd; 
    Out = KP * NowError + KD *(NowError-p->Error1); // PD 输出值 
    p->Error1 = NowError; //更新上次误差 
    // 输出限制
    if (Out > p->OutMax) { Out = p->OutMax; }
    if (Out < p->OutMin) { Out = p->OutMin; }
    return Out; 
}

/************************位置式速度环pd***************************/
// float PID_Control(float NowPoint, float SetPoint, float *TURN_PID) //PI控制速度环
// {
// 	  static float Integral,LastError = 0;      // 积分累积量（静态变量）
// 		float KP,KI,KD,Out,NowError; 
// 		KP = *TURN_PID; 
// 		KI = *(TURN_PID+1); 
// 		KD = *(TURN_PID+2); 
	
//     NowError = SetPoint - NowPoint;
    
// //    // 积分项累加（需限幅防饱和）
// //    Integral += NowError;
// //    Integral = Min_Max(Integral, -INTEGRAL_MAX, INTEGRAL_MAX); // 示例：INTEGRAL_MAX=1000
    
// 		//这里pi、pd我都试过了pd效果更好一点
//     Out = KP * NowError + KI * Integral+KD *(NowError-LastError);
   
//     return Out;
// }

/************************增量pid更新***************************/
void PID_Update(PID_t *p)
{
	/* 记录历史误差 */
	p->Error2 = p->Error1;					// 记录上上次误差
	p->Error1 = p->Error0;					// 记录上次误差
	p->Error0 = p->Target - p->Actual;		// 获取本次误差
	
	/* 增量计算 */
	float inc = p->Kp * (p->Error0 - p->Error1)
		      + p->Ki * p->Error0
		      + p->Kd * (p->Error0 - 2.0f * p->Error1 + p->Error2);
	
	/* 累加增量到Out，使其成为实际输出PWM值 */
	p->Out += inc;
	
	/* 总输出限幅（防止PWM越界）*/
	if (p->Out > p->OutMax) { p->Out = p->OutMax; }
	if (p->Out < p->OutMin) { p->Out = p->OutMin; }
}
/************************循迹差速计算***************************/
/**
 * @brief 速度差异计算函数
 * @note 根据转向控制量调整左右轮速度，实现转向功能
 * @param 无
 * @return 无
 * @details 根据Place_Out的值计算左右轮的速度差异，实现转向控制
 *          转向力度与基础速度相关，速度越快转向力度越大
 */

void Xunji_Speed(void) 
{ 
	 Xunji.Actual =Error_Calculate();  //获取传感器误差
	 Xunji.Target = 0; //目标误差为0
 
    //转向环	
	Place_Out=Place_Control(&Xunji);
	// // //  printf("%.2f\n",Place_Out);
	float k;  // 转向系数
	
	
	// 转向环的输出
	// 输出的速度取决于基础速度的大小，所以速度快了之后转向力度也会跟着变大
	
	if(Place_Out >= 0) //err
		{ 
			k = Place_Out * 0.01; // 计算转向系数
			Left_Speed = Basic_Speed * (1 - k);   // 如果Place_Out大于0，左轮减速，右轮加速
			Right_Speed = Basic_Speed * (1 + k*Turn_factor); // 右轮加速，Turn_factor为转向因子
		} 
		else 
		{ 
		k = -Place_Out * 0.01; // 取绝对值计算转向系数
		Left_Speed = Basic_Speed * (1 + k*Turn_factor); // 如果Place_Out小于0，左轮加速，右轮减速
		Right_Speed = Basic_Speed * (1 - k); // 右轮减速
		} 
		//printf("Target_L:%d, Target_R:%d\r\n", Left_Speed, Right_Speed);
}




//参数 target_angle: 目标绝对角度
void Turn_In_Place(float target_angle) 
{
	
    float turn_error;
    float Turn_Out;
	
	// Turn.Target = target_angle;
    // 切换底层速度环为【原地转角参数】
    speed_left.Kp = turn_speed_left.Kp; speed_left.Ki = turn_speed_left.Ki; speed_left.Kd = turn_speed_left.Kd;
    speed_right.Kp = turn_speed_right.Kp; speed_right.Ki = turn_speed_right.Ki; speed_right.Kd = turn_speed_right.Kd;
    //1. 读取陀螺仪当前 Yaw 角
    Gyro_Struct *JY61P_Data = get_angle();
    Turn.Actual = JY61P_Data->z;
    printf("Turn.Actual:%.2f\r\n", Turn.Actual);

    // 【方案2：动态目标限幅 (虚拟目标点)】
    // 使得目标角度始终只超前实际角度一小段距离，避免产生大阶跃误差
    float max_lead = 25.0f; // 相对放宽一点引导角，由于已经限制了 OutMax ，不用担心过载
    float real_diff = target_angle - Turn.Actual;
    
    // 处理 180 到 -180 的临界跳变
    while (real_diff > 180.0f)  real_diff -= 360.0f;
    while (real_diff < -180.0f) real_diff += 360.0f;

    if (real_diff > max_lead) {
        Turn.Target = Turn.Actual + max_lead;
    } else if (real_diff < -max_lead) {
        Turn.Target = Turn.Actual - max_lead;
    } else {
        Turn.Target = Turn.Actual + real_diff; // 使用修正后的差值计算目标
    }

    // 2. 计算控制用的角度误差 (被限幅到 max_lead 以内)
    // 注意：当距离终点不足 max_lead 时，Turn.Target 就是真正的 target_angle
    turn_error = Turn.Target - Turn.Actual;
    // printf("turn_error:%.2f\r\n", turn_error);

	
    // 4. 计算 PD 转向输出
    // 复用你写好的 Place_Control: 输入当前角度和目标角度
    Turn_Out = Place_Control(&Turn);
    
	//5. 靠近目标角时自动降速，避免惯性冲过头
	{
		float abs_err = ABS(real_diff);
		float out_limit = 45.0f;

		if(abs_err < 60.0f) out_limit = 35.0f;
		if(abs_err < 30.0f) out_limit = 25.0f;
		if(abs_err < 15.0f) out_limit = 15.0f;
		if(abs_err < 8.0f)  out_limit = 8.0f;
		if(abs_err < 4.0f)  out_limit = 4.0f;

		if(Turn_Out > out_limit) Turn_Out = out_limit;
		if(Turn_Out < -out_limit) Turn_Out = -out_limit;
	}

    // 6. 将转向输出给到左右轮 (原地转弯：左轮和右轮互为反向)
    // 这里的极性 (谁为正谁为负) 取决于你的陀螺仪正方向
    Left_Speed = -Turn_Out; 
    Right_Speed = Turn_Out; 
    //printf("Turn_Out:%.2f  \n", Turn_Out);
	// 7. 退出条件：进入小误差区后需稳定若干个控制周期，避免刚好掠过目标角
	if((real_diff > -1.0f) && (real_diff < 1.0f))
    {
		
			Left_Speed = 0;
			Right_Speed = 0;
			

			// 立刻清零底层速度环 PID 的累积输出，防止残留PWM继续带动电机
			speed_left.Out = 0;
			speed_right.Out = 0;
			Turn.Out = 0;
			Turn.Error0 = 0;
			Turn.Error1 = 0;
			Turn.Error2 = 0;
		
	}
	
}

// 原地转向：角度外环 + 角速度内环，优先用于锐角急转。
void Turn_In_Place_Rate(float target_angle)
{
    static u8 initialized = 0U;
    static float last_yaw = 0.0f;
    static float last_target_angle = 999.0f;
    static float last_rate_error = 0.0f;

    speed_left.Kp = turn_speed_left.Kp; speed_left.Ki = turn_speed_left.Ki; speed_left.Kd = turn_speed_left.Kd;
    speed_right.Kp = turn_speed_right.Kp; speed_right.Ki = turn_speed_right.Ki; speed_right.Kd = turn_speed_right.Kd;

    Gyro_Struct *JY61P_Data = get_angle();
    float current_yaw = JY61P_Data->z;
    float target_change = target_angle - last_target_angle;
    if (target_change < 0.0f) {
        target_change = -target_change;
    }

    if ((initialized == 0U) || (target_change > 0.5f)) {
        initialized = 1U;
        last_yaw = current_yaw;
        last_target_angle = target_angle;
        last_rate_error = 0.0f;
    }

    // 外环：角度误差越大，目标角速度越大；接近目标时自动降速。
    float angle_error = angle_diff(target_angle, current_yaw);
    float abs_angle_error = (angle_error < 0.0f) ? -angle_error : angle_error;
    float target_rate = angle_error * TurnRate.TargetKp;
    float target_rate_limit = TurnRate.TargetMax;

    if (abs_angle_error < 30.0f) {
        target_rate_limit = 80.0f;
    }
    if (abs_angle_error < 15.0f) {
        target_rate_limit = 45.0f;
    }
    if (abs_angle_error < 6.0f) {
        target_rate_limit = 20.0f;
    }

    if (target_rate > target_rate_limit) {
        target_rate = target_rate_limit;
    }
    if (target_rate < -target_rate_limit) {
        target_rate = -target_rate_limit;
    }

    // 内环：用相邻两次 yaw 差分估算当前角速度，再做 PD 修正。
    float yaw_delta = angle_diff(current_yaw, last_yaw);
    float current_rate = yaw_delta / TurnRate.DtSec;
    float rate_error = target_rate - current_rate;
    float turn_out = TurnRate.RateKp * rate_error + TurnRate.RateKd * (rate_error - last_rate_error);
    float turn_out_limit = TurnRate.PwmMax;

    if (abs_angle_error < 15.0f) {
        turn_out_limit = 25.0f;
    }
    if (abs_angle_error < 6.0f) {
        turn_out_limit = 12.0f;
    }

    if (turn_out > turn_out_limit) {
        turn_out = turn_out_limit;
    }
    if (turn_out < -turn_out_limit) {
        turn_out = -turn_out_limit;
    }

    // 角度接近且角速度很小，认为转向基本停稳，清掉速度环残留。
    if ((abs_angle_error < 1.0f) && (current_rate > -8.0f) && (current_rate < 8.0f)) {
        turn_out = 0.0f;
        last_rate_error = 0.0f;
        speed_left.Out = 0.0f;
        speed_right.Out = 0.0f;
        Turn.Out = 0.0f;
        Turn.Error0 = 0.0f;
        Turn.Error1 = 0.0f;
        Turn.Error2 = 0.0f;
    }

    Left_Speed = -turn_out;
    Right_Speed = turn_out;
    last_rate_error = rate_error;
    last_yaw = current_yaw;
}

// ==========================================
// 附加功能：利用陀螺仪保持特定角度走直线
// ==========================================
// 参数 target_angle: 你希望车头锁定的绝对角度
// 参数 base_speed: 你希望车子直行的基础速度 (比如30)
void Keep_Angle_Straight(float target_angle, int base_speed)
{
    float Angle_Out;
    static float last_target_angle = 999.0f;
    float target_change = target_angle - last_target_angle;
    if (target_change < 0.0f) target_change = -target_change;
    if (target_change > 0.01f) {
        Straight.Out = 0.0f;
        Straight.Error0 = 0.0f;
        Straight.Error1 = 0.0f;
        Straight.Error2 = 0.0f;
        last_target_angle = target_angle;
    }
    // 直行纠偏的PD参数 (P小一点只要能微调回来就行，D给一点用来防震荡画龙)
	speed_left.Kp = turn_speed_left.Kp; speed_left.Ki = turn_speed_left.Ki; speed_left.Kd = turn_speed_left.Kd;
    speed_right.Kp = turn_speed_right.Kp; speed_right.Ki = turn_speed_right.Ki; speed_right.Kd = turn_speed_right.Kd;
    // 1. 获取当前偏航角
    Gyro_Struct *JY61P_Data = get_angle();
    float current_yaw = JY61P_Data->z;
    float straight_diff = target_angle - current_yaw;
    
    // 处理 180 到 -180 的临界跳变
    while (straight_diff > 180.0f)  straight_diff -= 360.0f;
    while (straight_diff < -180.0f) straight_diff += 360.0f;

    Straight.Actual = current_yaw;
    Straight.Target = current_yaw + straight_diff;
    
    // 2. 将目标角度与当前角度之差，丢入你的位置式 PD 控制器计算差速修正值
    Angle_Out = Place_Control(&Straight);
    
    // 3. 限制最大修正力度 (不能因为车子被撞歪了就原地乱转)
    if(Angle_Out > 20) Angle_Out = 20;
    if(Angle_Out < -20) Angle_Out = -20;

    // 4. 将修正量叠加给左右电机的基础速度
	// 注意极性：如果发现车子偏了之后反倒越来越偏，把这里的 + 和 - 互换即可！
    Left_Speed = base_speed - Angle_Out; 
    Right_Speed = base_speed + Angle_Out;
    if (Left_Speed < 0.0f) Left_Speed = 0.0f;
    if (Right_Speed < 0.0f) Right_Speed = 0.0f;
    if (Left_Speed > DRIVE_PWM_LIMIT) Left_Speed = DRIVE_PWM_LIMIT;
    if (Right_Speed > DRIVE_PWM_LIMIT) Right_Speed = DRIVE_PWM_LIMIT;
}
