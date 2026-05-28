#include "motor.h"
float Velcity_Kp=2.6f,  Velcity_Ki=0.0f,  Velcity_Kd; //速度PID参数，根据实际情况调整
/*******************************************************
函数功能：外部中断模拟编码器信号
入口函数：无
返回  值：无
***********************************************************/

void Set_PWM_L(int pwmA)
{

 if(pwmA>0)
    {
			DL_GPIO_setPins(BIN_PORT,BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmA),GPIO_PWM_0_C1_IDX);
    }
    else
    {
        DL_GPIO_setPins(BIN_PORT,BIN_BIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN1_PIN);
		 DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmA),GPIO_PWM_0_C1_IDX);
    }

}
void Set_PWM_R(int pwmB)
{
	 if(pwmB>0)
    {
        DL_GPIO_setPins(AIN_PORT,AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT,AIN_AIN2_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmB),GPIO_PWM_0_C0_IDX);
    }
    else
    {
       
        DL_GPIO_setPins(AIN_PORT,AIN_AIN2_PIN);
        DL_GPIO_clearPins(AIN_PORT,AIN_AIN1_PIN);

		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmB),GPIO_PWM_0_C0_IDX);
    }
   
}

/**
 * @brief 速度控制函数（A通道）
 * @param TargetVelocity 目标速度
 * @param CurrentVelocity 当前速度
 * @return 速度控制量
 * @note 采用增量式PI控制算法，用于电机速度调节
 */
int Velocity_A(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  //速度偏差值
	static int ControlVelocityA = 0; //静态变量，用于存储当前控制量
	static int Last_biasA = 0;       //静态变量，用于存储上一次的偏差值
	
	Bias = TargetVelocity - CurrentVelocity; //计算速度偏差
	
	//增量式PI控制算法
	//公式：Δu = Kp*(e(k) - e(k-1)) + Ki*e(k)
	//其中：
	//Velcity_Kp*(Bias - Last_biasA) 是比例项，用于响应速度偏差的变化率
	//Velcity_Ki*Bias 是积分项，用于消除静态误差
	ControlVelocityA += Velcity_Ki * (Bias - Last_biasA) + Velcity_Kp * Bias;
	
	Last_biasA = Bias; //更新上一次的偏差值
	
	//限制控制量的范围，防止输出过大
    if(ControlVelocityA > 7000) 
        ControlVelocityA = 7000;
    else if(ControlVelocityA < -7000) 
        ControlVelocityA = -7000;
	
	return ControlVelocityA; //返回速度控制量
}

/***************************************************************************

***************************************************************************/
int Velocity_B(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;
		static int ControlVelocityB, Last_biasB; 
		Bias=TargetVelocity-CurrentVelocity;
		
		ControlVelocityB+=Velcity_Ki*(Bias-Last_biasB)+Velcity_Kp*Bias;
                                                                   
	    if(ControlVelocityB>7000) ControlVelocityB=7000;
	    else if(ControlVelocityB<-7000) ControlVelocityB=-7000;
		return ControlVelocityB; 

}