
#include "encoder.h"
#include "led.h"
uint32_t gpio_interrup1,gpio_interrup2;
int Get_Encoder_countA,Get_Encoder_countB;
/*******************************************************

***********************************************************/
void GROUP1_IRQHandler(void)
{

    gpio_interrup1 = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN);
    gpio_interrup2 = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
    
    
	//encoderA
	if((gpio_interrup1 & ENCODERA_E1A_PIN)==ENCODERA_E1A_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERA_PORT,ENCODERA_E1B_PIN))
		{
			Get_Encoder_countA--;
		}
		else
		{
			Get_Encoder_countA++;
		}
	}
	else if((gpio_interrup1 & ENCODERA_E1B_PIN)==ENCODERA_E1B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERA_PORT,ENCODERA_E1A_PIN))
		{
			Get_Encoder_countA++;
		}
		else
		{
			Get_Encoder_countA--;
		}
	}
	
	//encoderB
	if((gpio_interrup2 & ENCODERB_E2A_PIN)==ENCODERB_E2A_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERB_PORT,ENCODERB_E2B_PIN))
		{
			Get_Encoder_countB--;
		}
		else
		{
			Get_Encoder_countB++;
		}
	}
	else if((gpio_interrup2 & ENCODERB_E2B_PIN)==ENCODERB_E2B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERB_PORT,ENCODERB_E2A_PIN))
		{
			Get_Encoder_countB++;
		}                 
		else              
		{                 
			Get_Encoder_countB--;
		}
	}
	DL_GPIO_clearInterruptStatus(ENCODERA_PORT,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN);
	DL_GPIO_clearInterruptStatus(ENCODERB_PORT,ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
}


/**
 * @brief 计算电机转速（单位：RPM）
 * @param encoder_count 编码器计数
 * @param sample_time_ms 采样时间（毫秒）
 * @return 电机转速（单位：RPM）
 * @note 根据编码器计数和采样时间计算电机转速，需根据实际电机参数调整常量
 */
float Calculate_Motor_RPM(int encoder_count, int sample_time_ms) 
{
		//更换电机需修改此处参数
    const int ENCODER_LINES = 13;        // 编码器线数 (每转13个脉冲)
    const int MULTIPLY_FACTOR = 2;       // 2倍频系数 (只检测上升沿)
    const int GEAR_RATIO = 28;           // 减速比 28:1     
   
    // 计算每转的脉冲数
    int pulses_per_revolution = ENCODER_LINES * MULTIPLY_FACTOR; 

    // 计算电机转速
    // 公式：RPM = (编码器计数 * 60000) / (每转脉冲数 * 采样时间)
    // 其中60000是将毫秒转换为分钟的系数（60秒 * 1000毫秒）
    float motor_rpm = (float)encoder_count * 60000.0f / (pulses_per_revolution * sample_time_ms);
    
    // 考虑减速比，得到实际输出转速
    return motor_rpm / GEAR_RATIO;
}