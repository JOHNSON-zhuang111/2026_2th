#include "board.h"

static volatile u8 control_tick_pending = 0U;

//u8 set_quanshu=1;//设置圈数
u8 car_started = 0U; // 小车启动标志位（中断与主循环共享）
u8 task_mode = 0U;   // 题目的档位，1-4档
Gyro_Struct *JY61P_Data ; // 全局陀螺仪数据指针，供中断和主循环共用

int main(void)
{
	//初始化
	uint8_t xj_line_1[20];
    SYSCFG_DL_init();
	jy61pInit();
	OLED_Init();
	OLED_Clear();
	
	DL_Timer_startCounter(PWM_0_INST);
	NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
	NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	DL_UART_Main_disableLoopbackMode(UART_0_INST);
	DL_UART_enableInterrupt(UART_0_INST, DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	DL_UART_Main_disableLoopbackMode(UART_1_INST);
	DL_UART_enableInterrupt(UART_1_INST, DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

	// 上电默认未发车：关闭业务中断，优先保证按键选题和OLED显示
	SetRunInterrupts(0U);
	
	float YAW;
    while (1) 
    {
		static u8 last_car_started = 0U;
		
		//delay_ms(100);
		Key();

		if (car_started != last_car_started)
		{
			if ((car_started == 1U) && (last_car_started == 0U))
			{
				control_reset_runtime_state();
			}
			else if ((car_started == 0U) && (last_car_started == 1U))
			{
				// 停止边沿先立刻清零PWM，防止关中断后保持上一次占空比
				control_tick_pending = 0U;
				Set_PWM_L(0);
				Set_PWM_R(0);
				control_reset_runtime_state();
			}

			SetRunInterrupts(car_started ? 1U : 0U);
			last_car_started = car_started;
		}

		UI_ShowTaskSelect();

		if (control_tick_pending)
		{
			__disable_irq();
			control_tick_pending = 0U;
			__enable_irq();

			if (car_started == 1U)
			{
				control();
			}
			else
			{
				Set_PWM_L(0);
				Set_PWM_R(0);
			}
		}

    }
}


void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
			control_tick_pending = 1U;
    	}
	}
}

#if 0
static void TIMER_0_INST_IRQHandler_old(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
			//  LED_Flash(100);//测试是否可以进入中断
			//control();
			control_tick_pending = 1U;
			return;
			if ( car_started == 1 ) // 只有按键按下后才启动循迹控制
			{
				control();
			}
			else {
				Set_PWM_L(0); // 没有启动前保持电机不转
				Set_PWM_R(0);
			}
    	}
	}
}

#endif

void UART0_IRQHandler(void)
{
    // 获取当前中断状态
		const uint32_t uartMask = (DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
		uint32_t intStatus = DL_UART_getEnabledInterruptStatus(UART0, uartMask);
		if ((intStatus & uartMask) != 0)
    {
		while (DL_UART_isRXFIFOEmpty(UART0) == false) {
			uint8_t data = DL_UART_receiveData(UART0);
			vofa_set_data(data);
		}
    }
		DL_UART_clearInterruptStatus(UART0, uartMask);
}
void UART1_IRQHandler(void)
{
    // 获取当前中断状态
		const uint32_t uartMask = (DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
		uint32_t intStatus = DL_UART_getEnabledInterruptStatus(UART1, uartMask);
		if ((intStatus & uartMask) != 0)
    {
		while (DL_UART_isRXFIFOEmpty(UART1) == false) {
			uint8_t data = DL_UART_receiveData(UART1);
 			vofa_set_data(data);
		}
    }
		DL_UART_clearInterruptStatus(UART1, uartMask);
}
