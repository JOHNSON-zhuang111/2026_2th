#include "ti_msp_dl_config.h"
#include "board.h"

volatile unsigned long tick_ms;
volatile uint32_t start_time;

#define PRINTF_ROUTE_UART0 0
#define PRINTF_ROUTE_UART1 1
#define PRINTF_ROUTE_BOTH  2

#ifndef PRINTF_ROUTE_MODE
#define PRINTF_ROUTE_MODE PRINTF_ROUTE_BOTH
#endif

extern u8 car_started;
extern u8 task_mode;

static void debug_uart_send_byte(uint8_t ch)
{
#if (PRINTF_ROUTE_MODE == PRINTF_ROUTE_UART0)
	while (DL_UART_isBusy(UART_0_INST) == true) {
	}
	DL_UART_Main_transmitData(UART_0_INST, ch);
#elif (PRINTF_ROUTE_MODE == PRINTF_ROUTE_UART1)
	while (DL_UART_isBusy(UART_1_INST) == true) {
	}
	DL_UART_Main_transmitData(UART_1_INST, ch);
#else
	while (DL_UART_isBusy(UART_0_INST) == true) {
	}
	DL_UART_Main_transmitData(UART_0_INST, ch);
	while (DL_UART_isBusy(UART_1_INST) == true) {
	}
	DL_UART_Main_transmitData(UART_1_INST, ch);
#endif
}

void SetRunInterrupts(u8 enable)
{
	if (enable)
	{
		NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
		NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
		NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);

		NVIC_EnableIRQ(ENCODERA_INT_IRQN);
		NVIC_EnableIRQ(ENCODERB_INT_IRQN);
		NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	}
	else
	{
		NVIC_DisableIRQ(ENCODERA_INT_IRQN);
		NVIC_DisableIRQ(ENCODERB_INT_IRQN);
		NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);

		NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
		NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
		NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	}
}

void UI_ShowTaskSelect(void)
{
	static u8 last_task_mode = 0xFF;
	static u8 last_car_started = 0xFF;
	u8 mode = task_mode;

	if (mode < 1U || mode > 6U)
	{
		mode = 1U;
	}

	// 内容未变化则不重绘，避免闪屏
	if ((last_task_mode == mode) && (last_car_started == car_started))
	{
		return;
	}

	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t *)"TASK SELECT", 8);
	OLED_ShowString(0, 2, (uint8_t *)"MODE:", 8);
	OLED_ShowNum(36, 2, mode, 1, 8);

	OLED_ShowString(0, 4, (uint8_t *)"A:NEXT", 8);
	if (car_started == 0U)
	{
		OLED_ShowString(0, 5, (uint8_t *)"B:START", 8);
		OLED_ShowString(0, 7, (uint8_t *)"STATE:READY", 8);
	}
	else
	{
		OLED_ShowString(0, 5, (uint8_t *)"B:STOP", 8);
		OLED_ShowString(0, 7, (uint8_t *)"STATE:RUN", 8);
	}

	last_task_mode = mode;
	last_car_started = car_started;
}


void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}


//返回SysTick计数值
uint32_t Systick_getTick(void)
{
	return (SysTick->VAL);
}


//ms阻塞延迟
void delay_ms(uint32_t ms)
{
	//超出能满足的最大延迟
	//if( ms > SysTickMAX_COUNT/(SysTickFre/1000) ) ms = SysTickMAX_COUNT/(SysTickFre/1000);
	for(int i=0;i<1000;i++)
	{
		delay_us(ms);
	}
}


void delay_us(uint32_t us)
{
	if( us > SysTickMAX_COUNT/(SysTickFre/1000000) ) us = SysTickMAX_COUNT/(SysTickFre/1000000);
	
	us = us*(SysTickFre/1000000); //单位转换
	
	//用于保存已走过的时间
	uint32_t runningtime = 0;
	
	//获得当前时刻的计数值
	uint32_t InserTick = Systick_getTick();
	
	//用于刷新实时时间
	uint32_t tick = 0;
	
	uint8_t countflag = 0;
	//等待延迟
	while(1)
	{
		tick = Systick_getTick();//刷新当前时刻计数值
		
		if( tick > InserTick ) countflag = 1;//出现溢出轮询,则切换走时的计算方式
		
		if( countflag ) runningtime = InserTick + SysTickMAX_COUNT - tick;
		else runningtime = InserTick - tick;
		
		if( runningtime>=us ) break;
	}

}
void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

//重定向fputc函数
int fputc(int ch, FILE *stream)
{
	(void)stream;
	debug_uart_send_byte((uint8_t)ch);
    return ch;
}

//重定向fputs函数
int fputs(const char* restrict s, FILE* restrict stream) {

	(void)stream;
    uint16_t char_len=0;
    while(*s!=0)
    {
		debug_uart_send_byte((uint8_t)*s++);
        char_len++;
    }
    return char_len;
}
int puts(const char* _ptr)
{
    return 0;
}
