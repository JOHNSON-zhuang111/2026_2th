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
	
	/* 因为调用处有类似 Speed_Out_L += p->Out 的累加逻辑，
	   所以这里的 Out 直接输出本次计算的增量值ΔU */
	p->Out = inc;
	
	/* 增量输出限幅（防止单次变动过猛）*/
	if (p->Out > p->OutMax) { p->Out = p->OutMax; }
	if (p->Out < p->OutMin) { p->Out = p->OutMin; }
}

void PID_Update(PID_t *p)
{
	/*获取本次误差和上次误差*/
	p->Error1 = p->Error0;					//获取上次误差
	p->Error0 = p->Target - p->Actual;		//获取本次误差，目标值减实际值，即为误差值
	
	/*外环误差积分（累加）*/
	/*如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
	/*因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
	/*后续一旦Ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
	if (p->Ki != 0)					//如果Ki不为0
	{
		p->ErrorInt += p->Error0;	//进行误差积分
	}
	else							//否则
	{
		p->ErrorInt = 0;			//误差积分直接归0
	}
	
	/*PID计算*/
	/*使用位置式PID公式，计算得到输出值*/
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
		   + p->Kd * (p->Error0 - p->Error1);
	
	/*输出限幅*/
	 if (p->Out > p->OutMax) {p->Out = p->OutMax;}	//限制输出值最大为结构体指定的OutMax
	 if (p->Out < p->OutMin) {p->Out = p->OutMin;}	//限制输出值最小为结构体指定的OutMin
}




#include "key.h"


/**************************************************************************
Function: Key scan
Input   : Double click the waiting time
Output  : 0：No action；1：click；2：Double click
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击
**************************************************************************/
u8 click_N_Double (u8 time)
{
    static  u8 flag_key,count_key,double_key=0;
    static  u16 count_single,Forever_count;
    if(DL_GPIO_readPins(KEY_PORT,KEY_key_PIN)>0)  Forever_count++;   //长按标志位未置1
    else        Forever_count=0;
    if((DL_GPIO_readPins(KEY_PORT,KEY_key_PIN)>0)&&0==flag_key)     flag_key=1; //第一次按下
    if(0==count_key)
    {
            if(flag_key==1)
            {
                double_key++;
                count_key=1;            //标记按下一次
            }
            if(double_key==3)
            {                                       //按下两次
                double_key=0;
                count_single=0;
                return 2;                   //双击执行的指令
            }
    }
    if(0==DL_GPIO_readPins(KEY_PORT,KEY_key_PIN))          flag_key=0,count_key=0;
    if(1==double_key)
    {
        count_single++;
        if(count_single>time&&Forever_count<time)
        {
            double_key=0;
            count_single=0; //超时不标记为双击
			return 1;//单击执行的指令
        }
        if(Forever_count>time)
        {
            double_key=0;
            count_single=0;
        }
    }
    return 0;
}
/**************************************************************************
Function: Long press detection
Input   : none
Output  : 0：No action；1：Long press for 2 seconds；
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按2s
**************************************************************************/
u8 Long_Press(void)
{
        static u16 Long_Press_count, Long_Press;
        
        // TI的板子按键绝大多数是低电平触发（按下时KEY==0）
        // 如果你按下还是不行，那就是按键电平其实是低电平有效
        if(Long_Press==0 && KEY==0)  
        {
            Long_Press_count++; 
        }  
        else if (KEY>0) // 按键松开时
        {
            Long_Press_count=0;
        }

        // 因为主函数 while() 里 delay_ms(100) 后调用的 Key()
        // 所以 100ms * 20次 = 2秒，这里改为 20 就可以 2秒长按触发
        if(Long_Press_count>20)        
        {
            Long_Press=1;
            Long_Press_count=0;
            return 1;
        }
        
        // 当按键松开时，复位长按标志
        if(Long_Press==1 && KEY>0)     
        {
            Long_Press=0;
        }
        return 0;
}


extern u8 car_started;

void Key(void)
{
	u8 tmp;
	tmp = click_N_Double (5);
	if(tmp == 1||tmp == 2) // 单击或双击都启动小车
	{
		car_started = 1; // 启动小车
	}
}


#ifndef _LED_H
#define _LED_H
#include "ti_msp_dl_config.h"


void LED_ON(void);
void LED_OFF(void);
void LED_Toggle(void);
void LED_Flash(uint16_t time);
#endif 
