/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 文档网站：wiki.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 嘉立创社区问答：https://www.jlc-bbs.com/lckfb
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 */

#include "bsp_sr04.h"
#include "board.h"

volatile uint32_t msHcCount = 0; // ms计数

float distance = 0;
uint8_t SR04_Flag = 0; // 完成测量标志


/******************************************************************
 * 函 数 名 称：SR04_Init
 * 函 数 说 明：超声波初始化
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：TRIG引脚负责发送超声波脉冲串
******************************************************************/
void SR04_Init(void)
{
    // 清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    // 使能定时器中断
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}
/******************************************************************
 * 函 数 名 称：Open_Timer
 * 函 数 说 明：打开定时器
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：
******************************************************************/
void Open_Timer(void)
{

    DL_TimerG_setTimerCount(TIMER_0_INST, 0);   // 清除定时器计数

    msHcCount = 0;

    DL_TimerG_startCounter(TIMER_0_INST);   // 使能定时器
}

/******************************************************************
 * 函 数 名 称：Get_TIMER_Count
 * 函 数 说 明：获取定时器定时时间
 * 函 数 形 参：无
 * 函 数 返 回：数据
 * 作       者：LCKFB
 * 备       注：
******************************************************************/
uint32_t Get_TIMER_Count(void)
{
    uint32_t time  = 0;
    time   = msHcCount * 1000;                       // 得到us
    time  += DL_TimerG_getTimerCount(TIMER_0_INST);  // 得到ms

    DL_TimerG_setTimerCount(TIMER_0_INST, 0);   // 清除定时器计数
    delay_ms(1);
    return time ;
}

/******************************************************************
 * 函 数 名 称：Close_Timer
 * 函 数 说 明：关闭定时器
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：
******************************************************************/
void Close_Timer(void)
{
    DL_TimerG_stopCounter(TIMER_0_INST);     // 关闭定时器
}

/******************************************************************
 * 函 数 名 称：TIMER_0_INST_IRQHandler
 * 函 数 说 明：定时器中断服务函数
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：1ms进入一次
******************************************************************/
void TIMER_0_INST_IRQHandler(void)
{
    //如果产生了定时器中断
    switch( DL_TimerG_getPendingInterrupt(TIMER_0_INST) )
    {
        case DL_TIMERA_IIDX_LOAD:
                msHcCount++;
            break;

        default://其他的定时器中断
            break;
    }
}

void GROUP1_IRQHandler(void)//Group1的中断服务函数
{
    //读取Group1的中断寄存器并清除中断标志位
    switch( DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) )
    {
        //检查GPIO端口中断，注意是INT_IIDX
        case SR04_INT_IIDX:
            if( SR04_ECHO() ) // 上升沿
            {
                SR04_Flag = 0;
                distance = 0.0;
                Open_Timer();   //打开定时器
            }
            else // 下降沿
            {
                NVIC_DisableIRQ(SR04_INT_IRQN); // 关闭按键引脚的GPIO端口中断

                Close_Timer();   // 关闭定时器
                SR04_Flag = 1;
                distance = (float)Get_TIMER_Count() / 58.0f;   // 获取时间,分辨率为1us
            }
        break;
    }
}


/******************************************************************
 * 函 数 名 称：SR04_GetLength
 * 函 数 说 明：获取测量距离
 * 函 数 形 参：无
 * 函 数 返 回：测量距离
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
float SR04_GetLength(void)
{
    /* 测5次数据，去掉最高值和最低值后计算平均值 */
    float distances[5] = {0}; // 用于存储测量结果
    uint32_t TimeOut = 1000;
    uint8_t valid_count = 0;

    for (uint8_t i = 0; i < 5; i++)
    {
        msHcCount = 0; // ms计数清零
        SR04_Flag = 0; // 完成测量标志清零
        TimeOut = 1000; // 超时时间

        // 开启按键引脚的GPIO端口中断
        NVIC_EnableIRQ(SR04_INT_IRQN);
        delay_ms(10);

        // 触发测量
        SR04_TRIG(0); // trig拉低信号
        delay_1us(10); // 持续时间超过5us
        SR04_TRIG(1); // trig拉高信号
        delay_1us(15); // 持续时间超过10us
        SR04_TRIG(0); // trig拉低信号

        // 等待测量完成或超时
        while (SR04_Flag == 0 && TimeOut)
        {
            TimeOut--;
        }

        if (TimeOut == 0) // 超时处理
        {
            LOG_D("SR04 Time Out!");
            continue; // 跳过本次测量
        }

        distances[valid_count++] = distance; // 保存有效测量结果
    }

    NVIC_DisableIRQ(SR04_INT_IRQN); // 关闭按键引脚的GPIO端口中断

    // 检查有效测量次数
    if (valid_count < 3) // 少于3次有效数据，无法计算去掉最高最低值的平均值
    {
        LOG_D("Not enough valid measurements!");
        return 0;
    }

    // 排序以便去除最高值和最低值
    for (uint8_t i = 0; i < valid_count - 1; i++)
    {
        for (uint8_t j = i + 1; j < valid_count; j++)
        {
            if (distances[i] > distances[j])
            {
                float temp = distances[i];
                distances[i] = distances[j];
                distances[j] = temp;
            }
        }
    }

    // 计算去掉最高值和最低值后的平均值
    float sum = 0;
    for (uint8_t i = 1; i < valid_count - 1; i++)
    {
        sum += distances[i];
    }

    return sum / (valid_count - 2); // 返回中间值的平均值
}


