#include "xunji.h"

unsigned char digital(unsigned char channel) // 1-8 对应8路传感器
{
    u8 value = 0;
    switch(channel) 
    {
        case 1:  // 第1路：PB16
            if(DL_GPIO_readPins(XUNJI_PORT,XUNJI_XUNJI0_PIN) == 0) value = 1;//检测到黑线为低电平
            else value = 0;  
            break;  
        case 2:  // 第2路：PB0
            if(DL_GPIO_readPins(XUNJI_PORT,XUNJI_XUNJI1_PIN) == 0) value = 1;
            else value = 0;  
            break;  
        case 3:  // 第3路：PB6
            if(DL_GPIO_readPins(XUNJI_PORT,XUNJI_XUNJI2_PIN) == 0) value = 1;
            else value = 0;  
            break;   
        case 4:  // 第4路：PB7
            if(DL_GPIO_readPins(XUNJI_PORT,XUNJI_XUNJI3_PIN) == 0) value = 1;
            else value = 0;  
            break;   
        case 5:  // 第5路：PB8
            if(DL_GPIO_readPins(XUNJI_PORT,XUNJI_XUNJI4_PIN) == 0) value = 1;
            else value = 0;  
            break;
        case 6:  // 第6路：PB15
            if(DL_GPIO_readPins(XUNJI_PORT,XUNJI_XUNJI5_PIN) == 0) value = 1;
            else value = 0;  
            break;
        case 7:  // 第7路：PB17
            if(DL_GPIO_readPins(XUNJI_PORT,XUNJI_XUNJI6_PIN) == 0) value = 1;
            else value = 0;  
            break;
        case 8:  // 第8路：PB12
            if(DL_GPIO_readPins(XUNJI_PORT,XUNJI_XUNJI7_PIN) == 0) value = 1;
            else value = 0;  
            break;
        default:  // 无效通道返回0
            value = 0;
            break;
    }
    return value; 
}

void xunji_print_values(void)
{
    printf("XJ: %u %u %u %u %u %u %u %u\r\n",
           digital(1),
           digital(2),
           digital(3),
           digital(4),
           digital(5),
           digital(6),
           digital(7),
           digital(8));
}