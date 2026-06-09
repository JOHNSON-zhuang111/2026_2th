/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 文档网站：wiki.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 嘉立创社区问答：https://www.jlc-bbs.com/lckfb
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 */

#ifndef __BSP_SR04_H__
#define __BSP_SR04_H__

#include "board.h"

#define SR04_TRIG(x)  ( x ? DL_GPIO_setPins(SR04_PORT,SR04_Trig_PIN) : DL_GPIO_clearPins(SR04_PORT,SR04_Trig_PIN) )
#define SR04_ECHO()   ( ( ( DL_GPIO_readPins(SR04_PORT,SR04_Echo_PIN) & SR04_Echo_PIN ) > 0 ) ? 1 : 0 )

void SR04_Init(void);//超声波初始化
float SR04_GetLength(void );//获取超声波测距的距离

#endif
