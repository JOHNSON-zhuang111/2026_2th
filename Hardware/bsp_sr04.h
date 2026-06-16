#ifndef __BSP_SR04_H__
#define __BSP_SR04_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifndef SR04_INT_IRQN
#define SR04_INT_IRQN GPIO_MULTIPLE_GPIOA_INT_IRQN
#endif

#define SR04_TRIG(x)  ((x) ? DL_GPIO_setPins(SR04_PORT, SR04_Trig_PIN) : DL_GPIO_clearPins(SR04_PORT, SR04_Trig_PIN))
#define SR04_ECHO()   (((DL_GPIO_readPins(SR04_PORT, SR04_Echo_PIN) & SR04_Echo_PIN) > 0U) ? 1U : 0U)

extern volatile float distance;
extern volatile uint8_t SR04_Flag;

void SR04_Init(void);
void Open_Timer(void);
uint32_t Get_TIMER_Count(void);
void Close_Timer(void);
void SR04_HandleEchoInterrupt(void);
float SR04_GetLength(void);
float SR04_GetLengthNonBlocking(void);

#endif
