#ifndef __DELAY_H__
#define __DELAY_H__

extern volatile unsigned long tick_ms;

int mspm0_delay_ms(unsigned long num_ms);
int mspm0_get_clock_ms(unsigned long *count);
// void SysTick_Init(void);

#endif  /* #ifndef __DELAY_H__ */
