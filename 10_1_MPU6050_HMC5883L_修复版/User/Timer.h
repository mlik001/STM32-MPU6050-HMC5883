#ifndef __TIMER_H
#define __TIMER_H


#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f10x.h"

/*
 * TIM2 定时中断, 周期 10ms (100Hz)
 * 72MHz / 7200 = 10kHz 计数频率, 计数到 100 溢出 -> 10ms
 */
#define TIMER_PERIOD_MS		10
#define TIMER_TICK_HZ		(1000 / TIMER_PERIOD_MS)

void Timer_Init(void);
void NVIC_Init2(void);


#ifdef __cplusplus
}
#endif

#endif
