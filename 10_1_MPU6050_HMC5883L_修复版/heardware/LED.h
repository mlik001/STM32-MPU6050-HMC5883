#ifndef __LED_H
#define __LED_H


#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f10x.h"

/*
 * 外部 LED 驱动 (PA1 / PA2)
 * 注意: 板载 PC13 LED 由 main.c 中的 LED_Init() 独立管理,
 *       两者引脚不同, 不要混用。
 */

void LED_Init(void);		/* 修正原拼写错误 LED_Iint -> LED_Init */
void LED1_ON(void);
void LED1_OFF(void);
void LED2_ON(void);
void LED2_OFF(void);
void LED1_Turn(void);		/* 修正原拼写错误 LED1_Trun -> LED1_Turn */
void LED2_Turn(void);		/* 修正原拼写错误 LED2_Trun -> LED2_Turn */


#ifdef __cplusplus
}
#endif

#endif
