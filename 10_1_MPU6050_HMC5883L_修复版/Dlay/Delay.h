#ifndef __DELAY_H
#define __DELAY_H


#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f10x.h"

/*
 * 基于 SysTick 的阻塞式延时
 * 注意: SysTick 被本模块独占用于延时, 不要再把 SysTick 用作其他用途的计时基准,
 *       否则两边会互相重置 CTRL/LOAD 寄存器导致时间不准。
 *       stm32f10x_it.c 中的 SysTick_Handler 保持为空即可。
 */

/* 微秒级延时, 参数范围 0 ~ 233015 (受 LOAD 寄存器 24 位限制) */
void Delay_us(uint32_t us);
/* 毫秒级延时 */
void Delay_ms(uint32_t ms);
/* 秒级延时 */
void Delay_s(uint32_t s);


#ifdef __cplusplus
}
#endif

#endif
