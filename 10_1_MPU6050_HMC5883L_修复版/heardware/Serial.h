#ifndef __SERIAL_H
#define __SERIAL_H


#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f10x.h"

/*
 * USART1: PA9 = TX, PA10 = RX, 9600 8N1
 * 当前工程只使用发送功能, 接收引脚保留为浮空输入以便后续扩展。
 */
#define SERIAL_BAUDRATE		9600

void Serial_Init(void);


#ifdef __cplusplus
}
#endif

#endif
