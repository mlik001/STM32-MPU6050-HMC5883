#ifndef __KEY_H
#define __KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

void Key_Init(void);
uint8_t Key_GetNum(void);

#ifdef __cplusplus
}
#endif

#endif
