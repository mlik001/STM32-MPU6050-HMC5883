#ifndef __HMC5883_H
#define __HMC5883_H


#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f10x.h"

/* 磁力计数据（原始 LSB） */
typedef struct
{
	int16_t MagX;
	int16_t MagY;
	int16_t MagZ;
} HMC5883_Data_t;

void    HMC5883_Init(void);
uint8_t HMC5883_GetID(void);
void    HMC5883_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ);


#ifdef __cplusplus
}
#endif

#endif
