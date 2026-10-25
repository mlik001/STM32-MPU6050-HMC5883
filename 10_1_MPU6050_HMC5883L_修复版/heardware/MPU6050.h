#ifndef __MPU6050_H
#define __MPU6050_H


#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f10x.h"

/*
 * ============================================================================
 * MPU6050 六轴传感器驱动 (硬件 I2C2) + 姿态解算
 * ----------------------------------------------------------------------------
 * 总线: PB6 -> SCL, PB7 -> SDA
 *       本器件的 I2C 初始化是总线唯一入口, HMC5883L 复用同一条总线。
 * ============================================================================
 */

/* 陀螺零偏校准采样次数 */
#define MPU6050_CALI_SAMPLES	200

void    MPU6050_WriteReg(uint8_t ReAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t ReAddress, uint8_t *buffer, uint8_t n);
void    MPU6050_Init(void);
void    MPU6050_GetData(int16_t *Accx, int16_t *Accy, int16_t *Accz,
                        int16_t *Gyrox, int16_t *Gyroy, int16_t *Gyroz);
uint8_t MPU6050_GetID(void);

/* 陀螺零偏校准: 必须在静止状态下调用 */
void    MPU6050_CalibrateGyro(void);
/* 查询校准是否完成: 1 已完成 */
uint8_t MPU6050_IsCalibrated(void);

void    UpdataAngal(float *pitch, float *roll, float *yaw);
float   MPU6050_GetRoll(float *roll, int AX, int AY, int AZ);
float   MPU6050_GetPitch(float *pitch, int AX, int AY, int AZ);


#ifdef __cplusplus
}
#endif

#endif
