#ifndef __MPU6050_REG_H
#define __MPU6050_REG_H


#ifdef __cplusplus
extern "C" {
#endif
#define MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75

/* ---- 位域常量 ---- */
/* PWR_MGMT_1: 器件复位(写 1 触发, 硬件自动清零) */
#define MPU6050_PWR1_DEVICE_RESET	0x80
/* PWR_MGMT_1: 时钟源选择 PLL with X axis gyroscope reference (推荐, 比内部振荡器稳定) */
#define MPU6050_PWR1_CLKSEL_PLL_X	0x01
/* GYRO_CONFIG: 满量程 ±2000 dps, 灵敏度 16.4 LSB/(°/s) */
#define MPU6050_GYRO_FS_2000		0x18
/* ACCEL_CONFIG: 满量程 ±16g, 灵敏度 2048 LSB/g */
#define MPU6050_ACCEL_FS_16			0x18

/* 灵敏度常数, 与上面的量程配置必须一致, 改配置时务必同步修改 */
#define MPU6050_GYRO_SENS_2000DPS	16.4f	/* LSB/(°/s) */
#define MPU6050_ACCEL_SENS_16G		2048.0f	/* LSB/g */

/* 7 位 I2C 地址 (AD0 接地): 0x68 */
#define MPU6050_7BIT_ADDRESS		0x68
#define MPU6050_ID_VALUE			0x68


#ifdef __cplusplus
}
#endif

#endif
