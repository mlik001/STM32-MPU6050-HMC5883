#include "stm32f10x.h"                  // Device header
#include "MPU6050_reg.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "Delay.h"
#include <math.h>

/*
 * ============================================================================
 * MPU6050 六轴传感器驱动 + 互补滤波姿态解算
 * ----------------------------------------------------------------------------
 * 总线: I2C2, PB6=SCL / PB7=SDA, 与 HMC5883L 共用
 *
 * I2C_Send7bitAddress() 要求传入"7 位地址 << 1"后的值。
 * MPU6050 (AD0=GND) 7 位地址 = 0x68, 因此传 0xD0。
 * ============================================================================
 */
#define MPU6050_ADDRESS		(MPU6050_7BIT_ADDRESS << 1)	/* = 0xD0 */

/* 采样周期, 必须与 Timer.c 中 TIM2 的实际中断周期一致 */
#define MPU6050_DT			0.01f		/* 10ms -> 100Hz */
#define MPU6050_PI			3.14159265f

/* 互补滤波系数: 0.98 偏向陀螺积分(动态响应快), 0.02 用于修正累积漂移 */
#define COMP_FILTER_ALPHA	0.98f

int16_t AX, AY, AZ, GX, GY, GZ, HX, HY, HZ;

/* 陀螺三轴零偏(LSB), 由 MPU6050_CalibrateGyro() 计算 */
static float GyroBiasX = 0.0f;
static float GyroBiasY = 0.0f;
static float GyroBiasZ = 0.0f;
static uint8_t GyroCalibrated = 0;

static float HMC5883_Declination = 0.0f;	/* 磁偏角, 单位度, 按所在地区填写 */

/*
 * I2C 事件等待
 * 返回 1 表示等到目标事件, 0 表示超时
 */
static uint8_t MPU6050_WaitEvent(uint32_t I2C_EVENT)
{
	uint32_t Timeout = 100000;

	while (I2C_CheckEvent(I2C2, I2C_EVENT) != SUCCESS)
	{
		if (--Timeout == 0)
		{
			return 0;
		}
	}
	return 1;
}

/*
 * I2C 总线故障恢复
 * 详见 HMC5883.c 中同名函数的说明。共用总线时两边都可能遇到卡死,
 * 但恢复动作只需要做一次, 这里保持独立实现以便任一驱动单独使用。
 */
static void MPU6050_BusRecover(void)
{
	I2C_GenerateSTOP(I2C2, ENABLE);
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	I2C_Cmd(I2C2, DISABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
	Delay_us(5);

	for (uint8_t i = 0; i < 9; i++)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_6);
		Delay_us(5);
		GPIO_SetBits(GPIOB, GPIO_Pin_6);
		Delay_us(5);
	}

	GPIO_ResetBits(GPIOB, GPIO_Pin_7);
	Delay_us(5);
	GPIO_SetBits(GPIOB, GPIO_Pin_6);
	Delay_us(5);
	GPIO_SetBits(GPIOB, GPIO_Pin_7);
	Delay_us(5);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	I2C_Cmd(I2C2, ENABLE);
}

void MPU6050_WriteReg(uint8_t ReAddress, uint8_t Data)
{
	I2C_GenerateSTART(I2C2, ENABLE);
	if (!MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
	{
		MPU6050_BusRecover();
		return;
	}

	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	if (!MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		MPU6050_BusRecover();
		return;
	}

	I2C_SendData(I2C2, ReAddress);
	if (!MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		MPU6050_BusRecover();
		return;
	}

	I2C_SendData(I2C2, Data);
	if (!MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		MPU6050_BusRecover();
		return;
	}

	I2C_GenerateSTOP(I2C2, ENABLE);
}

uint8_t MPU6050_ReadReg(uint8_t ReAddress, uint8_t *buffer, uint8_t n)
{
	if (n == 0)
	{
		return 0;
	}

	I2C_GenerateSTART(I2C2, ENABLE);
	if (!MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
	{
		MPU6050_BusRecover();
		return 0;
	}

	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	if (!MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		MPU6050_BusRecover();
		return 0;
	}

	I2C_SendData(I2C2, ReAddress);
	if (!MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		MPU6050_BusRecover();
		return 0;
	}

	I2C_GenerateSTART(I2C2, ENABLE);
	if (!MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
	{
		MPU6050_BusRecover();
		return 0;
	}

	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
	if (!MPU6050_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		MPU6050_BusRecover();
		return 0;
	}

	for (uint8_t i = 0; i < n; i++)
	{
		if (i == n - 1)
		{
			/* 最后一个字节: 先关 ACK 再发 STOP, 否则会多读一个字节 */
			I2C_AcknowledgeConfig(I2C2, DISABLE);
			I2C_GenerateSTOP(I2C2, ENABLE);
		}

		if (!MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			MPU6050_BusRecover();
			return 0;
		}
		buffer[i] = I2C_ReceiveData(I2C2);
	}

	I2C_AcknowledgeConfig(I2C2, ENABLE);
	return 1;
}

/*
 * 初始化 MPU6050, 同时完成 I2C2 总线初始化
 *
 * 注意: 本函数是 I2C2 总线的唯一初始化入口。
 *       HMC5883L 与本器件共用 PB6/PB7, 若两处各自调用 I2C_Init(),
 *       后一次调用会覆盖前一次的时钟频率与配置寄存器。
 */
void MPU6050_Init(void)
{
	/* ---- 1. 总线级初始化(整个 I2C2 只做一次) ---- */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	I2C_InitTypeDef I2C_InitStruct;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 400000;		/* 400kHz 快速模式 */
	I2C_Init(I2C2, &I2C_InitStruct);

	I2C_Cmd(I2C2, ENABLE);
	Delay_ms(10);

	/* ---- 2. 器件复位: 清空上电残留状态, 保证下面写入的配置真正生效 ---- */
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
	Delay_ms(100);	/* 复位需要约 100ms, 期间寄存器不可访问 */

	/* ---- 3. 器件配置 ---- */
	/* 唤醒并选择 X 轴陀螺 PLL 作为时钟源(比内部 RC 振荡器稳定得多) */
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_PLL_X);
	/* 六轴全部使能(不进入休眠) */
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	/* 采样率分频: 输出率 = 1kHz / (1 + 9) = 100Hz, 与 TIM2 的 100Hz 对齐 */
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
	/* 数字低通滤波 DLPF_CFG = 3, 带宽 44Hz(陀螺) / 42Hz(加速度), 抑制高频噪声 */
	MPU6050_WriteReg(MPU6050_CONFIG, 0x03);
	/* 陀螺满量程 ±2000dps */
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_2000);
	/* 加速度满量程 ±16g */
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_16);

	Delay_ms(10);
}

uint8_t MPU6050_GetID(void)
{
	uint8_t id = 0;
	MPU6050_ReadReg(MPU6050_WHO_AM_I, &id, 1);
	return id;
}

void MPU6050_GetData(int16_t *Accx, int16_t *Accy, int16_t *Accz,
                     int16_t *Gyrox, int16_t *Gyroy, int16_t *Gyroz)
{
	uint8_t buffer[14];

	/* 一次性读取 14 字节: 加速度(6) + 温度(2) + 陀螺(6) */
	if (!MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H, buffer, 14))
	{
		/* 读取失败时保留上一次的有效值, 避免用 0 污染姿态解算 */
		return;
	}

	/* 显式做有符号 16 位组合。buffer 是 uint8_t, 直接左移会被提升为有符号 int,
	 * 负数的补码组合行为依赖实现, 这里统一转成 uint16_t 再截断, 语义明确。 */
	*Accx = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
	*Accy = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
	*Accz = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

	/* buffer[6]、buffer[7] 为温度, 本工程未使用 */

	*Gyrox = (int16_t)(((uint16_t)buffer[8]  << 8) | buffer[9]);
	*Gyroy = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
	*Gyroz = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);
}

/*
 * 陀螺零偏校准
 * 必须在器件完全静止(水平放置、不振动)时调用。
 * 原理: 静止时三轴角速度真值均为 0, 此时读到的值即为零偏。
 *       不校准的话, 零偏会被积分放大, 角度会持续单方向漂移。
 */
void MPU6050_CalibrateGyro(void)
{
	int32_t sumX = 0, sumY = 0, sumZ = 0;
	int16_t ax, ay, az, gx, gy, gz;

	for (uint16_t i = 0; i < MPU6050_CALI_SAMPLES; i++)
	{
		MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
		sumX += gx;
		sumY += gy;
		sumZ += gz;
		Delay_ms(5);
	}

	GyroBiasX = (float)sumX / (float)MPU6050_CALI_SAMPLES;
	GyroBiasY = (float)sumY / (float)MPU6050_CALI_SAMPLES;
	GyroBiasZ = (float)sumZ / (float)MPU6050_CALI_SAMPLES;

	GyroCalibrated = 1;
}

uint8_t MPU6050_IsCalibrated(void)
{
	return GyroCalibrated;
}

float MPU6050_GetPitch(float *pitch, int AX, int AY, int AZ)
{
	/* 绕 Y 轴旋转为俯仰, 由 X 轴重力分量与 YZ 平面合力求解 */
	*pitch = atan2f(-(float)AX, sqrtf((float)AY * (float)AY + (float)AZ * (float)AZ))
	         * 180.0f / MPU6050_PI;
	return *pitch;
}

float MPU6050_GetRoll(float *roll, int AX, int AY, int AZ)
{
	/* 绕 X 轴旋转为横滚, 由 Y/Z 轴重力分量求解, X 轴分量不参与计算 */
	(void)AX;
	*roll = atan2f((float)AY, (float)AZ) * 180.0f / MPU6050_PI;
	return *roll;
}

/*
 * 姿态更新: 加速度计 + 陀螺仪 + 磁力计互补滤波
 *
 * 修正说明:
 *   1. 陀螺零偏在积分前先扣除, 消除静态漂移
 *   2. pitch 改为由 GX(X 轴角速度)积分, roll 由 GY 积分
 *      —— 原代码用 Y 轴数据积分 pitch、X 轴数据积分 roll, 轴向是错的
 *   3. 系数由 0.976/0.024 调整为 0.98/0.02, 减少磁力计噪声对 yaw 的干扰
 *   4. 角度统一归一化到 [-180, 180]
 */
void UpdataAngal(float *pitch, float *roll, float *yaw)
{
	float acc_pitch, acc_roll, mag_yaw;
	float angle_pitch, angle_roll, angle_yaw;
	float groy_x_rate, groy_y_rate, groy_z_rate;
	float BX, BY;
	float roll_rad, pitch_rad;

	angle_pitch = *pitch;
	angle_roll  = *roll;
	angle_yaw   = *yaw;

	/* 获取传感器原始数据 */
	MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
	HMC5883_GetData(&HX, &HY, &HZ);

	/* 角速度换算为 °/s, 并扣除零偏 */
	groy_x_rate = ((float)GX - GyroBiasX) / MPU6050_GYRO_SENS_2000DPS;
	groy_y_rate = ((float)GY - GyroBiasY) / MPU6050_GYRO_SENS_2000DPS;
	groy_z_rate = ((float)GZ - GyroBiasZ) / MPU6050_GYRO_SENS_2000DPS;

	/* 加速度计解算重力方向倾角 */
	MPU6050_GetPitch(&acc_pitch, AX, AY, AZ);
	MPU6050_GetRoll(&acc_roll, AX, AY, AZ);

	/* ---- 磁力计倾角补偿: 把机体坐标系磁场投影到水平面 ---- */
	roll_rad  = angle_roll * MPU6050_PI / 180.0f;
	pitch_rad = angle_pitch * MPU6050_PI / 180.0f;

	BX = (float)HX * cosf(pitch_rad)
	   + (float)HY * sinf(roll_rad) * sinf(pitch_rad)
	   + (float)HZ * cosf(roll_rad) * sinf(pitch_rad);
	BY = (float)HY * cosf(roll_rad)
	   - (float)HZ * sinf(roll_rad);

	mag_yaw = atan2f(BY, BX) * 180.0f / MPU6050_PI;
	mag_yaw += HMC5883_Declination;	/* 补偿磁偏角, 得到真北朝向 */

	/* ---- 互补滤波: 陀螺积分提供短期精度, 加速度计/磁力计修正长期漂移 ---- */
	angle_pitch = COMP_FILTER_ALPHA * (angle_pitch + MPU6050_DT * groy_y_rate)
	            + (1.0f - COMP_FILTER_ALPHA) * acc_pitch;
	angle_roll  = COMP_FILTER_ALPHA * (angle_roll + MPU6050_DT * groy_x_rate)
	            + (1.0f - COMP_FILTER_ALPHA) * acc_roll;
	angle_yaw   = COMP_FILTER_ALPHA * (angle_yaw + MPU6050_DT * groy_z_rate)
	            + (1.0f - COMP_FILTER_ALPHA) * mag_yaw;

	/* 归一化到 [-180, 180] */
	while (angle_yaw > 180.0f)  angle_yaw -= 360.0f;
	while (angle_yaw < -180.0f) angle_yaw += 360.0f;

	*pitch = angle_pitch;
	*roll  = angle_roll;
	*yaw   = angle_yaw;
}
