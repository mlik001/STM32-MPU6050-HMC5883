#include "stm32f10x.h"
#include "HMC5883_reg.h"
#include "HMC5883.h"
#include "Delay.h"

/*
 * ============================================================================
 * HMC5883L 三轴磁力计驱动 (硬件 I2C2)
 * ----------------------------------------------------------------------------
 * 总线: 与 MPU6050 共用同一条 I2C2 总线
 *       PB6 -> SCL
 *       PB7 -> SDA
 *       两颗器件均为开漏输出, 需外接 4.7k 上拉电阻到 3.3V (GY-271 模块自带)
 *
 * 7 位地址: 0x1E  (I2C_Send7bitAddress 需要传左移 1 位后的值, 由库函数内部处理)
 * ============================================================================
 */

/* I2C_Send7bitAddress() 要求的地址格式: 7 位地址 << 1 */
#define HMC5883_ADDRESS		(HMC5883_7BIT_ADDRESS << 1)	/* = 0x3C */

/*
 * I2C 事件等待
 * 返回 1 表示在超时前等到了目标事件, 返回 0 表示超时。
 * 调用者必须检查返回值, 否则总线异常时会把错误数据当有效数据处理。
 */
static uint8_t HMC5883_WaitEvent(uint32_t I2C_EVENT)
{
	uint32_t Timeout = 100000;

	while (I2C_CheckEvent(I2C2, I2C_EVENT) != SUCCESS)
	{
		if (--Timeout == 0)
		{
			return 0;	/* 超时 */
		}
	}
	return 1;
}

/*
 * I2C 总线故障恢复
 * 当从机在传输中途掉电/复位时可能把 SDA 拉低不放, 导致主机永远等不到 EV 事件。
 * 处理办法: 释放总线 + 软件产生 9 个 SCL 时钟, 把从机的移位寄存器打空。
 */
static void HMC5883_BusRecover(void)
{
	I2C_GenerateSTOP(I2C2, ENABLE);
	I2C_AcknowledgeConfig(I2C2, ENABLE);

	/* 关闭 I2C 外设, 把引脚当作普通 GPIO 手动敲时钟 */
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

	/* 产生一个 STOP 条件: SDA 在 SCL 为高时由低变高 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_7);
	Delay_us(5);
	GPIO_SetBits(GPIOB, GPIO_Pin_6);
	Delay_us(5);
	GPIO_SetBits(GPIOB, GPIO_Pin_7);
	Delay_us(5);

	/* 恢复引脚复用功能并重新使能 I2C */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	I2C_Cmd(I2C2, ENABLE);
}

/*
 * 向指定寄存器写入 1 字节
 * 返回值: 1 成功, 0 失败
 */
uint8_t HMC5883_WriteReg(uint8_t regAddr, uint8_t data)
{
	I2C_GenerateSTART(I2C2, ENABLE);
	if (!HMC5883_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
	{
		HMC5883_BusRecover();
		return 0;
	}

	I2C_Send7bitAddress(I2C2, HMC5883_ADDRESS, I2C_Direction_Transmitter);
	if (!HMC5883_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		/* 此事件失败最常见的原因是器件不应答, 即地址错误或器件未接好 */
		HMC5883_BusRecover();
		return 0;
	}

	I2C_SendData(I2C2, regAddr);
	if (!HMC5883_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		HMC5883_BusRecover();
		return 0;
	}

	I2C_SendData(I2C2, data);
	if (!HMC5883_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		HMC5883_BusRecover();
		return 0;
	}

	I2C_GenerateSTOP(I2C2, ENABLE);
	return 1;
}

/*
 * 从指定寄存器连续读取 n 字节
 * 返回值: 1 成功, 0 失败
 */
uint8_t HMC5883_ReadReg(uint8_t regAddr, uint8_t *buffer, uint8_t n)
{
	if (n == 0)
	{
		return 0;
	}

	/* ---- 阶段一: 写寄存器地址 ---- */
	I2C_GenerateSTART(I2C2, ENABLE);
	if (!HMC5883_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
	{
		HMC5883_BusRecover();
		return 0;
	}

	I2C_Send7bitAddress(I2C2, HMC5883_ADDRESS, I2C_Direction_Transmitter);
	if (!HMC5883_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		HMC5883_BusRecover();
		return 0;
	}

	I2C_SendData(I2C2, regAddr);
	if (!HMC5883_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		HMC5883_BusRecover();
		return 0;
	}

	/* ---- 阶段二: 重复起始条件, 切换为读 ---- */
	I2C_GenerateSTART(I2C2, ENABLE);
	if (!HMC5883_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
	{
		HMC5883_BusRecover();
		return 0;
	}

	I2C_Send7bitAddress(I2C2, HMC5883_ADDRESS, I2C_Direction_Receiver);
	if (!HMC5883_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		HMC5883_BusRecover();
		return 0;
	}

	/* ---- 阶段三: 连续读取 ---- */
	for (uint8_t i = 0; i < n; i++)
	{
		if (i == n - 1)
		{
			/* 最后一个字节: 先关 ACK 再产生 STOP, 否则会多读一个字节 */
			I2C_AcknowledgeConfig(I2C2, DISABLE);
			I2C_GenerateSTOP(I2C2, ENABLE);
		}

		if (!HMC5883_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			HMC5883_BusRecover();
			return 0;
		}
		buffer[i] = I2C_ReceiveData(I2C2);
	}

	/* 恢复 ACK, 供后续传输使用 */
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	return 1;
}

/*
 * 初始化 HMC5883L
 *
 * 重要: 不再在此处调用 I2C_Init() 和配置 GPIO!
 *       因为 MPU6050 与本器件共用 I2C2 总线, 两处分别 I2C_Init 会互相覆盖配置
 *       (尤其 I2C_ClockSpeed 和引脚复用), 导致后初始化的那个把前一个的配置改掉。
 *       现在统一由 MPU6050_Init() 完成一次总线初始化, 本函数只做器件级配置。
 */
void HMC5883_Init(void)
{
	/* 等待器件上电稳定, HMC5883L 上电到可通信约需 10ms */
	Delay_ms(10);

	/* 配置 A: 8 次平均 + 75Hz 输出速率 + 正常测量 */
	HMC5883_WriteReg(HMC5883_CRA, HMC5883_CRA_CONFIG);
	/* 配置 B: 增益 1090 LSB/Gauss (±1.3Ga 量程) */
	HMC5883_WriteReg(HMC5883_CRB, HMC5883_CRB_GAIN_1090);
	/* 模式: 连续测量模式 —— 不写这一句芯片会停在单次测量模式, 数据永远不更新 */
	HMC5883_WriteReg(HMC5883_MODE_REG, HMC5883_MODE_CONTINUOUS);

	/* 等待第一次转换完成: 75Hz 下一个周期约 13.3ms */
	Delay_ms(20);
}

/*
 * 读取器件 ID, 返回识别寄存器 A 的值
 * HMC5883L 正常应返回 0x48 ('H')
 */
uint8_t HMC5883_GetID(void)
{
	uint8_t id = 0;
	HMC5883_ReadReg(HMC5883_IRA, &id, 1);
	return id;
}

/*
 * 读取三轴磁场原始数据
 * 数据排布: 0x03=X_H, 0x04=X_L, 0x05=Z_H, 0x06=Z_L, 0x07=Y_H, 0x08=Y_L
 * 注意 Z 轴排在 Y 轴前面, 这是 HMC5883L 特有的寄存器顺序, 不能按 XYZ 顺序解析。
 */
void HMC5883_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ)
{
	uint8_t buffer[6];

	if (!HMC5883_ReadReg(HMC5883_DO_X_H, buffer, 6))
	{
		/* 读取失败时保持上一次的值不变, 避免把 0 当作有效数据喂给姿态解算 */
		return;
	}

	/* 高位在前, 显式转成有符号 16 位 */
	*MagX = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
	*MagZ = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
	*MagY = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
}
