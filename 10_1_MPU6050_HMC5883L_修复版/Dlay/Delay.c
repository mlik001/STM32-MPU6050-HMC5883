#include "stm32f10x.h"
#include "Delay.h"

/*
 * 延时实现说明
 * SysTick 被本模块独占, 每次延时都重新配置 LOAD/CTRL, 结束后关闭计数器。
 * 使用 (uint32_t)SystemCoreClock 而不是硬编码 72000000, 这样即使系统时钟
 * 配置改变, 延时依然准确。
 */

/**
  * @brief  微秒级延时
  * @param  us 延时时长, 范围 0 ~ 233015 (LOAD 为 24 位)
  * @retval 无
  */
void Delay_us(uint32_t us)
{
	uint32_t ticks;

	/* 1us 对应的 SysTick 计数次数 = HCLK / 1e6 */
	ticks = us * (SystemCoreClock / 1000000U);

	/* 24 位重装载寄存器上限保护, 超范围时截断, 避免写入无效值 */
	if (ticks > 0x00FFFFFFU)
	{
		ticks = 0x00FFFFFFU;
	}

	SysTick->LOAD = ticks - 1;				// 设置重装载值(计数从 ticks-1 递减到 0)
	SysTick->VAL  = 0x00;					// 清空当前计数值
	SysTick->CTRL = 0x00000005;				// 时钟源 HCLK(bit2=1), 使能计数(bit0=1), 不使能中断(bit1=0)

	// 等待 COUNTFLAG(bit16) 置位, 表示计数到 0
	while (!(SysTick->CTRL & 0x00010000));

	SysTick->CTRL = 0x00000000;				// 关闭 SysTick
	SysTick->VAL  = 0x00;					// 复位计数器
}

/**
  * @brief  毫秒级延时
  * @param  ms 延时时长
  * @retval 无
  */
void Delay_ms(uint32_t ms)
{
	while (ms--)
	{
		Delay_us(1000);
	}
}

/**
  * @brief  秒级延时
  * @param  s 延时时长
  * @retval 无
  */
void Delay_s(uint32_t s)
{
	while (s--)
	{
		Delay_ms(1000);
	}
}
