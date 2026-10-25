#include "stm32f10x.h"
#include "Timer.h"

/*
 * TIM2 初始化: 产生 10ms 周期更新中断
 * PCLK1 = 36MHz, TIM2 时钟为 APB1 定时器时钟 = 72MHz
 * 分频: 7200 -> 10kHz 计数频率
 * 重载: 100  -> 10000Hz / 100 = 100Hz -> 10ms
 */
void Timer_Init(void)
{
    // 使能 TIM2 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 配置内部时钟源
    TIM_InternalClockConfig(TIM2);

    // 初始化时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 100 - 1;      // 自动重装载值 (ARR)
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;  // 预分频值 (PSC)
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 清除上电时可能已置位的更新标志, 避免使能中断后立即误触发一次
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    // 使能更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 使能自动重装载预装载
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    // 启动定时器
    TIM_Cmd(TIM2, ENABLE);
}

/*
 * 中断向量配置
 * 必须在 Timer_Init() 之前调用, 否则可能出现"中断已产生但 NVIC 未使能"的窗口,
 * 配合主循环的 __WFI() 会导致 CPU 永久休眠。
 */
void NVIC_Init2(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    // 设置中断优先级分组: 2 位抢占优先级 + 2 位响应优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // 配置 TIM2 中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}
