#include "stm32f10x.h"
#include "Serial.h"

/*
 * USART1 初始化
 * 引脚: PA9 = TX (复用推挽输出), PA10 = RX (浮空输入)
 * 参数: 9600 8N1, 无硬件流控
 * 说明: USART_ClearFlag 不能用于 TXE 标志(手册明确禁止对该标志调用清标志函数,
 *       该标志只能通过写数据寄存器自动清除), 因此这里只清除真正需要清除的标志位。
 */
void Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // 1. 使能 USART1 和 GPIOA 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    // 2. 配置 USART1 引脚
    // PA9: TX - 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA10: RX - 浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置 USART1 参数
    USART_InitStructure.USART_BaudRate = SERIAL_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;  // 当前只使用发送

    // 4. 初始化 USART1
    USART_Init(USART1, &USART_InitStructure);

    // 5. 使能 USART1
    USART_Cmd(USART1, ENABLE);

    // 6. 清除接收相关标志(未使用接收中断, 仅做上电清理)
    USART_ClearFlag(USART1, USART_FLAG_TC | USART_FLAG_RXNE);
}
