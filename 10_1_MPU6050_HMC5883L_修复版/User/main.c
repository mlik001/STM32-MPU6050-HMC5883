#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "HMC5883_reg.h"
#include "math.h"
#include "Timer.h"
#include "Serial.h"
#include <stdio.h>

/* ============================================================================
 * STM32F103C8T6 + MPU6050 + HMC5883L 姿态测量固件
 * ----------------------------------------------------------------------------
 * 数据流: TIM2 每 10ms 中断一次置标志 -> 主循环做姿态解算 -> 每 50ms 经 USART1 发送
 * 输出协议: $Roll,Pitch,Yaw*CS\r\n   (CS 为 $ 与 * 之间所有字符的异或校验)
 * ============================================================================
 */

/* 姿态角(单位: 度) */
float Roll, Pitch, Yaw;

/* 与中断共享的变量, 必须加 volatile 防止编译器优化掉读写 */
volatile uint32_t system_tick = 0;
volatile uint32_t packet_counter = 0;
volatile uint8_t  usart_send_flag = 0;
volatile uint8_t  imu_update_flag = 0;

/* 发送缓冲区: 主循环与中断中都不会并发访问, 无需加锁 */
static char usart_buffer[64];

static void USART1_SendString(const char *str);
static void USART1_SendData(void);
static void SystemClock_Config(void);
static void LED_Init(void);
static void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Value);

int main(void)
{
    uint8_t mag_id;

    /* 1. 配置系统时钟 - 必须最先执行, 否则后续的延时和外设时钟全部不准 */
    SystemClock_Config();

    /* 2. 初始化外设
     *    顺序说明: MPU6050_Init() 内部完成 I2C2 总线初始化,
     *              HMC5883L 复用同一条总线, 故必须放在 MPU6050_Init() 之后,
     *              且 HMC5883_Init() 内部不再重复初始化总线。 */
    LED_Init();
    OLED_Init();
    MPU6050_Init();          /* 包含 I2C2 总线初始化 */
    HMC5883_Init();          /* 只做器件级配置, 复用上面的总线 */

    /* 中断配置必须在定时器启动之前完成, 否则可能出现中断已触发但未使能的窗口 */
    NVIC_Init2();
    Timer_Init();

    Serial_Init();

    /* 3. 发送启动信息 */
    USART1_SendString("\r\n\r\n");
    USART1_SendString("========================================\r\n");
    USART1_SendString("STM32F103C8T6 IMU System Started!\r\n");
    USART1_SendString("========================================\r\n");
    USART1_SendString("System Clock: 72MHz\r\n");
    USART1_SendString("UART1: 9600 8N1\r\n");
    USART1_SendString("Data Format: $Roll,Pitch,Yaw*CS\\r\\n\r\n");

    /* 4. 校验传感器通信是否正常 */
    sprintf(usart_buffer, "MPU6050 ID: 0x%02X (expect 0x68)\r\n", MPU6050_GetID());
    USART1_SendString(usart_buffer);

    mag_id = HMC5883_GetID();
    sprintf(usart_buffer, "HMC5883L ID: 0x%02X (expect 0x48)\r\n", mag_id);
    USART1_SendString(usart_buffer);

    if (mag_id != HMC5883_ID_A)
    {
        USART1_SendString("WARNING: Magnetometer not responding!\r\n");
        USART1_SendString("Check wiring: SCL=PB6, SDA=PB7, VCC=3.3V, addr=0x1E\r\n");
    }

    /* 5. 陀螺零偏校准 —— 此处必须保持静止! */
    USART1_SendString("Calibrating gyro, keep device still...\r\n");
    MPU6050_CalibrateGyro();
    USART1_SendString("Calibration done.\r\n");
    USART1_SendString("========================================\r\n\r\n");

    /* 6. 初始化 OLED 显示 */
    OLED_ShowString(1, 1, "P:");
    OLED_ShowString(2, 1, "R:");
    OLED_ShowString(3, 1, "Y:");
    OLED_ShowString(4, 1, "PK:");

    /* 7. 主循环 */
    while (1)
    {
        /* 处理 IMU 数据更新 */
        if (imu_update_flag)
        {
            imu_update_flag = 0;

            UpdataAngal(&Pitch, &Roll, &Yaw);

            OLED_ShowFloat(1, 3, Pitch);
            OLED_ShowFloat(2, 3, Roll);
            OLED_ShowFloat(3, 3, Yaw);
        }

        /* 处理串口数据发送 */
        if (usart_send_flag)
        {
            usart_send_flag = 0;
            USART1_SendData();

            /* 更新 OLED 包计数, 显示为 4 位宽 */
            OLED_ShowNum(4, 4, packet_counter % 10000, 4);
        }

        /* 低功耗休眠。
         * 这里用"关中断检查标志 -> 开中断 -> 休眠"的顺序,
         * 保证在检查标志与进入休眠之间不会丢失中断事件:
         *   若在关中断期间中断已到来, 标志位已置 1, 此时关中断可避免被中断服务
         *   程序抢在后面把标志清零, 从而产生"标志已消费却仍进入休眠"的死锁。
         *   若中断在关中断之后、休眠指令之前到来, WFI 会在唤醒源已置位时不进入休眠,
         *   直接继续执行, 不会长时间挂起。 */
        __disable_irq();
        if (!imu_update_flag && !usart_send_flag)
        {
            __enable_irq();
            __WFI();
        }
        else
        {
            __enable_irq();
        }
    }
}

/*
 * 定时器中断处理函数 (TIM2, 10ms 周期)
 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        system_tick++;

        /* 姿态解算标志: 100Hz = 10ms */
        imu_update_flag = 1;

        /* 串口发送标志: 20Hz = 50ms, 每 5 个滴答发送一次 */
        if (system_tick % 5 == 0)
        {
            usart_send_flag = 1;
        }

        /* LED 闪烁: 1Hz = 1 秒 */
        if (system_tick % 100 == 0)
        {
            if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13))
                GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            else
                GPIO_SetBits(GPIOC, GPIO_Pin_13);
        }
    }
}

/*
 * 按协议打包并发送姿态数据
 * 数据包格式: $Roll,Pitch,Yaw*CS\r\n
 * CS = '$' 之后到 '*' 之前所有字符的逐字节异或
 */
static void USART1_SendData(void)
{
    uint8_t checksum = 0;
    uint8_t i;

    /* 一次 sprintf 生成完整数据帧(含校验和占位), 避免拼接多个缓冲区 */
    sprintf(usart_buffer, "$%.2f,%.2f,%.2f*00\r\n", Roll, Pitch, Yaw);

    /* 计算校验和: 从第 1 个字符 '$' 开始, 到 '*' 之前结束 */
    for (i = 1; usart_buffer[i] != '*' && usart_buffer[i] != '\0'; i++)
    {
        checksum ^= (uint8_t)usart_buffer[i];
    }

    /* 把校验和写回 '*' 后面的两个十六进制位 */
    sprintf(&usart_buffer[i + 1], "%02X\r\n", checksum);

    USART1_SendString(usart_buffer);

    packet_counter++;
}

/*
 * 阻塞式发送一个字符串
 */
static void USART1_SendString(const char *str)
{
    while (*str)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, (uint16_t)(uint8_t)(*str++));
    }
    /* 等待最后一个字节真正移出, 保证调用返回时数据已发出 */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/*
 * OLED 显示带 1 位小数的浮点数
 * 修正点: 原实现用 ((int)(Value * 10)) % 10 取小数位,
 *         当 Value 为负数时取模结果仍为负, 传给无符号形参会显示乱码。
 *         这里先取绝对值再拆分整数位和小数位。
 */
static void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Value)
{
    int32_t scaled = (int32_t)(Value * 10.0f);   /* 放大 10 倍后四舍五入到整数 */
    int32_t abs_val = (scaled < 0) ? -scaled : scaled;

    if (Value < 0)
    {
        /* 显示负号, '0' 字符位留给数字 */
        OLED_ShowChar(Line, Column, '-');
        OLED_ShowNum(Line, Column + 1, (uint32_t)(abs_val / 10), 2);
        OLED_ShowChar(Line, Column + 3, '.');
        OLED_ShowNum(Line, Column + 4, (uint32_t)(abs_val % 10), 1);
    }
    else
    {
        OLED_ShowChar(Line, Column, '+');
        OLED_ShowNum(Line, Column + 1, (uint32_t)(abs_val / 10), 2);
        OLED_ShowChar(Line, Column + 3, '.');
        OLED_ShowNum(Line, Column + 4, (uint32_t)(abs_val % 10), 1);
    }
}

/*
 * PC13 板载 LED 初始化
 */
static void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 高电平熄灭(板载 LED 阴极接 PC13, 低电平点亮) */
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

/*
 * 系统时钟配置: HSE 8MHz * 9 = 72MHz
 */
static void SystemClock_Config(void)
{
    /* 开启 HSE 晶振 */
    RCC->CR |= ((uint32_t)RCC_CR_HSEON);
    while ((RCC->CR & RCC_CR_HSERDY) == 0);

    /* 设置 FLASH 等待状态: 72MHz 需要 2 个等待周期 */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /* 配置 AHB、APB1、APB2 分频 */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;   /* HCLK  = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;  /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;  /* PCLK1 = HCLK/2 */

    /* 配置 PLL: HSE * 9 = 72MHz */
    RCC->CFGR &= (uint32_t)~RCC_CFGR_PLLMULL;
    RCC->CFGR &= (uint32_t)~RCC_CFGR_PLLSRC;
    RCC->CFGR &= (uint32_t)~RCC_CFGR_PLLXTPRE;
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);

    /* 使能 PLL */
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);

    /* 选择 PLL 作为系统时钟 */
    RCC->CFGR &= (uint32_t)~RCC_CFGR_SW;
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL);

    /* 更新 SystemCoreClock 变量, SysTick 等依赖该值 */
    SystemCoreClock = 72000000;
}
