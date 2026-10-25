#ifndef __HMC5883_REG_H
#define __HMC5883_REG_H


#ifdef __cplusplus
extern "C" {
#endif
/*
 * Honeywell HMC5883L 三轴磁力计寄存器定义
 * 数据手册: HMC5883L Datasheet (900405 Rev C)
 *
 * 注意: 本文件中的寄存器地址仅适用于 HMC5883L/HMC5883 原厂芯片。
 *       国产替代片 QMC5883L 的寄存器布局完全不同(配置在 0x09/0x0A,
 *       数据从 0x00 开始顺序为 X_L,X_H,Y_L,Y_H,Z_L,Z_H), 不可混用。
 */

/* ---- 配置寄存器 A: 数据输出速率 + 平均次数 ---- */
#define HMC5883_CRA        0x00
/* ---- 配置寄存器 B: 增益(量程) ---- */
#define HMC5883_CRB        0x01
/* ---- 模式寄存器: 测量模式 ---- */
#define HMC5883_MODE_REG   0x02
/* ---- 数据输出寄存器: X_H,X_L,Z_H,Z_L,Y_H,Y_L (注意 Z 在 Y 之前) ---- */
#define HMC5883_DO_X_H     0x03
#define HMC5883_DO_X_L     0x04
#define HMC5883_DO_Z_H     0x05
#define HMC5883_DO_Z_L     0x06
#define HMC5883_DO_Y_H     0x07
#define HMC5883_DO_Y_L     0x08
/* ---- 状态寄存器: bit0=RDY, bit1=LOCK ---- */
#define HMC5883_SR         0x09
/* ---- 识别寄存器 A/B/C: 依次返回 'H','4','3' ---- */
#define HMC5883_IRA        0x0A
#define HMC5883_IRB        0x0B
#define HMC5883_IRC        0x0C

/* ---- 配置寄存器 A 位域 ---- */
/* 平均次数: 8 次平均(可有效抑制噪声) */
#define HMC5883_CRA_MA_8   (0x03 << 5)
/* 数据输出速率: 75Hz */
#define HMC5883_CRA_DO_75  (0x06 << 2)
/* 测量模式: 正常测量(非正负偏置) */
#define HMC5883_CRA_MS_NOR (0x00 << 0)
/* 推荐配置: 8 次平均 + 75Hz */
#define HMC5883_CRA_CONFIG (HMC5883_CRA_MA_8 | HMC5883_CRA_DO_75 | HMC5883_CRA_MS_NOR)

/* ---- 配置寄存器 B 位域 (增益) ---- */
/* 增益 1090 LSB/Gauss, 量程 ±1.3Ga, 灵敏度 0.92 mGa/LSB —— 通用推荐值 */
#define HMC5883_CRB_GAIN_1090 0x20
/* 增益 1370 LSB/Gauss (默认值, ±0.88Ga) */
#define HMC5883_CRB_GAIN_1370 0x60

/* ---- 模式寄存器位域 ---- */
#define HMC5883_MODE_CONTINUOUS 0x00  /* 连续测量模式 */
#define HMC5883_MODE_SINGLE     0x01  /* 单次测量模式 */
#define HMC5883_MODE_IDLE       0x02  /* 空闲模式 */

/* ---- 状态寄存器位 ---- */
#define HMC5883_SR_RDY  0x01  /* 数据就绪 */
#define HMC5883_SR_LOCK 0x02  /* 数据被锁(正在读取) */

/* ---- 器件识别常量 ---- */
#define HMC5883_7BIT_ADDRESS 0x1E  /* 7 位 I2C 地址 */
#define HMC5883_ID_A         0x48  /* 'H' */
#define HMC5883_ID_B         0x34  /* '4' */
#define HMC5883_ID_C         0x33  /* '3' */


#ifdef __cplusplus
}
#endif

#endif
