#ifndef __IR_H
#define __IR_H

#include "stm32f10x.h"

/* ===== 键值（按卖家表：用户码 00 FF） ===== */
#define IR_1           0x45
#define IR_2           0x46
#define IR_3           0x47
#define IR_4           0x44
#define IR_5           0x40
#define IR_6           0x43
#define IR_7           0x07
#define IR_8           0x15
#define IR_9           0x09
#define IR_STAR        0x16
#define IR_0           0x19
#define IR_HASH        0x0D
#define IR_UP          0x18
#define IR_LEFT        0x08
#define IR_OK          0x1C
#define IR_RIGHT       0x5A
#define IR_DOWN        0x52

/* ===== 调试计数器（中断里自增，主循环可直接显示） ===== */
extern volatile uint32_t IR_EdgeCnt;     /* 进入中断的次数（下降沿数） */
extern volatile uint32_t IR_FrameCnt;    /* 成功解出一帧的次数 */
extern volatile uint16_t IR_LastDelta;   /* 最近测得的间隔：1 格 = 1µs，起始码应≈13500 */
extern volatile uint16_t IR_MinDelta;    /* 有效间隔最小值（调试用） */
extern volatile uint16_t IR_MaxDelta;    /* 有效间隔最大值（调试用） */
extern volatile uint8_t  IR_Raw[4];      /* 最近一帧的 4 个原始字节（调试用） */
extern volatile uint32_t IR_BadCnt;      /* 收满 32 位但校验失败的次数（调试用） */
extern volatile uint8_t  IR_MaxBits;     /* 一帧里最多收到的位数（调试用） */
extern volatile uint16_t IR_ErrDelta;    /* 让解码中断的那个非法间隔（调试用） */
extern volatile uint16_t IR_GlitchCnt;   /* 被过滤掉的毛刺次数（调试用） */

void    IR_Init(void);
uint8_t IR_GetDataFlag(void);      /* 收到完整一帧（读完自动清） */
uint8_t IR_GetRepeatFlag(void);    /* 收到连发帧（读完自动清） */
uint8_t IR_GetAddress(void);       /* 地址码（用户码） */
uint8_t IR_GetCommand(void);       /* 命令码（键值） */

#endif
