#include "IR.h"

/* ================= 对外数据 ================= */
static volatile uint8_t IR_Data[4];
static volatile uint8_t IR_Address;
static volatile uint8_t IR_Command;
static volatile uint8_t IR_DataFlag;
static volatile uint8_t IR_RepeatFlag;

/* ================= 调试计数器 ================= */
volatile uint32_t IR_EdgeCnt  = 0;
volatile uint32_t IR_FrameCnt = 0;
volatile uint16_t IR_LastDelta = 0;
volatile uint16_t IR_MinDelta  = 0xFFFF;
volatile uint16_t IR_MaxDelta  = 0;
volatile uint8_t  IR_Raw[4]    = {0};
volatile uint32_t IR_BadCnt    = 0;
volatile uint8_t  IR_MaxBits   = 0;
volatile uint16_t IR_ErrDelta  = 0;
volatile uint16_t IR_GlitchCnt = 0;

/* ================= 内部状态 ================= */
static volatile uint8_t  IR_State  = 0;    /* 0空闲 1等起始/连发 2收数据 */
static volatile uint8_t  IR_pData  = 0;
static volatile uint8_t  haveLast  = 0;
static volatile uint16_t lastCnt   = 0;

/* 时间参数（单位 µs；TIM3 被配成 1MHz，1 格 = 1µs） */
#define IR_GLITCH_MIN   600      /* 小于此间隔的沿 → 毛刺，丢弃 */
#define IR_IDLE_MIN     20000    /* 大于此间隔 → 空闲（两帧之间） */
#define IR_LEADER_MIN   12000    /* 起始码 12~16ms */
#define IR_LEADER_MAX   16000
#define IR_REPEAT_MIN   9500     /* 连发码 9.5~12ms */
#define IR_REPEAT_MAX   12000
#define IR_BIT0_MAX     1685     /* <1685 算 0，>=1685 算 1 */
#define IR_BIT1_MAX     4000     /* 超过算非法 */

/**
  * @brief  红外接收初始化：PA0 + EXTI0 下降沿 + TIM3 自由计数（1µs）
  * @note   ★ 不再使用 TIM3 的"更新(溢出)标志"判断超时。
  *         原因：NEC 一帧约 67.5ms，而 16 位定时器 65.5ms 就溢出一次，
  *         帧播到一半必然绕回，会被误判成"丢帧"，导致状态机每帧被清零。
  *         改为直接看"两次下降沿的间隔"是否 >20ms 来判断空闲。
  */
void IR_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    EXTI_InitTypeDef        EXTI_InitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* PA0 上拉输入 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* AFIO 路由：EXTI0 ← PA0 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

    /* EXTI0 下降沿触发 */
    EXTI_InitStructure.EXTI_Line    = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* NVIC */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&NVIC_InitStructure);

    /* TIM3：72MHz/72 = 1MHz → 每格 1µs */
    TIM_TimeBaseStructure.TIM_Prescaler     = 72 - 1;
    TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

/* ================= 对外接口 ================= */
uint8_t IR_GetDataFlag(void)
{
    if (IR_DataFlag) { IR_DataFlag = 0; return 1; }
    return 0;
}

uint8_t IR_GetRepeatFlag(void)
{
    if (IR_RepeatFlag) { IR_RepeatFlag = 0; return 1; }
    return 0;
}

uint8_t IR_GetAddress(void) { return IR_Address; }
uint8_t IR_GetCommand(void) { return IR_Command; }

/* ================= EXTI0 中断 ================= */
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        uint16_t now    = TIM_GetCounter(TIM3);
        uint16_t delta  = 0;
        uint8_t  glitch = 0;
        uint8_t  first  = 0;

        IR_EdgeCnt++;

        if (!haveLast)
        {
            /* 第一个沿：只记起点 */
            first    = 1;
            haveLast = 1;
            lastCnt  = now;
            IR_State = 1;
            IR_pData = 0;
        }
        else
        {
            delta = (uint16_t)(now - lastCnt);   /* 16 位减法自动处理回绕 */
            IR_LastDelta = delta;

            if (delta < IR_GLITCH_MIN)
            {
                /* 毛刺：丢弃这次沿，且不更新起点，
                   让下一个真实沿量到"跨越毛刺的整段时长" */
                glitch = 1;
                IR_GlitchCnt++;
            }
            else
            {
                lastCnt = now;
                if (delta < IR_MinDelta) IR_MinDelta = delta;
                if (delta > IR_MaxDelta) IR_MaxDelta = delta;
            }
        }

        if (!first && !glitch)
        {
            if (delta > IR_IDLE_MIN)
            {
                /* 间隔超过 20ms：说明是两帧之间的空闲，
                   当前这个沿就是新一帧的第一个沿 */
                IR_State = 1;
                IR_pData = 0;
            }
            else if (IR_State == 0)
            {
                IR_State = 1;
                IR_pData = 0;
            }
            else if (IR_State == 1)
            {
                if (delta > IR_LEADER_MIN && delta < IR_LEADER_MAX)
                {
                    IR_State = 2;          /* 起始码 OK */
                    IR_pData = 0;
                }
                else if (delta > IR_REPEAT_MIN && delta < IR_REPEAT_MAX)
                {
                    IR_RepeatFlag = 1;     /* 连发码 */
                    IR_State = 1;
                }
                /* 其它：保持状态 1，继续等起始码 */
            }
            else /* IR_State == 2：收数据 */
            {
                uint8_t bit;

                if (delta > IR_GLITCH_MIN && delta < IR_BIT0_MAX)
                    bit = 0;
                else if (delta >= IR_BIT0_MAX && delta < IR_BIT1_MAX)
                    bit = 1;
                else
                    bit = 0xFF;            /* 非法 */

                if (bit == 0xFF)
                {
                    IR_ErrDelta = delta;
                    IR_pData = 0;
                    IR_State = 1;
                }
                else
                {
                    if (bit == 0)
                        IR_Data[IR_pData / 8] &= ~(0x01 << (IR_pData % 8));
                    else
                        IR_Data[IR_pData / 8] |= (0x01 << (IR_pData % 8));

                    IR_pData++;
                    if (IR_pData > IR_MaxBits)
                        IR_MaxBits = IR_pData;

                    if (IR_pData >= 32)    /* 收满 32 位 */
                    {
                        IR_pData = 0;

                        IR_Raw[0] = IR_Data[0];
                        IR_Raw[1] = IR_Data[1];
                        IR_Raw[2] = IR_Data[2];
                        IR_Raw[3] = IR_Data[3];

                        /* 命令码与命令反码必然互补（标准/扩展 NEC 都成立）
                           必须强转 uint8_t，否则 ~ 会提升为 32 位导致永不相等 */
                        if (IR_Data[2] == (uint8_t)~IR_Data[3])
                        {
                            IR_Address  = IR_Data[0];
                            IR_Command  = IR_Data[2];
                            IR_DataFlag = 1;
                            IR_FrameCnt++;
                        }
                        else
                        {
                            IR_BadCnt++;
                        }

                        IR_State = 0;
                        haveLast = 0;
                    }
                }
            }
        }

        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
