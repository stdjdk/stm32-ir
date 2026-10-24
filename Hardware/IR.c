/**
  ******************************************************************************
  * @file    IR.c
  * @brief   NEC 红外遥控接收驱动（STM32F103 + 标准外设库）
  *
  * 一、硬件怎么接
  *     YX62876 / VS1838B 等 38kHz 红外接收头：
  *         VCC  -> 3.3V
  *         GND  -> GND
  *         OUT  -> PA0
  *     注意：接收头是"反相"输出的 —— 有 38kHz 载波时输出【低】，没载波时输出【高】。
  *
  * 二、NEC 协议长什么样（本文件的判断依据）
  *
  *          ┌──── 起始码 ────┐┌────────── 32 位数据 ──────────┐
  *     低   │◄──── 9ms ────►││ 每一位都是 0.56ms 低 + 一段高   │
  *     高                    │◄── 4.5ms ──►│                    │
  *
  *     数据 0：低 0.56ms + 高 0.56ms  ->  两个下降沿相隔 1.12ms
  *     数据 1：低 0.56ms + 高 1.69ms  ->  两个下降沿相隔 2.25ms
  *     起始码：低 9ms    + 高 4.5ms   ->  两个下降沿相隔 13.5ms
  *     连发码：低 9ms    + 高 2.25ms  ->  两个下降沿相隔 11.25ms
  *
  *     32 位数据 = 地址码 + 地址反码 + 命令码 + 命令反码（低位先发）
  *
  * 三、解码思路（本文件的核心）
  *     1) TIM3 配成 1MHz 自由计数 -> 每 1 个数 = 1 微秒，相当于一块不停走的秒表；
  *     2) PA0 配成 EXTI0 下降沿中断 -> 每来一个"载波段的开始"就进一次中断；
  *     3) 中断里读秒表、减去上次的值 = 两次下降沿的间隔；
  *     4) 用间隔去查表：13.5ms=起始码，1.12ms=0，2.25ms=1，11.25ms=连发；
  *     5) 攒够 32 位后做"反码校验"，通过就置 IR_DataFlag 通知主程序。
  *
  * 四、为什么只认"下降沿"
  *     接收头输出反相，每次"载波段开始"都是高->低。
  *     又因为 0 和 1 的差别在"载波后面的停顿"，所以测量
  *     "上一个下降沿到这一个下降沿"的整段时间（= 载波 + 停顿），
  *     一个数字里就同时包含了这两段信息，每一位只需要一次中断。
  ******************************************************************************
  */

#include "IR.h"

/* ============================================================================
 * 一、对外数据（主程序通过 IR_GetXXX() 读取）
 * ==========================================================================*/

/**
  * @brief 32 位原始数据的"草稿箱"，4 个字节 = 32 位。
  *        收位时就往这里填：
  *          IR_Data[0] = 地址码
  *          IR_Data[1] = 地址反码
  *          IR_Data[2] = 命令码
  *          IR_Data[3] = 命令反码
  * @note  用 static 是为了只在本文件可见，外部只能通过接口函数读，防止误改；
  *        用 volatile 是因为它会在中断里被修改，编译器不能把它优化到寄存器里。
  */
static volatile uint8_t IR_Data[4];

/** @brief 解析出来的地址码（用户码）。你这套遥控器是 0x00。 */
static volatile uint8_t IR_Address;

/** @brief 解析出来的命令码（键值）。例如按"1"就是 0x45。 */
static volatile uint8_t IR_Command;

/** @brief "收到完整一帧"的旗子。中断里置 1，主程序读一次后自动清 0。 */
static volatile uint8_t IR_DataFlag;

/** @brief "收到连发帧"的旗子（按住不放时出现），用法同上。 */
static volatile uint8_t IR_RepeatFlag;


/* ============================================================================
 * 二、调试用的"仪表"（不影响功能，排查问题时在 OLED 上显示）
 *    正常使用时可以整块删掉，或者在 IR.h 里去掉对应的 extern 声明。
 * ==========================================================================*/

volatile uint32_t IR_EdgeCnt  = 0;      /* 进入 EXTI0 中断的总次数（= 下降沿个数）*/
volatile uint32_t IR_FrameCnt = 0;      /* 成功解出完整帧的次数 */
volatile uint16_t IR_LastDelta = 0;     /* 最近一次测得的间隔（单位 µs）*/
volatile uint16_t IR_MinDelta  = 0xFFFF;/* 有效间隔的最小值（看有没有异常短脉冲）*/
volatile uint16_t IR_MaxDelta  = 0;     /* 有效间隔的最大值（看有没有异常长脉冲）*/
volatile uint8_t  IR_Raw[4]    = {0};   /* 最近一帧的 4 个原始字节（不管校验是否通过）*/
volatile uint32_t IR_BadCnt    = 0;     /* 收满 32 位但反码校验失败的次数 */
volatile uint8_t  IR_MaxBits   = 0;     /* 一帧里最多收到过多少位（正常应到 32）*/
volatile uint16_t IR_ErrDelta  = 0;     /* 让解码中断的那个"非法间隔"的数值 */
volatile uint16_t IR_GlitchCnt = 0;     /* 被当成毛刺丢弃的边沿次数 */


/* ============================================================================
 * 三、内部状态（解码状态机的核心）
 * ==========================================================================*/

/**
  * @brief 解码状态。
  *        0 = 空闲：刚开机，或上一帧刚处理完，等待新的一帧
  *        1 = 等待起始码 / 连发码
  *        2 = 正在逐位接收 32 位数据
  */
static volatile uint8_t  IR_State  = 0;

/**
  * @brief 已经收到的位数（0~31，收到 32 表示一帧收满）。
  *        它同时决定"这一位该写进第几个字节的第几位"：
  *          IR_pData / 8 -> 第几个字节
  *          IR_pData % 8 -> 该字节的第几位
  */
static volatile uint8_t  IR_pData  = 0;

/**
  * @brief 是否已经记录过"上一次下降沿"。
  *        刚开机第一次进中断时没有"上一次"可比，只能记起点，不能算间隔。
  */
static volatile uint8_t  haveLast  = 0;

/**
  * @brief 上一次下降沿时 TIM3 的计数值（单位 µs）。
  *        用来和本次的计数相减，得到两次下降沿之间的间隔。
  */
static volatile uint16_t lastCnt   = 0;


/* ============================================================================
 * 四、时间判定的阈值（单位都是 µs，因为 TIM3 被配置成 1MHz，1 格 = 1µs）
 *
 *   下面的窗口是在"标准值"基础上放宽得到的，用来容忍遥控器个体差异、
 *   接收头延迟和信号抖动。窗口太窄会漏判，太宽会误判。
 * ==========================================================================*/

/**
  * @brief 毛刺门限。
  *        正常的位最短也有 560µs（载波段），所以小于 600µs 的边沿
  *        基本可以肯定是接收头切换瞬间的抖动/干扰，直接忽略。
  */
#define IR_GLITCH_MIN   600

/**
  * @brief 空闲门限。
  *        NEC 一帧内部最长的间隔是 13.5ms（起始码），两帧之间至少几十毫秒，
  *        所以间隔超过 20ms 就说明"上一帧结束了，这是一个新的开始"。
  * @note  ★ 这里不用 TIM3 的"更新(溢出)标志"来判断超时：
  *        NEC 一帧约 67.5ms，而 16 位定时器 65.5ms 就溢出一次，
  *        帧播到一半必然绕回，会被误判成丢帧，导致每帧都被清零。
  */
#define IR_IDLE_MIN     20000

/**
  * @brief 起始码窗口：标准 13.5ms，放宽到 12~16ms。
  *        实测你这套是 13.577ms，落在这个范围内。
  */
#define IR_LEADER_MIN   12000
#define IR_LEADER_MAX   16000

/**
  * @brief 连发码窗口：标准 11.25ms，取 9.5~12ms，与起始码窗口区分开。
  */
#define IR_REPEAT_MIN   9500
#define IR_REPEAT_MAX   12000

/**
  * @brief 位值判定分界点：数据 0 是 1.12ms，数据 1 是 2.25ms，
  *        取两者中点 (1120 + 2250) / 2 ≈ 1685。
  *        "就近取整"比死磕固定窗口更抗误差。
  */
#define IR_BIT0_MAX     1685

/**
  * @brief 合法位间隔的上限：超过 4ms 既不像 0 也不像 1，判为非法。
  */
#define IR_BIT1_MAX     4000


/* ============================================================================
 * 五、初始化
 * ==========================================================================*/

/**
  * @brief  红外接收初始化
  * @note   做四件事：
  *         1) PA0 配成上拉输入；
  *         2) PA0 -> EXTI0 下降沿中断；
  *         3) NVIC 允许 EXTI0 中断；
  *         4) TIM3 配成 1µs 一格的自由计数器（永不停）。
  * @param  无
  * @retval 无
  */
void IR_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    EXTI_InitTypeDef        EXTI_InitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    /* ---- ① 开时钟：GPIOA（引脚）+ AFIO（EXTI 选路）+ TIM3（计时） ----
       注意：AFIO 时钟是给"EXTI0 从哪个端口来"用的，漏开会导致中断不触发。 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* ---- ② PA0 配成上拉输入 ----
       接收头空闲时输出高，上拉可以防止接线松时引脚悬空乱触发。 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ---- ③ AFIO 选路：把 EXTI0 这条中断线接到 PA0 ----
       51 单片机里这一步是硬件焊死的，STM32 里必须软件指定。 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

    /* ---- ④ EXTI0 配置为下降沿触发 ----
       接收头"开始载波"时输出由高变低，所以每个下降沿 = 一个数据位的起点。 */
    EXTI_InitStructure.EXTI_Line    = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* ---- ⑤ NVIC：允许 EXTI0 中断，并设成较高优先级 ----
       红外信号是"连续快节奏"的，优先级高一点可以避免被慢任务拖到丢帧。 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   /* 全工程只需配置一次 */
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&NVIC_InitStructure);

    /* ---- ⑥ TIM3：配成 1MHz 自由计数器（16 位，量程 0~65535，约 65.5ms） ----
       分频计算：72MHz / (PSC + 1) = 72MHz / 72 = 1MHz
                 => 每计 1 个数 = 1 / 1MHz = 1µs
       注意：这里 PSC 写 72-1，是因为硬件会自动加 1；
             ARR 写 0xFFFF 表示数到 65535 后自动回到 0 继续数（16 位，不是 8 位）。 */
    TIM_TimeBaseStructure.TIM_Prescaler     = 72 - 1;   /* 预分频：72MHz -> 1MHz，1 格 = 1µs */
    TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;   /* 自动重装值：16 位满量程 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* ---- ⑦ 计数器清零并启动 ----
       一旦启动就一直数，不再关闭；中断里只"读差值"，不需要反复清零。 */
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}


/* ============================================================================
 * 六、对外接口
 * ==========================================================================*/

/**
  * @brief  查询是否收到完整一帧
  * @retval 1 = 收到（本次调用会把旗子清掉，保证一帧只处理一次）；0 = 没收到
  */
uint8_t IR_GetDataFlag(void)
{
    if (IR_DataFlag) { IR_DataFlag = 0; return 1; }
    return 0;
}

/**
  * @brief  查询是否收到连发帧（按住按键不放）
  * @retval 1 = 收到；0 = 没收到
  */
uint8_t IR_GetRepeatFlag(void)
{
    if (IR_RepeatFlag) { IR_RepeatFlag = 0; return 1; }
    return 0;
}

/** @brief 读取最近一次解析出的地址码（用户码） */
uint8_t IR_GetAddress(void) { return IR_Address; }

/** @brief 读取最近一次解析出的命令码（键值） */
uint8_t IR_GetCommand(void) { return IR_Command; }


/* ============================================================================
 * 七、核心：EXTI0 中断服务函数
 *
 *   每当 PA0 出现一个下降沿（= 一段载波的开始），就会进入这里。
 *   一帧 NEC 数据大约会进来 34 次（1 个起始码 + 32 个数据位 + 1 个收尾脉冲）。
 *
 *   执行顺序：
 *     1) 读 TIM3 当前值 -> now
 *     2) 若之前记过时间 -> 算出间隔 delta
 *     3) 过滤毛刺（太短的间隔离不算数）
 *     4) 进入状态机：根据 delta 的数值决定"这是什么"
 * ==========================================================================*/
void EXTI0_IRQHandler(void)
{
    /* 先判断确实是 EXTI0 触发（本工程只用了一条线，但保留这个判断是好习惯） */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        /* --------------------------------------------------------------
         * 局部变量说明（每次进中断重新创建，退出即销毁）
         * ------------------------------------------------------------*/
        uint16_t now    = TIM_GetCounter(TIM3);  /* 本次下降沿的"时刻"（µs）*/
        uint16_t delta  = 0;                     /* 与上一次下降沿的间隔（µs）*/
        uint8_t  glitch = 0;                     /* 本次是否为毛刺：1=是 */
        uint8_t  first  = 0;                     /* 本次是否为本轮第一个沿：1=是 */

        IR_EdgeCnt++;                            /* 调试：中断计数 +1 */

        /* ==============================================================
         * 第 1 步：算出"距上次下降沿过了多久"
         * ============================================================*/
        if (!haveLast)
        {
            /* 本轮第一次见到下降沿（刚开机，或上一帧刚处理完）：
               没有"上一次"可比，只记录起点，不做任何判断。 */
            first    = 1;
            haveLast = 1;                        /* 标记"现在有上一次了" */
            lastCnt  = now;                      /* 记下起点时刻 */
            IR_State = 1;                        /* 进入"等待起始码"状态 */
            IR_pData = 0;
        }
        else
        {
            /* 16 位无符号减法：即使计数器从 65535 回绕到 0，
               结果依然是正确的间隔（模 65536 运算）。
               例如 now=100、lastCnt=65500 时，100-65500 会自动得到 136。 */
            delta = (uint16_t)(now - lastCnt);
            IR_LastDelta = delta;                /* 调试：记录最近一次间隔 */

            if (delta < IR_GLITCH_MIN)
            {
                /* 间隔太短（<600µs）：接收头在切换瞬间抖了一下。
                   处理方式：丢弃本次边沿，并且【不更新 lastCnt】，
                   这样下一个真实边沿测得的是"跨越毛刺的整段时长"，不会丢位。 */
                glitch = 1;
                IR_GlitchCnt++;
            }
            else
            {
                /* 正常边沿：更新起点，并记录最小/最大间隔（调试用） */
                lastCnt = now;
                if (delta < IR_MinDelta) IR_MinDelta = delta;
                if (delta > IR_MaxDelta) IR_MaxDelta = delta;
            }
        }

        /* ==============================================================
         * 第 2 步：状态机
         *   - first（本轮第一个沿）和 glitch（毛刺）不参与判断
         * ============================================================*/
        if (!first && !glitch)
        {
            if (delta > IR_IDLE_MIN)
            {
                /* 间隔超过 20ms：这是两帧之间的空闲。
                   当前这个下降沿就是新一帧的第一个沿，所以回到"等待起始码"。
                   注意是回到状态 1 而不是 0：这个沿本身已经是新一帧的第一拍了，
                   若置 0 会把它浪费掉，导致下一个 13.5ms 的起始码被当成"第一个沿"。 */
                IR_State = 1;
                IR_pData = 0;
            }
            else if (IR_State == 0)
            {
                /* 空闲状态下收到的第一个短间隔：同样当作新一帧的开始 */
                IR_State = 1;
                IR_pData = 0;
            }
            else if (IR_State == 1)
            {
                /* ---------- 状态 1：等待起始码或连发码 ---------- */
                if (delta > IR_LEADER_MIN && delta < IR_LEADER_MAX)
                {
                    /* 12~16ms：这是 NEC 的起始码（标准 13.5ms）。
                       代表"接下来要发 32 位数据了"，进入接收状态。 */
                    IR_State = 2;
                    IR_pData = 0;                /* 从第 0 位开始收 */
                }
                else if (delta > IR_REPEAT_MIN && delta < IR_REPEAT_MAX)
                {
                    /* 9.5~12ms：这是连发码（标准 11.25ms）。
                       说明按键一直被按住，不需要重复接收数据，
                       只置一个标志通知主程序"还是刚才那个键"。 */
                    IR_RepeatFlag = 1;
                    IR_State = 1;
                }
                /* 其它情况：既不是起始码也不是连发码，保持状态 1 继续等 */
            }
            else /* IR_State == 2：正在接收 32 位数据 */
            {
                uint8_t bit;                     /* 本次解出的位值 */

                /* ---------- 位值判定：就近取整 ----------
                   1.12ms（数据0）与 2.25ms（数据1）以 1.685ms 为界：
                   小于它算 0，大于等于它算 1；
                   超过 4ms 则既不像 0 也不像 1，判为非法。 */
                if (delta > IR_GLITCH_MIN && delta < IR_BIT0_MAX)
                    bit = 0;
                else if (delta >= IR_BIT0_MAX && delta < IR_BIT1_MAX)
                    bit = 1;
                else
                    bit = 0xFF;                  /* 0xFF 作为"非法"标记 */

                if (bit == 0xFF)
                {
                    /* 出现非法间隔：记录数值（调试用），本帧作废，退回等待起始码 */
                    IR_ErrDelta = delta;
                    IR_pData = 0;
                    IR_State = 1;
                }
                else
                {
                    /* ---------- 把这一位写进草稿箱 ----------
                       IR_pData / 8 -> 第几个字节（0~3）
                       IR_pData % 8 -> 该字节的第几位（0~7）
                       因为 NEC 是"低位先发"，所以从 bit0 开始往高位填，
                       收到的顺序与存储顺序天然一致，无需翻转。 */
                    if (bit == 0)
                        IR_Data[IR_pData / 8] &= ~(0x01 << (IR_pData % 8));  /* 该位清 0 */
                    else
                        IR_Data[IR_pData / 8] |=  (0x01 << (IR_pData % 8));  /* 该位置 1 */

                    IR_pData++;                  /* 位数 +1 */
                    if (IR_pData > IR_MaxBits)
                        IR_MaxBits = IR_pData;   /* 调试：记录最多收到几位 */

                    /* ---------- 32 位收满，开始"验货" ---------- */
                    if (IR_pData >= 32)
                    {
                        IR_pData = 0;

                        /* 保存一份原始字节（调试用，不管校验是否通过都保存） */
                        IR_Raw[0] = IR_Data[0];
                        IR_Raw[1] = IR_Data[1];
                        IR_Raw[2] = IR_Data[2];
                        IR_Raw[3] = IR_Data[3];

                        /* 反码校验：
                           NEC 把"命令码"和"命令反码"成对发送，两者必然逐位相反。
                           只校验这一对，是因为标准 NEC 和扩展 NEC 都成立
                           （标准 NEC 是 地址/地址反码/命令/命令反码，
                             扩展 NEC 是 地址低/地址高/命令/命令反码）。

                           ★ 必须强转 (uint8_t)：
                             C 语言里 unsigned char 参与运算会先提升为 int，
                             ~IR_Data[3] 会变成 0xFFFFFF00 这类 32 位数，
                             与 0~255 的字节比较永远不相等 —— 校验会永远失败。 */
                        if (IR_Data[2] == (uint8_t)~IR_Data[3])
                        {
                            IR_Address  = IR_Data[0];   /* 地址码 */
                            IR_Command  = IR_Data[2];   /* 命令码（键值） */
                            IR_DataFlag = 1;            /* 举旗子，通知主程序 */
                            IR_FrameCnt++;              /* 调试：成功帧计数 */
                        }
                        else
                        {
                            IR_BadCnt++;                /* 调试：校验失败计数 */
                        }

                        /* 一帧处理完，回到空闲。
                           haveLast 清零很关键：下一帧的第一个沿会被当成"第一次"，
                           否则会拿上一帧最后一个沿去算间隔，得到一个巨大的假值。 */
                        IR_State = 0;
                        haveLast = 0;
                    }
                }
            }
        }

        /* 最后：清除 EXTI 挂起标志。
           不清的话中断会反复触发，主程序将无法运行。 */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
