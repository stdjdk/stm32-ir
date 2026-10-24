#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Servo.h"
#include "IR.h"

float    Angle   = 0;		//舵机角度
uint8_t  LastCmd = 0;		//最近一次收到的命令码

/* 设为 1：上电时舵机自动从 0° 扫到 90° 再回中位，用来单独验证 PWM 通路
   设为 0：跳过自检（调好以后建议设 0） */
#define SERVO_SELFTEST	0

/* 设为 1：PA1 输出 1Hz 慢速方波（纯 GPIO，不用定时器）
   用来区分"引脚/接线"和"定时器 PWM"两类问题。
   注意：开启后会停在这个测试里，不再执行红外逻辑。 */
#define PWM_GPIO_TEST	0

/* 把命令码翻译成按键名，方便在 OLED 上直接看 */
static const char *KeyName(uint8_t cmd)
{
	switch (cmd)
	{
		case IR_1:     return "1";
		case IR_2:     return "2";
		case IR_3:     return "3";
		case IR_4:     return "4";
		case IR_5:     return "5";
		case IR_6:     return "6";
		case IR_7:     return "7";
		case IR_8:     return "8";
		case IR_9:     return "9";
		case IR_0:     return "0";
		case IR_STAR:  return "*";
		case IR_HASH:  return "#";
		case IR_UP:    return "UP";
		case IR_DOWN:  return "DN";
		case IR_LEFT:  return "LT";
		case IR_RIGHT: return "RT";
		case IR_OK:    return "OK";
		default:       return "?";
	}
}

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化

#if PWM_GPIO_TEST
	{
		GPIO_InitTypeDef gi;

		/* PA1 配成普通推挽输出，输出 1Hz 方波 */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		gi.GPIO_Pin   = GPIO_Pin_1;
		gi.GPIO_Mode  = GPIO_Mode_Out_PP;
		gi.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &gi);

		OLED_ShowString(1, 1, "GPIO TEST PA1");
		while (1)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_1);
			Delay_ms(500);
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
			Delay_ms(500);
		}
	}
#endif

	Servo_Init();		//舵机初始化（TIM2_CH2 → PA1）
	IR_Init();			//红外初始化（PA0 → EXTI0 + TIM3）

#if SERVO_SELFTEST
	{
		uint8_t i;
		/* 上电自检：慢速扫一遍，眼睛能看到舵机在动就说明 PWM 通路没问题 */
		for (i = 0; i <= 9; i++)
		{
			Servo_SetAngle((float)i * 10.0f);
			Delay_ms(200);
		}
		Servo_SetAngle(0);
		Delay_ms(300);
	}
#endif

	/*显示静态标题*/
	OLED_ShowString(1, 1, "Angle:");
	OLED_ShowString(2, 1, "Cmd:");
	OLED_ShowString(3, 1, "Key:");

	while (1)
	{
		/* ★ 收到一帧才处理一次（IR_GetDataFlag 读一次即清零） */
		if (IR_GetDataFlag())
		{
			LastCmd = IR_GetCommand();

			switch (LastCmd)
			{
				case IR_2:					//按 "2"：+30°
					Angle += 30;
					if (Angle > 90)
					{
						Angle = 0;
					}
					break;

				case IR_1:					//按 "1"：-30°
					Angle -= 30;
					if (Angle < 0)
					{
						Angle = 90;
					}
					break;

				case IR_UP:					//按 "↑"：+30°
					Angle += 30;
					if (Angle > 90)
					{
						Angle = 0;
					}
					break;

				case IR_DOWN:				//按 "↓"：-30°
					Angle -= 30;
					if (Angle < 0)
					{
						Angle = 90;
					}
					break;

				case IR_OK:					//按 "OK"：回 0°
					Angle = 0;
					break;

				default:
					break;
			}
		}

		Servo_SetAngle(Angle);								//设置舵机角度

		OLED_ShowNum   (1, 7, (uint32_t)Angle, 3);			//角度
		OLED_ShowHexNum(2, 6, LastCmd, 2);					//命令码（十六进制）
		OLED_ShowString(3, 6, (char *)KeyName(LastCmd));	//按键名
	}
}
