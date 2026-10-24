#include "stm32f10x.h"
#include "LED.h"

/*==============================================================================
 * LED.c —— 指示灯驱动（S1）
 *----------------------------------------------------------------------------
 * 引脚配置放在本文件顶部；换板子/换灯只改这里。
 *
 * 实现方式：每个灯一个"状态"（花样 + 亮灭 + 上次翻转时刻），
 * 每 1ms 由 LED_Tick1ms() 推进一次 —— 不用 Delay，也不占用定时器。
 * 时间基准用本文件内部的 1ms 计数，不依赖其它模块。
 *============================================================================*/

/*------------------------- 引脚配置（改接线只改这里） ---------------------*/
#define LED_CLK          RCC_APB2Periph_GPIOE
#define LED_PORT         GPIOE
#define LED1_PIN         GPIO_Pin_6		/* 运行灯 */
#define LED2_PIN         GPIO_Pin_7		/* 状态灯 */
#define LED_ACTIVE_LOW   1				/* 1 = 低电平点亮 */

/* 花样参数 */
#define LED_SLOW_HALF_MS 500
#define LED_FAST_HALF_MS 100

typedef struct
{
	uint8_t  pat;			/* 当前花样 */
	uint8_t  on;			/* 当前亮灭 */
	uint16_t half_ms;		/* 半个周期（亮多久/灭多久） */
	uint32_t tick;			/* 本次亮/灭开始的时刻 */
} LedCtl_t;

static LedCtl_t  s_led[2];
static uint32_t  s_ms = 0;		/* 内部 1ms 计数 */

/* 底层：把"亮灭"写到引脚（考虑低电平点亮） */
static void LED_WritePin(LedId_t id, uint8_t on)
{
	uint16_t pin = (id == LED_ID_RUN) ? LED1_PIN : LED2_PIN;
	BitAction level;

#if (LED_ACTIVE_LOW != 0)
	level = (on != 0) ? Bit_RESET : Bit_SET;
#else
	level = (on != 0) ? Bit_SET : Bit_RESET;
#endif
	GPIO_WriteBit(LED_PORT, pin, level);
}

/**
  * 函    数：指示灯初始化（上电全灭）
  */
void LED_Init(void)
{
	GPIO_InitTypeDef g;

	RCC_APB2PeriphClockCmd(LED_CLK, ENABLE);

	g.GPIO_Mode  = GPIO_Mode_Out_PP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	g.GPIO_Pin   = LED1_PIN | LED2_PIN;
	GPIO_Init(LED_PORT, &g);

	s_led[0].pat = LED_PAT_OFF; s_led[0].on = 0; s_led[0].tick = 0; s_led[0].half_ms = LED_SLOW_HALF_MS;
	s_led[1].pat = LED_PAT_OFF; s_led[1].on = 0; s_led[1].tick = 0; s_led[1].half_ms = LED_SLOW_HALF_MS;
	s_ms = 0;

	LED_AllOff();
}

/**
  * 函    数：全部熄灭
  */
void LED_AllOff(void)
{
	s_led[0].pat = LED_PAT_OFF; s_led[0].on = 0; LED_WritePin(LED_ID_RUN, 0);
	s_led[1].pat = LED_PAT_OFF; s_led[1].on = 0; LED_WritePin(LED_ID_STATUS, 0);
}

/**
  * 函    数：常亮 / 常灭
  */
void LED_Set(LedId_t id, uint8_t on)
{
	if (id > LED_ID_STATUS) { return; }

	s_led[id].pat = (on != 0) ? LED_PAT_ON : LED_PAT_OFF;
	s_led[id].on = on;
	LED_WritePin(id, on);
}

/**
  * 函    数：切换闪烁花样
  */
void LED_SetPattern(LedId_t id, LedPattern_t pat)
{
	if (id > LED_ID_STATUS) { return; }

	if ((pat == LED_PAT_OFF) || (pat == LED_PAT_ON))
	{
		LED_Set(id, (uint8_t)((pat == LED_PAT_ON) ? 1 : 0));
		return;
	}

	s_led[id].pat = (uint8_t)pat;
	s_led[id].half_ms = (uint16_t)((pat == LED_PAT_FAST) ? LED_FAST_HALF_MS : LED_SLOW_HALF_MS);
	s_led[id].tick = s_ms;
	s_led[id].on = 1;
	LED_WritePin(id, 1);
}

/**
  * 函    数：闪烁推进，每 1ms 调用一次
  */
void LED_Tick1ms(void)
{
	uint8_t i;

	s_ms++;

	for (i = 0; i < 2; i++)
	{
		if ((s_led[i].pat != LED_PAT_SLOW) && (s_led[i].pat != LED_PAT_FAST)) { continue; }

		if ((s_ms - s_led[i].tick) >= s_led[i].half_ms)
		{
			s_led[i].tick = s_ms;
			s_led[i].on = (uint8_t)((s_led[i].on != 0) ? 0 : 1);
			LED_WritePin((LedId_t)i, s_led[i].on);
		}
	}
}
