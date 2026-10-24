#include "stm32f10x.h"
#include "Key.h"

/*==============================================================================
 * Key.c —— 按键消抖与事件（S1）
 *----------------------------------------------------------------------------
 * 原理：
 *   每 1ms 采一次电平；电平一变就开始计数，连续 20 次（20ms）都保持新电平
 *   才认为"真的变了"。中途只要抖回去一次，计数清零 —— 干扰被过滤掉。
 *
 *   按下时只记录状态；松开时产生一次"短按事件"。
 *   为什么在松开时产生事件：这样"按多久"不影响，一次按=一次事件，最稳。
 *============================================================================*/

/*------------------------- 引脚配置（改接线只改这里） ---------------------*/
#define KEY1_CLK        RCC_APB2Periph_GPIOC
#define KEY1_PORT       GPIOC
#define KEY1_PIN        GPIO_Pin_13
#define KEY1_ACTIVE     1			/* 1 = 按下高电平（WK_UP 接 3.3V）*/

#define KEY2_CLK        RCC_APB2Periph_GPIOE
#define KEY2_PORT       GPIOE
#define KEY2_PIN        GPIO_Pin_5
#define KEY2_ACTIVE     0			/* 0 = 按下低电平 */

#define KEY_DEBOUNCE_MS 20			/* 消抖时间 */
#define KEY_NUM         2

typedef struct
{
	GPIO_TypeDef *port;
	uint16_t      pin;
	uint8_t       active;		/* 1=高电平按下 */
	uint8_t       stable;		/* 消抖后的状态：1=按下 */
	uint8_t       cnt;			/* 候选电平连续次数 */
} KeyCfg_t;

static KeyCfg_t   s_key[KEY_NUM];
static KeyEvent_t s_evt = KEY_EVT_NONE;

/**
  * 函    数：按键初始化
  */
void Key_Init(void)
{
	GPIO_InitTypeDef g;

	RCC_APB2PeriphClockCmd(KEY1_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(KEY2_CLK, ENABLE);

	/* 高电平有效 → 内部下拉；低电平有效 → 内部上拉 */
	g.GPIO_Mode  = (KEY1_ACTIVE != 0) ? GPIO_Mode_IPD : GPIO_Mode_IPU;
	g.GPIO_Pin   = KEY1_PIN;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KEY1_PORT, &g);

	g.GPIO_Mode  = (KEY2_ACTIVE != 0) ? GPIO_Mode_IPD : GPIO_Mode_IPU;
	g.GPIO_Pin   = KEY2_PIN;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KEY2_PORT, &g);

	s_key[0].port = KEY1_PORT; s_key[0].pin = KEY1_PIN; s_key[0].active = KEY1_ACTIVE;
	s_key[1].port = KEY2_PORT; s_key[1].pin = KEY2_PIN; s_key[1].active = KEY2_ACTIVE;
	s_key[0].stable = 0; s_key[0].cnt = 0;
	s_key[1].stable = 0; s_key[1].cnt = 0;
	s_evt = KEY_EVT_NONE;
}

/**
  * 函    数：读某一键的"按下状态"（1=按下）
  */
static uint8_t Key_RawPressed(uint8_t idx)
{
	uint8_t level = (uint8_t)((GPIO_ReadInputDataBit(s_key[idx].port, s_key[idx].pin) != 0) ? 1 : 0);

	if (s_key[idx].active != 0) { return level; }
	return (uint8_t)((level == 0) ? 1 : 0);
}

/**
  * 函    数：按键状态机，每 1ms 调用一次
  */
void Key_Tick1ms(void)
{
	uint8_t i;

	for (i = 0; i < KEY_NUM; i++)
	{
		uint8_t raw = Key_RawPressed(i);

		if (raw != s_key[i].stable)
		{
			s_key[i].cnt++;
			if (s_key[i].cnt >= KEY_DEBOUNCE_MS)
			{
				s_key[i].cnt = 0;
				s_key[i].stable = raw;

				/* 只在"松开"时产生一次事件，保证按一次 = 一个事件 */
				if (raw == 0)
				{
					if (s_evt == KEY_EVT_NONE)
					{
						s_evt = (i == 0) ? KEY_EVT_START : KEY_EVT_STOP;
					}
				}
			}
		}
		else
		{
			s_key[i].cnt = 0;
		}
	}
}

/**
  * 函    数：取按键事件（取走后清空）
  */
KeyEvent_t Key_GetEvent(void)
{
	KeyEvent_t e = s_evt;
	s_evt = KEY_EVT_NONE;
	return e;
}

/**
  * 函    数：清空未处理事件（启动/复位时防止残留误触发）
  */
void Key_Flush(void)
{
	s_evt = KEY_EVT_NONE;
}

/**
  * 函    数：查询稳定按下状态（调试用）
  */
uint8_t Key_IsPressed(uint8_t idx)
{
	if (idx >= KEY_NUM) { return 0; }
	return s_key[idx].stable;
}
