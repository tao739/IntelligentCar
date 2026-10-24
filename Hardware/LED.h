#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

/*==============================================================================
 * LED.h —— 指示灯驱动（S1）
 *----------------------------------------------------------------------------
 * 两颗灯（引脚定义在 LED.c 顶部）：
 *   LED_ID_RUN    = PE6 运行灯：待启动慢闪、运行常亮、完成慢闪
 *   LED_ID_STATUS = PE7 状态灯：按键/顶点提示闪一下、异常快闪
 *
 * 闪烁全部由 LED_Tick1ms() 推进，调用后立刻返回，不阻塞主循环。
 *============================================================================*/

typedef enum
{
	LED_ID_RUN    = 0,		/* 运行灯 PE6 */
	LED_ID_STATUS = 1		/* 状态灯 PE7 */
} LedId_t;

typedef enum
{
	LED_PAT_OFF = 0,		/* 常灭 */
	LED_PAT_ON,				/* 常亮 */
	LED_PAT_SLOW,			/* 慢闪 500ms 亮 / 500ms 灭 */
	LED_PAT_FAST			/* 快闪 100ms 亮 / 100ms 灭 */
} LedPattern_t;

void LED_Init(void);
void LED_Set(LedId_t id, uint8_t on);					/* 常亮/常灭 */
void LED_SetPattern(LedId_t id, LedPattern_t pat);		/* 切换闪烁花样 */
void LED_AllOff(void);
void LED_Tick1ms(void);									/* 每 1ms 调用一次 */

#endif
