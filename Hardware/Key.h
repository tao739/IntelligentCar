#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

/*==============================================================================
 * Key.h —— 按键驱动（S1）
 *----------------------------------------------------------------------------
 * 两个按键（引脚定义在 Key.c 顶部，改接线只改那一处）：
 *   KEY1 = PC13（WK_UP，按下高电平）：待启动时一键启动；完成/异常后复位
 *   KEY2 = PE5 （按下低电平）        ：运行中急停
 *
 * 特点（对比工程里原来那份教程模板）：
 *   1) 1ms 采样 + 20ms 计数消抖，全程非阻塞（原版用 Delay_ms 死等）
 *   2) 一次按键只产生一次"事件"，主循环用 Key_GetEvent() 取
 *   3) 引脚与 8 路红外（PA1~PA7/PC0）和红线使能（PB1）不冲突
 *============================================================================*/

/* 按键事件 */
typedef enum
{
	KEY_EVT_NONE = 0,
	KEY_EVT_START,		/* KEY1 短按：启动 / 复位 */
	KEY_EVT_STOP		/* KEY2 短按：急停 */
} KeyEvent_t;

void       Key_Init(void);
void       Key_Tick1ms(void);		/* 每 1ms 调用一次 */
KeyEvent_t Key_GetEvent(void);		/* 取一个事件，没有返回 KEY_EVT_NONE */
void       Key_Flush(void);			/* 清空未处理事件 */
uint8_t    Key_IsPressed(uint8_t idx);	/* 查询稳定按下状态（0=KEY1,1=KEY2，调试用） */

#endif
