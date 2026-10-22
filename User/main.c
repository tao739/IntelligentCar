#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "AD.h"
#include "Serial.h"
#include "IR.h"
#include "S2_IR.h"

int main(void)
{
	uint32_t time_ms = 0;
	uint8_t print_cnt = 0;
	const S2_IR_State *ir;

	/* 模块初始化 */
	Serial_Init();          // USART1，PA9/PA10，115200
	IR_Init();              // PB1高电平，使能红外管
	AD_Init();              // ADC1扫描8通道 + DMA循环采集
	S2_IR_Init();           // 阈值/滤波/位置处理

	Serial_Printf("\r\nS2 8CH IR ADC+DMA\r\n");
	Serial_Printf("VET6: D1-D7 PA1-PA7, D8 PC0, IR PB1\r\n");
	Serial_Printf("t_ms,raw0,raw1,raw2,raw3,raw4,raw5,raw6,raw7,found,pos,mask,conf\r\n");

	while (1)
	{
		/* 10ms更新一次，S3/状态机用这个节奏 */
		S2_IR_Update();
		Delay_ms(10);
		time_ms += 10;

		/* 100ms打印一次，10Hz，方便看串口 */
		print_cnt++;
		if (print_cnt >= 10U)
		{
			print_cnt = 0U;
			ir = S2_IR_Get();

			Serial_Printf("%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,0x%02X,%u\r\n",
						  (unsigned)time_ms,
						  (unsigned)AD_Value[0],
						  (unsigned)AD_Value[1],
						  (unsigned)AD_Value[2],
						  (unsigned)AD_Value[3],
						  (unsigned)AD_Value[4],
						  (unsigned)AD_Value[5],
						  (unsigned)AD_Value[6],
						  (unsigned)AD_Value[7],
						  (unsigned)ir->line_found,
						  (int)ir->position_mm,
						  (unsigned)ir->active_mask,
						  (unsigned)ir->confidence);
		}
	}
}
