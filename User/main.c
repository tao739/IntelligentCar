/*==============================================================================
 * 文件名 ：main.c
 * 作  用 ：S1（系统与软件负责人）主控程序 —— 项目一：A→B 直行定点停车
 *
 * 平台 ：STM32F103 + 标准外设库（SPL）
 *
 * 本文件只做"整车调度"：
 *   一键启动 / 状态机 / 声光事件映射 / 异常与复位 / 串口调试 / 1ms 调度
 *
 * 模块划分（各管各的，main.c 只调用）：
 *   Hardware/AD.c      S2：ADC1 8 通道 + DMA 循环采集
 *   Hardware/IR.c      S2：PB1 红外发射管使能
 *   Hardware/Serial.c  S2：USART1 115200 + printf
 *   Hardware/S2_IR.c   S2：阈值/回差/位图/位置（+AnyBlack/AllWhite）
 *   System/Task1.c     S2：项目一的判定（离开A / 到B / 超时）
 *   System/Delay.c     S2：延时（初始化用）
 *   Hardware/Key.c     S1：按键消抖（PC13 / PE5）
 *   Hardware/LED.c     S1：两颗指示灯（PE6 / PE7）
 *   Hardware/Buzzer.c  S1：蜂鸣器（PA8 = TIM1_CH1）
 *   System/Timer.c     S1：1ms 系统时基（TIM2，按键/灯/蜂鸣器/状态机都靠它）
 *   Hardware/Motor.c   S3：电机（等交付，本文件留 3 个空接口）
 *
 * 操作流程：
 *   上电 → 待启动（运行灯慢闪）
 *   短按 PC13 → 一键启动（一声长鸣 + 运行灯常亮）
 *   车直行，S2 的 Task1 判断是否到 B
 *   到 B  → 停车 + 三短音 + 双灯慢闪       （项目一要求：停车有明显声光提示）
 *   超时  → 停车 + 五声急促 + 双灯快闪     （13.5s 未到 B）
 *   按 PE5 → 立即停车（急停）
 *   完成/异常后 短按 PC13 → 复位回待启动
 *
 * 串口调试（115200 8N1，发送勾"加回车换行"）：
 *   RUN 启动 / STOP 停车 / RESET 复位 / STATUS 查状态
 *============================================================================*/

#include "stm32f10x.h"
#include "Serial.h"
#include "IR.h"
#include "AD.h"
#include "S2_IR.h"
#include "Task1.h"
#include "Delay.h"
#include "Key.h"
#include "LED.h"
#include "Buzzer.h"
#include "Timer.h"

/*==============================================================================
 * 第 0 节  编译开关
 *============================================================================*/
#define MOTOR_READY        0		/* 0=S3 未交付（只打印） 1=已接 S3 电机 */
#define TASK1_PWM          500		/* 直行 PWM（0~1000），S3 接入后标定 */

/*==============================================================================
 * 第 1 节  参数
 *============================================================================*/
#define CTRL_MS             10		/* 控制/感知周期（Task1 要求 10ms 一次） */

/*==============================================================================
 * 第 2 节  基础工具
 *----------------------------------------------------------------------------
 * 1ms 时基不在这里 —— 已独立成 System/Timer.c（Timer_Init / Timer_GetMs /
 * Timer_TakeFlag1ms），本文件只调用它的接口，不关心 TIM2 怎么配。
 *============================================================================*/
static uint8_t S1_StrEq(const char *a, const char *b)
{
	uint8_t i = 0;

	if ((a == 0) || (b == 0)) { return 0; }
	while ((a[i] != '\0') && (b[i] != '\0'))
	{
		if (a[i] != b[i]) { return 0; }
		i++;
	}
	return (uint8_t)(((a[i] == '\0') && (b[i] == '\0')) ? 1 : 0);
}

/*==============================================================================
 * 第 3 节  声光事件映射（把"业务事件"翻译成灯 + 声音）
 *----------------------------------------------------------------------------
 * 项目一只有 5 种提示，集中在这里，以后加任务也只改这一节：
 *   待启动   ：运行灯慢闪
 *   启动     ：一声长音 + 运行灯常亮
 *   按键     ：滴一声
 *   到 B/完成：三短音 + 双灯慢闪（明显区别于"路上"的状态）
 *   异常/超时：五声急促 + 双灯快闪
 *============================================================================*/
static void S1_Fb_Idle(void)
{
	Buzzer_Stop();
	LED_SetPattern(LED_ID_RUN, LED_PAT_SLOW);
	LED_Set(LED_ID_STATUS, 0);
}

static void S1_Fb_Start(void)
{
	Buzzer_Play(1, 250, 0, 1500);		/* 一声长音 */
	LED_Set(LED_ID_RUN, 1);				/* 运行灯常亮 */
	LED_Set(LED_ID_STATUS, 0);
}

static void S1_Fb_Key(void)
{
	Buzzer_Play(1, 30, 0, 2600);
}

static void S1_Fb_AtB(void)
{
	Buzzer_Play(3, 120, 120, 2200);		/* 三短音 */
	LED_SetPattern(LED_ID_RUN, LED_PAT_SLOW);
	LED_SetPattern(LED_ID_STATUS, LED_PAT_SLOW);
}

static void S1_Fb_Fault(void)
{
	Buzzer_Play(5, 70, 70, 4200);		/* 五声急促 */
	LED_SetPattern(LED_ID_RUN, LED_PAT_FAST);
	LED_SetPattern(LED_ID_STATUS, LED_PAT_FAST);
}

/*==============================================================================
 * 第 4 节  电机接口（S3 的活，现在留空）
 *----------------------------------------------------------------------------
 * S3 交付后：
 *   1) 在 Hardware/Motor.c 里实现下面 3 个函数（名字必须一致）
 *   2) 把第 0 节 MOTOR_READY 改成 1
 *   3) 编译即可，本文件其它地方不用改
 *
 *   void Motor_Init(void);                  TB6612 + TIM8 PWM + 方向脚
 *   void Motor_Straight(int16_t pwm);       两轮同速前进，pwm 0~1000
 *   void Motor_Stop(void);                  停车
 *============================================================================*/
static int16_t g_pwm_cmd = 0;		/* 当前想给的 PWM（0~1000） */
static uint8_t g_motor_cmd = 0;		/* 1=正在给直行指令 */

#if (MOTOR_READY != 0)
extern void Motor_Init(void);
extern void Motor_Straight(int16_t pwm);
extern void Motor_Stop(void);
#endif

static void S1_Motor_Init(void)
{
#if (MOTOR_READY != 0)
	Motor_Init();
#endif
	g_pwm_cmd = 0;
	g_motor_cmd = 0;
}

static void S1_Motor_Straight(int16_t pwm)
{
	if (pwm < 0)    { pwm = 0; }
	if (pwm > 1000) { pwm = 1000; }

	g_pwm_cmd = pwm;
	g_motor_cmd = 1;
#if (MOTOR_READY != 0)
	Motor_Straight(g_pwm_cmd);
#endif
}

static void S1_Motor_Stop(void)
{
	g_pwm_cmd = 0;
	g_motor_cmd = 0;
#if (MOTOR_READY != 0)
	Motor_Stop();
#endif
}

/*==============================================================================
 * 第 5 节  串口调试指令（按键没接好时的后门）
 *============================================================================*/
#define CLI_LINE_MAX   16

static char    g_cli_buf[CLI_LINE_MAX];
static uint8_t g_cli_len = 0;

static void S1_StartRequest(void);
static void S1_StopRequest(void);
static void S1_ResetRequest(void);
static void S1_PrintStatus(void);

static void S1_Cli_Reset(void)
{
	g_cli_len = 0;
}

static char S1_Upper(char c)
{
	if ((c >= 'a') && (c <= 'z')) { return (char)(c - 32); }
	return c;
}

static void S1_Cli_Exec(char *line)
{
	uint8_t i = 0;
	uint8_t j = 0;
	char buf[CLI_LINE_MAX];

	while ((line[i] == ' ') || (line[i] == '\t')) { i++; }
	while ((line[i] != '\0') && (j < (CLI_LINE_MAX - 1)))
	{
		buf[j] = S1_Upper(line[i]);
		j++;
		i++;
	}
	buf[j] = '\0';
	while ((j > 0) && (buf[j - 1] == ' '))
	{
		j--;
		buf[j] = '\0';
	}
	if (j == 0) { return; }

	if (S1_StrEq(buf, "RUN"))    { S1_StartRequest(); return; }
	if (S1_StrEq(buf, "STOP"))   { S1_StopRequest();  return; }
	if (S1_StrEq(buf, "RESET") || S1_StrEq(buf, "R")) { S1_ResetRequest(); return; }
	if (S1_StrEq(buf, "STATUS")) { S1_PrintStatus();  return; }

	Serial_Printf("ERR %s\r\n", buf);
}

static void S1_Cli_Poll(void)
{
	while (Serial_GetRxFlag() != 0)
	{
		uint8_t ch = Serial_GetRxData();

		if ((ch == '\r') || (ch == '\n'))
		{
			if (g_cli_len > 0)
			{
				g_cli_buf[g_cli_len] = '\0';
				S1_Cli_Exec(g_cli_buf);
				g_cli_len = 0;
			}
		}
		else if (ch >= 0x20)
		{
			if (g_cli_len < (CLI_LINE_MAX - 1))
			{
				g_cli_buf[g_cli_len] = (char)ch;
				g_cli_len++;
			}
			else
			{
				g_cli_len = 0;
			}
		}
	}
}

/*==============================================================================
 * 第 6 节  整车状态机
 *----------------------------------------------------------------------------
 *   IDLE ──按键/串口 RUN──> RUN ──Task1 到 B──> FINISH
 *                              ├──Task1 超时──> TIMEOUT
 *                              └──按 PE5──────> ESTOP
 *   FINISH / TIMEOUT / ESTOP ──按键复位──> IDLE
 *
 * RUN 里每 10ms：
 *   ① S2_IR_Update() 更新感知
 *   ② Task1_Update() 让 S2 的裁判走一步（累计 10ms，判到 B / 超时）
 *   ③ Task1_StopRequested() 为 1 → 停车 + 声光
 *============================================================================*/
#define SM_IDLE      0
#define SM_RUN       1
#define SM_FINISH    2
#define SM_TIMEOUT   3
#define SM_ESTOP     4

static uint8_t  g_sm = SM_IDLE;
static uint32_t g_run_tick = 0;

static const char *S1_StateName(uint8_t st)
{
	switch (st)
	{
		case SM_IDLE:    return "IDLE";
		case SM_RUN:     return "RUN";
		case SM_FINISH:  return "FINISH";
		case SM_TIMEOUT: return "TIMEOUT";
		case SM_ESTOP:   return "ESTOP";
		default:         return "?";
	}
}

static void S1_EnterIdle(const char *why)
{
	g_sm = SM_IDLE;
	S1_Motor_Stop();
	Task1_Init();
	S1_Fb_Idle();

	Serial_Printf("\r\n== IDLE == %s\r\n", why);
	Serial_Printf("短按PC13启动 串口RUN也可\r\n");
}

static void S1_StartRequest(void)
{
	if (g_sm == SM_RUN)
	{
		Serial_Printf("已在运行\r\n");
		return;
	}
	if (g_sm != SM_IDLE)
	{
		S1_ResetRequest();			/* 完成/异常状态下按启动键 = 先复位 */
		return;
	}

	Key_Flush();
	S1_Cli_Reset();
	g_run_tick = Timer_GetMs();

	Task1_Start();					/* S2 的裁判：从"等离开 A"开始 */
	S1_Motor_Straight(TASK1_PWM);	/* 开始直行 */
	S1_Fb_Start();
	g_sm = SM_RUN;

	Serial_Printf("\r\n== START == A->B\r\n");
	Serial_Printf("电机:%s PWM:%d\r\n",
				  (MOTOR_READY != 0) ? "S3已接" : "未接只打印", (int)TASK1_PWM);
}

static void S1_StopRequest(void)
{
	if (g_sm != SM_RUN)
	{
		Serial_Printf("当前不是运行态\r\n");
		return;
	}
	S1_Motor_Stop();
	S1_Fb_Fault();
	g_sm = SM_ESTOP;
	Serial_Printf("\r\n== ESTOP == 按PC13复位\r\n");
}

static void S1_ResetRequest(void)
{
	S1_Motor_Stop();
	Buzzer_Stop();
	Key_Flush();
	S1_Cli_Reset();
	S1_EnterIdle("人工复位");
}

static void S1_PrintStatus(void)
{
	const S2_IR_State *ir = S2_IR_Get();

	Serial_Printf("状态:%s\r\n", S1_StateName(g_sm));
	Serial_Printf("Task1:%u 用时:%lu ms\r\n",
				  (unsigned)Task1_GetState(), (unsigned long)Task1_GetElapsedMs());
	Serial_Printf("mask:0x%02X pos:%d line:%u\r\n",
				  (unsigned)ir->active_mask, (int)ir->position_mm, (unsigned)ir->line_found);
	Serial_Printf("PWM:%d cmd:%u\r\n", (int)g_pwm_cmd, (unsigned)g_motor_cmd);
}

/* RUN 状态下每 10ms：推进 S2 的 Task1 并处理结论 */
static void S1_RunTick10ms(void)
{
	Task1_State st;

	Task1_Update();
	st = Task1_GetState();

	if ((st == TASK1_WAIT_A_CLEAR) || (st == TASK1_RUN_TO_B))
	{
		S1_Motor_Straight(TASK1_PWM);		/* 还在找 B：保持直行 */
		return;
	}

	if (st == TASK1_AT_B)					/* 到 B：停车 + 三短音 + 双灯慢闪 */
	{
		S1_Motor_Stop();
		S1_Fb_AtB();
		g_sm = SM_FINISH;
		Serial_Printf("\r\n== FINISH == 到B停车\r\n");
		Serial_Printf("用时 %lu ms (限 15000)\r\n", (unsigned long)Task1_GetElapsedMs());
		Serial_Printf("按PC13复位\r\n");
		return;
	}

	if (st == TASK1_TIMEOUT)				/* 超时：停车 + 报警 */
	{
		S1_Motor_Stop();
		S1_Fb_Fault();
		g_sm = SM_TIMEOUT;
		Serial_Printf("\r\n== TIMEOUT == 13.5s没到B\r\n");
		Serial_Printf("按PC13复位\r\n");
		return;
	}
}

/* 按键事件分发 */
static void S1_Key_Dispatch(void)
{
	KeyEvent_t e = Key_GetEvent();

	if (e == KEY_EVT_START)
	{
		if (g_sm == SM_IDLE) { S1_StartRequest(); return; }	/* 一键启动 */
		if (g_sm == SM_RUN)  { return; }					/* 运行中忽略启动键 */
		S1_ResetRequest();									/* 完成后复位 */
		return;
	}

	if (e == KEY_EVT_STOP)
	{
		if (g_sm == SM_RUN) { S1_StopRequest(); }
		else                { S1_Fb_Key(); }
	}
}

/*==============================================================================
 * 第 7 节  开机信息 + 进度日志
 *============================================================================*/
static void S1_Banner(void)
{
	Serial_Printf("\r\n========================\r\n");
	Serial_Printf("S1 主控 V1.0 项目一\r\n");
	Serial_Printf("A->B 直行定点停车\r\n");
	Serial_Printf("限时15s 软件13.5s\r\n");
	Serial_Printf("短按PC13启动 PE5急停\r\n");
	Serial_Printf("串口 RUN STOP RESET\r\n");
	Serial_Printf("       STATUS\r\n");
	Serial_Printf("========================\r\n");
}

static void S1_Log_Progress(void)
{
	const S2_IR_State *ir = S2_IR_Get();

	if (g_sm != SM_RUN) { return; }

	Serial_Printf("t%lu st%u mask%02X pos%d pwm%d\r\n",
				  (unsigned long)(Timer_GetMs() - g_run_tick),
				  (unsigned)Task1_GetState(),
				  (unsigned)ir->active_mask,
				  (int)ir->position_mm,
				  (int)g_pwm_cmd);
}

/*==============================================================================
 * 第 8 节  main
 *----------------------------------------------------------------------------
 * 初始化顺序：S2 模块 → S1 驱动模块 → 时基 → 待启动
 * 主循环：高速轮询串口；1ms 跑按键/灯/蜂鸣器；10ms 跑感知+Task1；500ms 打进度
 *============================================================================*/
int main(void)
{
	uint8_t c10 = 0;
	uint8_t c500 = 0;

	/* ---- 1. S2 的模块（顺序按他 main 的写法） ---- */
	Serial_Init();			/* USART1 115200：PA9/PA10 */
	IR_Init();				/* PB1 高电平：红外发射管通电 */
	AD_Init();				/* ADC1 8 通道 + DMA 循环到 AD_Value[] */
	S2_IR_Init();			/* S2 的阈值/回差/位置处理 */
	Task1_Init();			/* S2 的项目一判定状态机 */
	Delay_ms(20);			/* 等 ADC 稳定几帧 */

	/* ---- 2. S1 的驱动模块 ---- */
	Key_Init();
	LED_Init();
	Buzzer_Init();
	S1_Motor_Init();
	Timer_Init();			/* TIM2 1ms 时基，最后开 */

	/* ---- 3. 开机 ---- */
	S1_Banner();
	S2_IR_Update();			/* 先更新一帧，避免首帧全 0 */
	S1_EnterIdle("上电完成");

	/* ---- 4. 后台循环 ---- */
	while (1)
	{
		/* (a) 串口高频轮询：保证 115200 不丢字节 */
		S1_Cli_Poll();

		if (Timer_TakeFlag1ms() == 0) { continue; }

		/* (b) 1ms：按键 / 指示灯 / 蜂鸣器 */
		Key_Tick1ms();
		LED_Tick1ms();
		Buzzer_Tick1ms();
		S1_Key_Dispatch();

		/* (c) 10ms：感知 + S2 的 Task1 + 状态机 */
		c10++;
		if (c10 >= CTRL_MS)
		{
			c10 = 0;

			S2_IR_Update();

			if (g_sm == SM_RUN) { S1_RunTick10ms(); }
			else                { S1_Motor_Stop(); }
		}

		/* (d) 500ms 进度日志 */
		c500++;
		if (c500 >= 50)
		{
			c500 = 0;
			S1_Log_Progress();
		}
	}
}
