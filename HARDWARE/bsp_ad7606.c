/*
*********************************************************************************************************
*
*	模块名称 : AD7606数据采集模块
*	文件名称 : bsp_ad7606.c
*
*********************************************************************************************************
*/
#include "bsp_ad7606.h"
#include "FreeRTOS.h"      //os 使用	 
#include "semphr.h"
#include "task.h"

extern TaskHandle_t DATAPROTask_Handler;	//AD数据接收任务句柄
volatile u32 GetADNum=0;						//AD采样计数器
AD7606_VAR_T g_tAD7606;		/* 定义1个全局变量，保存一些参数 */
AD7606_FIFO_T g_tAdcFifo;	/* 定义FIFO结构体变量 */

static void AD7606_CtrlLinesConfig(void);
static void AD7606_FSMCConfig(void);

void AD7606DmaRead(u32 par,u32 mar,u16 ndtr)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(DMA_CLK,ENABLE);//DMA2时钟使能 

  DMA_DeInit(DMA_STREAM);
	
	while (DMA_GetCmdStatus(DMA_STREAM) != DISABLE){}//等待DMA可配置 
	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_CHANNEL;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory ;//存储器到存储器
  DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据长度:16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;//存储器数据长度:16位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//高等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA_STREAM, &DMA_InitStructure);//初始化DMA Stream
		
//	DMA_ITConfig(DMA_STREAM, DMA_IT_TC, ENABLE);
//  DMA_ClearFlag(DMA_STREAM, DMA_TCIF);
  DMA_Cmd(DMA_STREAM, ENABLE);
}

void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}	  
/*
*********************************************************************************************************
*	函 数 名: bsp_InitExtSRAM
*	功能说明: 配置连接外部SRAM的GPIO和FSMC
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitAD7606(void)
{
	AD7606_CtrlLinesConfig();
	AD7606_FSMCConfig();

	AD7606_SetOS(AD_OS_NO);		/* 无过采样 */

	AD7606_Reset();

	CONVST_1();					/* 启动转换的GPIO平时设置为高 */
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_CtrlLinesConfig
*	功能说明: 配置LCD控制口线，FSMC管脚设置为复用功能
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
/*
	PD0/FSMC_D2
	PD1/FSMC_D3
	PD4/FSMC_NOE		--- 读控制信号，OE = Output Enable ， N 表示低有效
	PD5/FSMC_NWE		--- 写控制信号，AD7606 只有读，无写信号
	PD8/FSMC_D13
	PD9/FSMC_D14
	PD10/FSMC_D15

	PD14/FSMC_D0
	PD15/FSMC_D1

	PE4/FSMC_A20		--- 和主片选一起译码
	PE5/FSMC_A21		--- 和主片选一起译码
	PE7/FSMC_D4
	PE8/FSMC_D5
	PE9/FSMC_D6
	PE10/FSMC_D7
	PE11/FSMC_D8
	PE12/FSMC_D9
	PE13/FSMC_D10
	PE14/FSMC_D11
	PE15/FSMC_D12

	PG12/FSMC_NE4		--- 主片选（TFT, OLED 和 AD7606）

	其他的控制IO:
	PG2/AD7606_OS0			---> AD7606_OS0		OS2:OS0 选择数字滤波参数
	PG3/AD7606_OS1      ---> AD7606_OS1
	PG4/AD7606_OS2      ---> AD7606_OS2
	PD12/AD7606_CONVST  ---> AD7606_CONVST	启动ADC转换
	PG5/AD7606_RESET    ---> AD7606_RESET	  复位
	PD13/AD7606_BUSY		---> AD7606_BUSY    转换结束的信号

*/
static void AD7606_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能FSMC时钟 */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

	/* 使能 GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG, ENABLE);

	/* 设置 PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
	 PD.10(D15), PD.14(D0), PD.15(D1) 为复用推挽输出 */

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
	                            GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |
	                            GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/*
		PE4/FSMC_A20		--- 和主片选一起译码
		PE5/FSMC_A21		--- 和主片选一起译码

		PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
	 	PE.14(D11), PE.15(D12)
	*/
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5 , GPIO_AF_FSMC);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
	                            GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
	                            GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* 设置 PG12  为复用推挽输出 */
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	/*	配置几个控制用的GPIO
		PG2/AD7606_OS0			---> AD7606_OS0		OS2:OS0 选择数字滤波参数
		PG3/AD7606_OS1      ---> AD7606_OS1
		PG4/AD7606_OS2      ---> AD7606_OS2
		PD12/AD7606_CONVST  ---> AD7606_CONVST	启动ADC转换
		PG5/AD7606_RESET    ---> AD7606_RESET	  复位
		PD13/AD7606_BUSY		---> AD7606_BUSY    转换结束的信号
	*/
	{
		/* 使能 GPIO时钟 */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2| GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
		GPIO_Init(GPIOG, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
	}
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_FSMCConfig
*	功能说明: 配置FSMC并口访问时序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AD7606_FSMCConfig(void)
{
	FSMC_NORSRAMInitTypeDef  init;
	FSMC_NORSRAMTimingInitTypeDef  timing;

	/*
		AD7606规格书要求(3.3V时)：RD读信号低电平脉冲宽度最短21ns，高电平脉冲最短宽度15ns。

		按照如下配置 读数均正常。为了和同BANK的LCD配置相同，选择3-0-6-1-0-0
		3-0-5-1-0-0  : RD高持续75ns， 低电平持续50ns.  1us以内可读取8路样本数据到内存。
		1-0-1-1-0-0  : RD高75ns，低电平执行12ns左右，下降沿差不多也12ns.  数据读取正确。
	*/
	/* FSMC_Bank1_NORSRAM4 configuration */
	timing.FSMC_AddressSetupTime = 3;
	timing.FSMC_AddressHoldTime = 0;
	timing.FSMC_DataSetupTime = 6;
	timing.FSMC_BusTurnAroundDuration = 1;
	timing.FSMC_CLKDivision = 0;
	timing.FSMC_DataLatency = 0;
	timing.FSMC_AccessMode = FSMC_AccessMode_A;

	/*
	 LCD configured as follow:
	    - Data/Address MUX = Disable
	    - Memory Type = SRAM
	    - Data Width = 16bit
	    - Write Operation = Enable
	    - Extended Mode = Enable
	    - Asynchronous Wait = Disable
	*/
	init.FSMC_Bank = FSMC_Bank1_NORSRAM4;
	init.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	init.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	init.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	init.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	init.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	init.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	init.FSMC_WrapMode = FSMC_WrapMode_Disable;
	init.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	init.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	init.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	init.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	init.FSMC_WriteBurst = FSMC_WriteBurst_Disable;

	init.FSMC_ReadWriteTimingStruct = &timing;
	init.FSMC_WriteTimingStruct = &timing;

	FSMC_NORSRAMInit(&init);

	/* - BANK 1 (of NOR/SRAM Bank 1~4) is enabled */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_SetOS
*	功能说明: 配置AD7606数字滤波器，也就设置过采样倍率。
*			 通过设置 AD7606_OS0、OS1、OS2口线的电平组合状态决定过采样倍率。
*			 启动AD转换之后，AD7606内部自动实现剩余样本的采集，然后求平均值输出。
*
*			 过采样倍率越高，转换时间越长。
*			 无过采样时，AD转换时间 4us;
*				2倍过采样时 = 8.7us;
*				4倍过采样时 = 16us
*			 	64倍过采样时 = 286us
*
*	形    参: _ucOS : 过采样倍率
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_SetOS(AD7606_OS_E _ucOS)
{
	g_tAD7606.ucOS = _ucOS;
	switch (_ucOS)
	{
		case AD_OS_X2:
			OS2_0();
			OS1_0();
			OS0_1();
			break;

		case AD_OS_X4:
			OS2_0();
			OS1_1();
			OS0_0();
			break;

		case AD_OS_X8:
			OS2_0();
			OS1_1();
			OS0_1();
			break;

		case AD_OS_X16:
			OS2_1();
			OS1_0();
			OS0_0();
			break;

		case AD_OS_X32:
			OS2_1();
			OS1_0();
			OS0_1();
			break;

		case AD_OS_X64:
			OS2_1();
			OS1_1();
			OS0_0();
			break;

		case AD_OS_NO:
		default:
			g_tAD7606.ucOS = AD_OS_NO;
			OS2_0();
			OS1_0();
			OS0_0();
			break;
	}
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_Reset
*	功能说明: 硬件复位AD7606。复位之后恢复到正常工作状态。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_Reset(void)
{
	RESET_0();	/* 退出复位状态 */

	RESET_1();	/* 进入复位状态 */
	RESET_1();	/* 仅用于延迟。 RESET复位高电平脉冲宽度最小50ns。 */
	RESET_1();
	RESET_1();

	RESET_0();	/* 退出复位状态 */
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_StartConvst
*	功能说明: 启动1次ADC转换
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_StartConvst(void)
{
	/* page 7：  CONVST 高电平脉冲宽度和低电平脉冲宽度最短 25ns */
	/* CONVST平时为高 */
	CONVST_0();
	CONVST_0();
	CONVST_0();
	CONVST_1();
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_ReadNowAdc
*	功能说明: 读取8路采样结果。结果存储在全局变量 g_tAD7606
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_ReadNowAdc(void)
{
//	g_tAD7606.sNowAdc[0] = AD7606_RESULT();	/* 读第1路样本 */
//	g_tAD7606.sNowAdc[1] = AD7606_RESULT();	/* 读第2路样本 */
//	g_tAD7606.sNowAdc[2] = AD7606_RESULT();	/* 读第3路样本 */
//	g_tAD7606.sNowAdc[3] = AD7606_RESULT();	/* 读第4路样本 */
//	g_tAD7606.sNowAdc[4] = AD7606_RESULT();	/* 读第5路样本 */
//	g_tAD7606.sNowAdc[5] = AD7606_RESULT();	/* 读第6路样本 */
//	g_tAD7606.sNowAdc[6] = AD7606_RESULT();	/* 读第7路样本 */
//	g_tAD7606.sNowAdc[7] = AD7606_RESULT();	/* 读第8路样本 */
	if(DMA_GetFlagStatus(DMA_STREAM,DMA_TCIF)!=RESET)//等待DMA2_Steam0传输完
	{
		DMA_ClearFlag(DMA_STREAM,DMA_TCIF);//清除DMA2_Steam7传输完成标志
		MYDMA_Enable(DMA_STREAM,8);	
//		GetADNum++;
	}	
	
}

/*
*********************************************************************************************************
*		下面的函数用于定时采集模式。 TIM4硬件定时中断中读取ADC结果，存在全局FIFO
*********************************************************************************************************
*/
/*
		CONVST 引脚，PD12使用TIM4_CH1输出PWM脉冲，触发AD7606启动ADC转换。
		设置BUSY口线为下降沿中断。在中断服务程序保存ADC结果。
*/
/*
*********************************************************************************************************
*	函 数 名: AD7606_EnterAutoMode
*	功能说明: 配置硬件工作在自动采集模式，结果存储在FIFO缓冲区。
*	形    参：_ulFreq : 采样频率，单位Hz，	1k，2k，5k，10k，20K，50k，100k，200k
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_EnterAutoMode(uint32_t _ulFreq)
{
	/* 配置PD12为复用功能，TIM4_CH1 . 执行后bsp_InitAD7606()对PD12口线的配置将失效 */
	{
		GPIO_InitTypeDef GPIO_InitStructure;

		/* TIM4 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

		/* GPIOD clock enable */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

		/* GPIOD Configuration: PD12  -> TIM4 CH1 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		/* Connect TIM4 pins to AF2 */
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	}

	{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		uint32_t uiTIMxCLK;
		uint16_t usPrescaler;
		uint16_t usPeriod;

		//TIM_DeInit(TIM4);	/* 复位TIM定时器 */

	    /*-----------------------------------------------------------------------
			system_stm32f4xx.c 文件中 void SetSysClock(void) 函数对时钟的配置如下：

			HCLK = SYSCLK / 1     (AHB1Periph)
			PCLK2 = HCLK / 2      (APB2Periph)
			PCLK1 = HCLK / 4      (APB1Periph)

			因为APB1 prescaler != 1, 所以 APB1上的TIMxCLK = PCLK1 x 2 = SystemCoreClock / 2;
			因为APB2 prescaler != 1, 所以 APB2上的TIMxCLK = PCLK2 x 2 = SystemCoreClock;

			APB1 定时器有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM6, TIM12, TIM13,TIM14
			APB2 定时器有 TIM1, TIM8 ,TIM9, TIM10, TIM11
		*/

		uiTIMxCLK = SystemCoreClock / 2;

		if (_ulFreq < 3000)
		{
			usPrescaler = 100 - 1;					/* 分频比 = 10 */
			usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;		/* 自动重装的值 */
		}
		else	/* 大于4K的频率，无需分频 */
		{
			usPrescaler = 0;					/* 分频比 = 1 */
			usPeriod = uiTIMxCLK / _ulFreq - 1;	/* 自动重装的值 */
		}

		/* Time base configuration */

		TIM_TimeBaseStructure.TIM_Period = usPeriod;
		TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_Pulse=usPeriod/2;		
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//		TIM_OCInitStructure.TIM_Pulse = 4;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

		TIM_OC1Init(TIM4, &TIM_OCInitStructure);

		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

		TIM_ARRPreloadConfig(TIM4, ENABLE);

		TIM_Cmd(TIM4, ENABLE);
	}

	/* 配置PD13, BUSY 作为中断输入口，下降沿触发 */
	{
		EXTI_InitTypeDef   EXTI_InitStructure;
		GPIO_InitTypeDef   GPIO_InitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;

		/* Enable GPIOD clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		/* Enable SYSCFG clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

		/* Configure PD13 pin as input floating */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		/* Connect EXTI Line13 to PD13 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource13);

		/* Configure EXTI Line13 */
		EXTI_InitStructure.EXTI_Line = EXTI_Line13;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

		//EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Enable and set EXTI Line13 Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_ISR
*	功能说明: 定时采集中断服务程序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_ISR(void)
{
//	uint8_t i;

	AD7606_ReadNowAdc();

//	for (i = 0; i < 8; i++)
//	{
//		g_tAdcFifo.sBuf[g_tAdcFifo.usWrite] = g_tAD7606.sNowAdc[i];
//		if (++g_tAdcFifo.usWrite >= ADC_FIFO_SIZE)
//		{
//			g_tAdcFifo.usWrite = 0;
//		}
//		if (g_tAdcFifo.usCount < ADC_FIFO_SIZE)
//		{
//			g_tAdcFifo.usCount++;
//		}
//		else
//		{
//			g_tAdcFifo.ucFull = 1;		/* FIFO 满，主程序来不及处理数据 */
//		}
//	}
}

/*
*********************************************************************************************************
*	函 数 名: EXTI15_10_IRQnHandler
*	功能说明: 外部中断服务程序入口。PH12/AD7606_BUSY 下降沿中断触发
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void EXTI15_10_IRQHandler(void)
{
	BaseType_t xDataProcTaskWoken;	
	
	if (EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		AD7606_ISR();
		if(DATAPROTask_Handler!=NULL)//接收到数据，并且接收任务通知的任务有效
		{
			vTaskNotifyGiveFromISR(DATAPROTask_Handler,&xDataProcTaskWoken);//发送任务通知
			portYIELD_FROM_ISR(xDataProcTaskWoken);//如果需要的话进行一次任务切换
		}	
		/* Clear the EXTI line 13 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_HasNewData
*	功能说明: 判断FIFO中是否有新数据
*	形    参：_usReadAdc : 存放ADC结果的变量指针
*	返 回 值: 1 表示有，0表示暂无数据
*********************************************************************************************************
*/
uint8_t AD7606_HasNewData(void)
{
	if (g_tAdcFifo.usCount > 0)
	{
		return 1;
	}
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_FifoFull
*	功能说明: 判断FIFO是否满
*	形    参：_usReadAdc : 存放ADC结果的变量指针
*	返 回 值: 1 表示满，0表示未满
*********************************************************************************************************
*/
uint8_t AD7606_FifoFull(void)
{
	return g_tAdcFifo.ucFull;
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_ReadFifo
*	功能说明: 从FIFO中读取一个ADC值
*	形    参：_usReadAdc : 存放ADC结果的变量指针
*	返 回 值: 1 表示OK，0表示暂无数据
*********************************************************************************************************
*/
uint8_t AD7606_ReadFifo(uint16_t *_usReadAdc)
{
	if (AD7606_HasNewData())
	{
		*_usReadAdc = g_tAdcFifo.sBuf[g_tAdcFifo.usRead];
		if (++g_tAdcFifo.usRead >= ADC_FIFO_SIZE)
		{
			g_tAdcFifo.usRead = 0;
		}

		DISABLE_INT();
		if (g_tAdcFifo.usCount > 0)
		{
			g_tAdcFifo.usCount--;
		}
		ENABLE_INT();
		return 1;
	}
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_StartRecord
*	功能说明: 开始采集
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_StartRecord(uint32_t _ulFreq)
{
	AD7606_StopRecord();

	AD7606_Reset();					/* 复位硬件 */
	AD7606_StartConvst();			/* 启动采样，避免第1组数据全0的问题 */

	g_tAdcFifo.usRead = 0;			/* 必须在开启TIM2之前清0 */
	g_tAdcFifo.usWrite = 0;
	g_tAdcFifo.usCount = 0;
	g_tAdcFifo.ucFull = 0;

	AD7606_EnterAutoMode(_ulFreq);
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_StopRecord
*	功能说明: 停止采集定时器
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_StopRecord(void)
{
	TIM_Cmd(TIM4, DISABLE);

	/* 将PD12 重新配置为普通输出口 */
	{
		GPIO_InitTypeDef GPIO_InitStructure;

		/* 使能 GPIO时钟 */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
	}

	CONVST_1();					/* 启动转换的GPIO平时设置为高 */	\
}



/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
