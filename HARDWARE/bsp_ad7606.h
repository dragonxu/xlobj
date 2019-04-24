/*
*********************************************************************************************************
*
*	模块名称 : AD7606数据采集模块
*	文件名称 : bsp_ad7606.h
*	版    本 : V1.0
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_AD7606_H
#define _BSP_AD7606_H

#include "sys.h"	 
#include "stdlib.h" 

#define  BUFFER_SIZE     32

#define  DMA_CLK         RCC_AHB1Periph_DMA2
#define  DMA_STREAM      DMA2_Stream0
#define  DMA_TCIF        DMA_FLAG_TCIF0
#define  DMA_IRQN        DMA2_Stream0_IRQn
#define  DMA_CHANNEL     DMA_Channel_0
#define  DMA_IRQ_HANDLER DMA2_Stream0_IRQHandler
#define  SRAM_BANK       0x6C400000
#define  DATA_AREA       (uint32_t)(0x000000)
/* 设置过采样的GPIO: PG2 PG3 PG4 */
#define OS0_1()		GPIOG->BSRRL = GPIO_Pin_2
#define OS0_0()		GPIOG->BSRRH = GPIO_Pin_2
#define OS1_1()		GPIOG->BSRRL = GPIO_Pin_3
#define OS1_0()		GPIOG->BSRRH = GPIO_Pin_3
#define OS2_1()		GPIOG->BSRRL = GPIO_Pin_4
#define OS2_0()		GPIOG->BSRRH = GPIO_Pin_4

/* 启动AD转换的GPIO : PD12   */
#define CONVST_1()	GPIOD->BSRRL = GPIO_Pin_12
#define CONVST_0()	GPIOD->BSRRH = GPIO_Pin_12

/* AD7606复位口线 : PG5  */
#define RESET_1()	GPIOG->BSRRL = GPIO_Pin_5
#define RESET_0()	GPIOG->BSRRH = GPIO_Pin_5

/* AD7606 FSMC总线地址，只能读，无需写 */
#define AD7606_RESULT()	*(__IO uint16_t *)0x6C000000
/* 过采样倍率 */
typedef enum
{
	AD_OS_NO = 0,
	AD_OS_X2 = 1,
	AD_OS_X4 = 2,
	AD_OS_X8 = 3,
	AD_OS_X16 = 4,
	AD_OS_X32 = 5,
	AD_OS_X64 = 6
}AD7606_OS_E;

extern volatile u32 GetADNum;						//AD采样计数器
/* AD数据采集缓冲区 FIFO */
#define ADC_FIFO_SIZE	(8*10)	/* 总体样本数 */

typedef struct
{
	uint8_t ucOS;			/* 过采样倍率，0 - 6. 0表示无过采样 */
	uint8_t ucRange;		/* 输入量程，0表示正负5V, 1表示正负10V */
	int16_t sNowAdc[8];		/* 当前ADC值, 有符号数 */
}AD7606_VAR_T;

typedef struct
{
	/* FIFO 结构 */
	uint16_t usRead;		/* 读指针 */
	uint16_t usWrite;		/* 写指针 */

	uint16_t usCount;		/* 新数据个数 */
	uint8_t ucFull;			/* FIFO满标志 */

	int16_t  sBuf[ADC_FIFO_SIZE];
}AD7606_FIFO_T;

void bsp_InitAD7606(void);
void AD7606_SetOS(AD7606_OS_E _ucOS);
void AD7606_Reset(void);
void AD7606_StartConvst(void);
void AD7606_ReadNowAdc(void);

/* 下面的函数用于FIFO操作模式 */
void AD7606_EnterAutoMode(uint32_t _ulFreq);
void AD7606_StartRecord(uint32_t _ulFreq);
void AD7606_StopRecord(void);
uint8_t AD7606_FifoNewData(void);
uint8_t AD7606_ReadFifo(uint16_t *_usReadAdc);
uint8_t AD7606_FifoFull(void);
void AD7606DmaRead(u32 par,u32 mar,u16 ndtr);
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
/* 全局变量 */
extern AD7606_VAR_T g_tAD7606;
extern AD7606_FIFO_T g_tAdcFifo;

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
