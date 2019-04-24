/*
*********************************************************************************************************
*
*	ģ������ : AD7606���ݲɼ�ģ��
*	�ļ����� : bsp_ad7606.h
*	��    �� : V1.0
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
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
/* ���ù�������GPIO: PG2 PG3 PG4 */
#define OS0_1()		GPIOG->BSRRL = GPIO_Pin_2
#define OS0_0()		GPIOG->BSRRH = GPIO_Pin_2
#define OS1_1()		GPIOG->BSRRL = GPIO_Pin_3
#define OS1_0()		GPIOG->BSRRH = GPIO_Pin_3
#define OS2_1()		GPIOG->BSRRL = GPIO_Pin_4
#define OS2_0()		GPIOG->BSRRH = GPIO_Pin_4

/* ����ADת����GPIO : PD12   */
#define CONVST_1()	GPIOD->BSRRL = GPIO_Pin_12
#define CONVST_0()	GPIOD->BSRRH = GPIO_Pin_12

/* AD7606��λ���� : PG5  */
#define RESET_1()	GPIOG->BSRRL = GPIO_Pin_5
#define RESET_0()	GPIOG->BSRRH = GPIO_Pin_5

/* AD7606 FSMC���ߵ�ַ��ֻ�ܶ�������д */
#define AD7606_RESULT()	*(__IO uint16_t *)0x6C000000
/* ���������� */
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

extern volatile u32 GetADNum;						//AD����������
/* AD���ݲɼ������� FIFO */
#define ADC_FIFO_SIZE	(8*10)	/* ���������� */

typedef struct
{
	uint8_t ucOS;			/* ���������ʣ�0 - 6. 0��ʾ�޹����� */
	uint8_t ucRange;		/* �������̣�0��ʾ����5V, 1��ʾ����10V */
	int16_t sNowAdc[8];		/* ��ǰADCֵ, �з����� */
}AD7606_VAR_T;

typedef struct
{
	/* FIFO �ṹ */
	uint16_t usRead;		/* ��ָ�� */
	uint16_t usWrite;		/* дָ�� */

	uint16_t usCount;		/* �����ݸ��� */
	uint8_t ucFull;			/* FIFO����־ */

	int16_t  sBuf[ADC_FIFO_SIZE];
}AD7606_FIFO_T;

void bsp_InitAD7606(void);
void AD7606_SetOS(AD7606_OS_E _ucOS);
void AD7606_Reset(void);
void AD7606_StartConvst(void);
void AD7606_ReadNowAdc(void);

/* ����ĺ�������FIFO����ģʽ */
void AD7606_EnterAutoMode(uint32_t _ulFreq);
void AD7606_StartRecord(uint32_t _ulFreq);
void AD7606_StopRecord(void);
uint8_t AD7606_FifoNewData(void);
uint8_t AD7606_ReadFifo(uint16_t *_usReadAdc);
uint8_t AD7606_FifoFull(void);
void AD7606DmaRead(u32 par,u32 mar,u16 ndtr);
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
/* ȫ�ֱ��� */
extern AD7606_VAR_T g_tAD7606;
extern AD7606_FIFO_T g_tAdcFifo;

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
