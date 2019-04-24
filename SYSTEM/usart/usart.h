#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
	
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define U3DMA_SEND_SIZE			100	
#define U4DMA_SEND_SIZE			100	 
#define U6DMA_SEND_SIZE			8000	 
#define U6DMA_RECIVE_SIZE		100	 

extern u8  U3TX_BUF[U3DMA_SEND_SIZE];		//����3DMA���ͻ���
extern u8  U4TX_BUF[U4DMA_SEND_SIZE];		//����4DMA���ͻ���
extern u8  U6TX_BUF[U6DMA_SEND_SIZE];		//����6DMA���ͻ���

extern u8  U3RX_BUF[U3DMA_SEND_SIZE];		//����3���ջ���
extern u8  U4RX_BUF[U4DMA_SEND_SIZE];		//����4���ջ���
extern u8  U6RX_BUF[U6DMA_RECIVE_SIZE];	//����6���ջ���
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         				//����״̬���	
extern u8  DISCmdLen;										//����������ָ���
extern u8  DATACmdLen;									//���ݴ��䷵��ָ���
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void uart3_init(u32 bound);
void uart4_init(u32 bound);
void uart6_init(u32 bound);
#endif


