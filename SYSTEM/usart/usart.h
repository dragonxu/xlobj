#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
	
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define U3DMA_SEND_SIZE			100	
#define U4DMA_SEND_SIZE			100	 
#define U6DMA_SEND_SIZE			8000	 
#define U6DMA_RECIVE_SIZE		100	 

extern u8  U3TX_BUF[U3DMA_SEND_SIZE];		//串口3DMA发送缓存
extern u8  U4TX_BUF[U4DMA_SEND_SIZE];		//串口4DMA发送缓存
extern u8  U6TX_BUF[U6DMA_SEND_SIZE];		//串口6DMA发送缓存

extern u8  U3RX_BUF[U3DMA_SEND_SIZE];		//串口3接收缓存
extern u8  U4RX_BUF[U4DMA_SEND_SIZE];		//串口4接收缓存
extern u8  U6RX_BUF[U6DMA_RECIVE_SIZE];	//串口6接收缓存
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         				//接收状态标记	
extern u8  DISCmdLen;										//触摸屏返回指令长度
extern u8  DATACmdLen;									//数据传输返回指令长度
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void uart3_init(u32 bound);
void uart4_init(u32 bound);
void uart6_init(u32 bound);
#endif


