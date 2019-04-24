#ifndef __LAN8720_H
#define __LAN8720_H
#include "sys.h"
#include "stm32f4x7_eth.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
//////////////////////////////////////////////////////////////////////////////////	 
// STM32F407开发板
//LAN8720 驱动代码	   							  
////////////////////////////////////////////////////////////////////////////////// 

/* 接受等待时间，不是MAX就是定期查询，是MAX就是等中断 */
#define TIME_WAITING_FOR_INPUT 	( portMAX_DELAY )
/* 网络协议占有堆栈大小 */
#define INTERFACE_THREAD_STACK_SIZE 	( 350 ) 
/* 用于接收处理上下文的信号量 */
extern SemaphoreHandle_t s_xSemaphore;

#define LAN8720_PHY_ADDRESS  	0x00				//LAN8720 PHY芯片地址.
#define LAN8720_RST 		   	PDout(3) 			//LAN8720复位引脚	 

extern ETH_DMADESCTypeDef *DMARxDscrTab;			//以太网DMA接收描述符数据结构体指针
extern ETH_DMADESCTypeDef *DMATxDscrTab;			//以太网DMA发送描述符数据结构体指针 
extern uint8_t *Rx_Buff; 							//以太网底层驱动接收buffers指针 
extern uint8_t *Tx_Buff; 							//以太网底层驱动发送buffers指针
extern ETH_DMADESCTypeDef  *DMATxDescToSet;			//DMA发送描述符追踪指针
extern ETH_DMADESCTypeDef  *DMARxDescToGet; 		//DMA接收描述符追踪指针 
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;	//DMA最后接收到的帧信息指针
 

u8 LAN8720_Init(void);
u8 LAN8720_Get_Speed(void);
u8 ETH_MACDMA_Config(void);
FrameTypeDef ETH_Rx_Packet(void);
u8 ETH_Tx_Packet(u16 FrameLength);
u32 ETH_GetCurrentTxBuffer(void);
u8 ETH_Mem_Malloc(void);
void ETH_Mem_Free(void);
#endif 

