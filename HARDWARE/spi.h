#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//SPI ��������
////////////////////////////////////////////////////////////////////////////////// 	
 	    													  
void SPI2_Init(void);			 //��ʼ��SPI2��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI2�ٶ�   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI2���߶�дһ���ֽ�
void SPI3_Init(void);			 //��ʼ��SPI3��
void SPI3_SetSpeed(u8 SpeedSet); //����SPI3�ٶ�   
u8 SPI3_ReadWriteByte(u8 TxData);//SPI3���߶�дһ���ֽ�		 
#endif

