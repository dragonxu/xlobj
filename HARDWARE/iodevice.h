#ifndef __IODEVICE_H
#define __IODEVICE_H
#include "sys.h"
//�˿ڶ���
#define LED0 PFout(9)	  // LED1
#define LED1 PFout(10)	// LED2 
#define OUT1 PFout(13)	// OUTPUT1 
#define OUT2 PFout(14)	// OUTPUT2  
#define OUT3 PFout(15)	// OUTPUT3 	 
#define BEEP PAout(8)	  // ����������IO 

void BEEP_Init(void);   //LED��ʼ��		 
void LED_Init(void);    //BEEP��ʼ��
void OUTCTL_Init(void); //IO�����ʼ��
#endif
