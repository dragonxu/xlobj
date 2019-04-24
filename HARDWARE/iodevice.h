#ifndef __IODEVICE_H
#define __IODEVICE_H
#include "sys.h"
//端口定义
#define LED0 PFout(9)	  // LED1
#define LED1 PFout(10)	// LED2 
#define OUT1 PFout(13)	// OUTPUT1 
#define OUT2 PFout(14)	// OUTPUT2  
#define OUT3 PFout(15)	// OUTPUT3 	 
#define BEEP PAout(8)	  // 蜂鸣器控制IO 

void BEEP_Init(void);   //LED初始化		 
void LED_Init(void);    //BEEP初始化
void OUTCTL_Init(void); //IO输出初始化
#endif
