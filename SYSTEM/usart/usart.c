#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用	  
#endif

u8 U3TX_BUF[U3DMA_SEND_SIZE];				//串口3DMA发送缓存
u8 U4TX_BUF[U4DMA_SEND_SIZE];				//串口4DMA发送缓存
u8 U6TX_BUF[U6DMA_SEND_SIZE];				//串口6DMA发送缓存
u8 U3RX_BUF[U3DMA_SEND_SIZE];				//串口3接收缓存
u8 U4RX_BUF[U4DMA_SEND_SIZE];				//串口3接收缓存
u8 U6RX_BUF[U6DMA_RECIVE_SIZE];			//串口6接收缓存
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8 U3Rx_Num = 0;
u8 DISCmdLen = 0;
u8 DATACmdLen = 0;
u16 DEVCmdLen = 0;
u16 USART_RX_STA=0;       					//接收状态标记	


//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
} 


//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	
}

void uart3_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA10复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA11复用为USART3
	
	//USART3端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA10与GPIOA11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9，PA10

   //USART3 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串3
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&USART3->DR);//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)U3TX_BUF;	//串口3DMA发送缓存;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = U3DMA_SEND_SIZE;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);//初始化DMA Stream
	DMA_Cmd(DMA1_Stream3, ENABLE);	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&USART3->DR);//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)U3RX_BUF;	//串口3DMA接收缓存;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = U3DMA_SEND_SIZE;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA1_Stream1, &DMA_InitStructure);//初始化DMA Stream		
	DMA_Cmd(DMA1_Stream1, ENABLE);

  USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送 
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);  //使能串口3的DMA接收
	USART_Cmd(USART3, ENABLE);  //使能串口3 	
	
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//开启相关中断
	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	
}

void uart4_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	DMA_InitTypeDef  DMA_InitStructure;	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA2时钟使能 	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能UART4时钟
 
	//串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10复用为USART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11复用为USART4
	
	//USART4端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PA10，PA11

   //USART4 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(UART4, &USART_InitStructure); //初始化串口1
  /* 配置 DMA Stream Tx*/
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART4->DR);//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)U4TX_BUF;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = U4DMA_SEND_SIZE;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);//初始化DMA Stream
	DMA_Cmd(DMA1_Stream4, ENABLE);
  /* 配置 DMA Stream Rx*/
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART4->DR);//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)U4RX_BUF;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = U4DMA_SEND_SIZE;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA1_Stream2, &DMA_InitStructure);//初始化DMA Stream
	DMA_Cmd(DMA1_Stream2, ENABLE);
	
  USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  //使能串口6的DMA发送 
	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);  //使能串口6的DMA接收
	USART_Cmd(UART4, ENABLE);  //使能串口4
	
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级5
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	
}
void uart6_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOA6复用为USART6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOA7复用为USART6
	
	//USART6端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOC6与GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC6，PC7

   //USART6 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART6, &USART_InitStructure); //初始化串口6

  /* 配置 DMA Stream Tx*/
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)U6TX_BUF;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = U6DMA_SEND_SIZE;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);//初始化DMA Stream
	DMA_Cmd(DMA2_Stream7, ENABLE);
  /* 配置 DMA Stream Rx*/
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)U6RX_BUF;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = U6DMA_RECIVE_SIZE;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);//初始化DMA Stream
	DMA_Cmd(DMA2_Stream1, ENABLE);
	
  USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);  //使能串口6的DMA发送 
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);  //使能串口6的DMA接收
	USART_Cmd(USART6, ENABLE);  //使能串口6
	
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);//开启相关中断

	//Usart6 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口6中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//抢占优先级4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	
}
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
	} 
} 
//注意空闲中断与接收中断有干涉，接收中断有可能会清除空闲中断的标记。在二者同时生效时，空闲中断无法使用
//而数据接收中断工作是正常的，虽没有确凿证据，但是目前的实验结果是这样的，因此改为使用DMA+空闲中断方式
void USART3_IRQHandler(void)
{	
	u8 air;
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  //空闲中断
	{
		air = USART3->SR;
		air = USART3->DR;		//清除IDLE标记位
		DMA_Cmd(DMA1_Stream1, DISABLE);                      //关闭DMA传输 
		DISCmdLen = U3DMA_SEND_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1); //获取接收到的字节数		
		DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);//清除全部标记，否则可能无法正常工作
		DMA1_Stream1->NDTR = U3DMA_SEND_SIZE;
		DMA_Cmd(DMA1_Stream1, ENABLE);                      	//开启DMA传输 
	}
}

void USART6_IRQHandler(void)
{	
	u8 air;
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)  //空闲中断
	{
		air = USART6->SR;
		air = USART6->DR;  //清除IDLE标记位
		DMA_Cmd(DMA2_Stream1, DISABLE);                      //关闭DMA传输 
		DATACmdLen = U6DMA_RECIVE_SIZE - DMA_GetCurrDataCounter(DMA2_Stream1); //获取接收到的字节数		
//		printf("USART6 IDLE ：%d!\r\n",DATACmdLen);	
		DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);//清除全部标记，否则可能无法正常工作
		DMA2_Stream1->NDTR = U6DMA_RECIVE_SIZE;
		DMA_Cmd(DMA2_Stream1, ENABLE);                      	//开启DMA传输 
	}
}

void UART4_IRQHandler(void)
{	
	u8 air;
	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)  //空闲中断
	{
		air = UART4->SR;
		air = UART4->DR;  //清除IDLE标记位
		DMA_Cmd(DMA1_Stream2, DISABLE);                      //关闭DMA传输 
		DEVCmdLen = U4DMA_SEND_SIZE - DMA_GetCurrDataCounter(DMA1_Stream2); //获取接收到的字节数		
		printf("UART4 IDLE ：%d!\r\n",DEVCmdLen);	
		DMA_ClearFlag(DMA1_Stream2,DMA_FLAG_TCIF2 | DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2);//清除全部标记，否则可能无法正常工作
		DMA1_Stream2->NDTR = U4DMA_SEND_SIZE;
		DMA_Cmd(DMA1_Stream2, ENABLE);                      	//开启DMA传输 
	}
}

