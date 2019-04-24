#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "iodevice.h"
#include "bsp_ad7606.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "rtc.h"
#include "ds18b20.h"
#include "spi.h"
#include "w25qxx.h" 
#include "malloc.h" 
#include "ff.h"
#include "exfuns.h"
#include "usbh_usr.h" 
#include "lwip_comm.h"
#include "LAN8720.h"
#include "lwipopts.h"
#include "timer.h"
#include "string.h"
/********************************任务描述**********************************
***FreeRTOS优先级数字越大，优先级越高
***start_task           开始任务
***DataProc_task				数据处理任务
***DisPlay_task					显示任务
***DisPlay_task					显示任务
***DataInfo_task				数据信息任务
***Other_task						其它工作任务
***SysInfo_task					系统信息任务
***HeapSize_task				堆栈检查任务
**************************************************************************/

//任务优先级
#define START_TASK_PRIO		configMAX_PRIORITIES - 2
//任务堆栈大小	
#define START_STK_SIZE 		128  
//任务句柄
TaskHandle_t STARTTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define DATAPRO_TASK_PRIO		17
//任务堆栈大小	
#define DATAPRO_STK_SIZE 		128  
//任务句柄
TaskHandle_t DATAPROTask_Handler;
//任务函数
void DataPro_task(void *pvParameters);

//任务优先级
#define DISPLAY_TASK_PRIO		10
//任务堆栈大小	
#define DISPLAY_STK_SIZE 		128  
//任务句柄
TaskHandle_t DISPLAYTask_Handler;
//任务函数
void DisPlay_task(void *pvParameters);

//任务优先级
#define DATAINFO_TASK_PRIO		9
//任务堆栈大小	
#define DATAINFO_STK_SIZE 		128
//任务句柄
TaskHandle_t DATAINFOTask_Handler;
//任务函数
void DataInfo_task(void *pvParameters);

//任务优先级
#define OTHER_TASK_PRIO		8
//任务堆栈大小	
#define OTHER_STK_SIZE 		128
//任务句柄
TaskHandle_t OTHERTask_Handler;
//任务函数
void Other_task(void *pvParameters);

//任务优先级
#define SYSINFO_TASK_PRIO		7
//任务堆栈大小
#define SYSINFO_STK_SIZE 		256
//任务句柄
TaskHandle_t SYSINFOTask_Handler;
//任务函数
void SysInfo_task(void *pvParameters);

//任务优先级
#define HEAPSIZE_TASK_PRIO		6
//任务堆栈大小
#define HEAPSIZE_STK_SIZE 		300
//任务句柄
TaskHandle_t HEAPSIZETask_Handler;
//任务函数
void HeapSize_task(void *pvParameters);

RTC_TimeTypeDef RTC_TimeStruct;
RTC_DateTypeDef RTC_DateStruct;
RTC_TimeTypeDef RTC_TimeSet;
RTC_DateTypeDef RTC_DateSet;
SYS_PARA	Sys_ParaMeter;
volatile SEND_LIST SendData_List[3];
u8	SysParaArr[200];
char InfoBuffer[1000];										//保存信息的数组	
short Temp;																//温度	
u16 FlashID;															//FlashID
u8 *test;
u8 ChoseChannel = 0;
u8 FindUSBDeviceFlag = 0;
u8 DataUpdataOver = 0;
u8 LvBoNum=0;
u8 ErrFlagF=0;
u8 ErrFlagH=0;
volatile u8 FWaitSendList = 0;													//等待发送队列序号 分闸
volatile u8 HWaitSendList = 0;													//等待发送队列序号 合闸
u8 DeviceStatus[3];																			//设备状态
u8 FileBuf[2000]; 		 																	//定义文件缓存数组
u16 DemaData = 2000;
char *TelPhone = "029-88326721";
const u8 SPassWord[7] = {'8','1','2','4','3','8','8',};
const u8 SoftVer[3] = {1,0,15};
s32 BDBuf[6];
s32 ZeroBuf[6];
s16 LvBoBuf[6][32];
s32 LoBoValue[6];
u16 CatchMenXian = 100; 																//设置捕获门限
u16 WenDingMenXian = 100; 															//设置稳定门限
u16 WaitSendNum[3];																			//等待发送缓存数量 分闸
u16 HWaitSendNum;																				//等待发送缓存数量 合闸
volatile s16 GetBuf[BUFFER_SIZE];												//原始AD值
s16 DataBuf[BUFFER_SIZE];																//滤波后的力值
s16 PDataBuf[BUFFER_SIZE];															//未滤波的力值
s16 SpeedBuf[BUFFER_SIZE];															//速度缓存
u32 FLASH_SIZE=16*1024*1024;									 					//FLASH 大小为16M字节	
u32 MCUID[3];	
USBH_HOST 					 USB_Host;	
USB_OTG_CORE_HANDLE  USB_OTG_Core;

u16 PICNO=0;	//界面序号
__align(32) u8 FileSaveBuf[4000] __attribute__((at(0X1000D000))); 		 	//定义文件缓存数组到RAM3
__align(32) s16 FCatchSaveBuf[3][4000] __attribute__((at(0X10003000))); //定义捕获数组到RAM3分闸
__align(32) s16 HCatchSaveBuf[4000] 	 __attribute__((at(0X10009000))); //定义捕获数组到RAM3合闸
__align(32) s16 HCatchBuf[2][2000] 		 __attribute__((at(0X1000B000))); 	//合闸弹簧缓存数组
s16 FCatchBuf[2][2000]; 																									//分闸弹簧缓存数组
void DisValue(u16 PicNo,u16 CtrNo,s32 data,u8 Dbit);	  //显示数字，支持正负数，5位以内小数
void MainDis1(void);																		//主页1显示函数
void MainDis2(void);																		//主页2显示函数	
void DemaDis(void);																			//标定页面显示函数	
void ParaDis(void);																			//参数页面显示
void TimeDis(void);																			//时钟页面显示
void SaveDis(void);																			//存储页面显示
void CommDis(void);																			//通信页面显示
void HelpDis(void);																			//帮助页面显示
void SoftVerDis(u16 PicNo,u16 CtrNo);										//版本号显示函数
void GetPicNo(void);																		//获取当前页面编号	
void ChangePic(u16 PicNo);															//切换页面
void StrDis(u16 PicNo,u16 CtrNo,char * str);						//显示字符串
void DefautSet(void);																		//恢复默认参数
void SysParaToStruct(u8 * paraArr);											//系统参数写入到结构体
void SysParaToArr(u8 * paraArr);												//系统参数写入到数组
void cpuidGetId(void);																	//获取MCUID
void ADDataSend(void);																	//AD转换后数据发送
void FCatchDataSend(void);															//捕获到的数据发送 分闸
void HCatchDataSend(void);															//捕获到的数据发送 合闸

int main(void)
{
	u8 i,res;
 	u32 total,free;		
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(168);															//延时初始化 
	uart_init(115200);														//串口1初始化波特率为115200
	uart3_init(115200);														//串口2初始化波特率为115200
  uart4_init(115200);														//串口4初始化波特率为115200
	uart6_init(115200);														//串口6初始化波特率为115200
	delay_xms(500);
	ChangePic(0);																	//切换到首页
	delay_xms(500);
	SoftVerDis(0,2);															//显示软件版本
	delay_xms(500);	
	LED_Init();		  															//初始化与LED连接的硬件接口 
	BEEP_Init();																	//初始化与BEEP连接的硬件接口 
	OUTCTL_Init();																//初始化继电器控制的硬件接口 
	My_RTC_Init();																//初始化内部RTC
	cpuidGetId();																	//读取MCUID
	printf("CPUID 0x:%x %x %x\r\n",MCUID[0],MCUID[1],MCUID[2]);
	StrDis(0,1,"系统初始化中...");	
	delay_xms(500);	
	W25QXX_Init();																//W25Q128初始化	
//	TIM3_Int_Init(999,839); 										//100khz的频率,计数1000为10ms			
 	if(DS18B20_Init())														//DS18B20初始化	
	{
		printf("DS18B20 Error!\r\n");
		delay_ms(200);
	} 
	FlashID  = W25QXX_ReadID();										//获取FLASH芯片ID
	if(FlashID!=W25Q128)													//检测不到W25Q128
	{
		printf("W25Q128 Check Failed!\r\n");
		StrDis(0,1,"SPI-Flash 初始化失败！");	
		delay_xms(500);			
	}
	else
	{
		printf("Flash ID: %x!\r\n",FlashID);
		StrDis(0,1,"SPI-Flash 初始化成功！");	
		delay_xms(500);			
	}
	my_mem_init(SRAMIN);													//初始化内部内存池
	my_mem_init(SRAMCCM);													//初始化CCM内存池
 	exfuns_init();																//为fatfs相关变量申请内存				 
 	res=f_mount(fs[1],"1:",1); 										//挂载FLASH.	
	if(res==0X0D)																	//FLASH磁盘,FAT文件系统错误,重新格式化FLASH
	{
		printf("Flash Disk Formatting...\r\n");			//格式化FLASH
		StrDis(0,1,"Flash Disk格式化！");	
		delay_xms(500);				
		res=f_mkfs("1:",1,4096);										//格式化FLASH,1,盘符;1,不需要引导区,8个扇区为1个簇
		if(res==0)
		{
			f_setlabel((const TCHAR *)"1:sela");			//设置Flash磁盘的名字为：sela
			printf("Flash Disk Format Finish\r\n");		//格式化完成	
			StrDis(0,1,"Flash Disk格式化成功！");	
			delay_xms(500);						
		}
		else
		{
			printf("Flash Disk Format Error \r\n");	//格式化失败
			StrDis(0,1,"Flash Disk格式化失败！");			
		}			
		delay_ms(500);
	}	
	else
	{
		printf("FLASH Fatfs挂载代码：%d\r\n",res);	//FLASH挂载代码
		StrDis(0,1,"FATFS初始化成功！");	
		delay_xms(500);				
	}
	W25QXX_Read(SysParaArr,FLASH_SIZE-200,200);
	if(SysParaArr[0] != 2) //非第一次使用
	{
		printf("Flash 参数初始化！\r\n");
		DefautSet();
		SysParaToArr(SysParaArr);
		W25QXX_Write(SysParaArr,FLASH_SIZE-200,200);
	}
	else
	{
		printf("Flash 参数读取！\r\n");
		SysParaToStruct(SysParaArr);
		if(Sys_ParaMeter.UartDebugFlag == 1)//允许打印测试信息		
		{
			for(i=0;i<140;i++)
				printf("Para%d:%d\r\n",i,SysParaArr[i]);		
		}
	}
	f_mount(fs[2],"2:",1); 												//挂载U盘	
	bsp_InitAD7606();															//配置AD7606所用的GPIO
	AD7606_SetOS(AD_OS_X4);												//4倍过采样设置
	AD7606_StartConvst();													//启动1次AD转换
	printf("\r\nAD7606进入FIFO工作模式 (50KHz 8通道同步采集)...\r\n");
	AD7606_StartRecord(10000);										//启动10kHz采样速率
	AD7606DmaRead(SRAM_BANK,(u32)GetBuf,BUFFER_SIZE);	//启动一次DMA传输
  USBH_Init(&USB_OTG_Core,USB_OTG_FS_CORE_ID,&USB_Host,&USBH_MSC_cb,&USR_Callbacks);  	//初始化USB主机
	for(i=0;i<6;i++)
	{
		LED0=!LED0;
		BEEP=!BEEP;	
		delay_ms(200);
	}
	StrDis(0,1,"创建进程...");	
	delay_xms(500);			
	
	//创建开始任务
	xTaskCreate((TaskFunction_t )start_task,            //任务函数
							(const char*    )"start_task",          //任务名称
							(uint16_t       )START_STK_SIZE,        //任务堆栈大小
							(void*          )NULL,                  //传递给任务函数的参数
							(UBaseType_t    )START_TASK_PRIO,       //任务优先级
							(TaskHandle_t*  )&STARTTask_Handler);   //任务句柄              
	vTaskStartScheduler();          //开启任务调度
}
//开始任务任务函数
void start_task(void *pvParameters)
{
		u8 res;
	
    taskENTER_CRITICAL();           //进入临界区
		//创建数据处理任务
    xTaskCreate((TaskFunction_t )DataPro_task,     	
                (const char*    )"DataPro_task",   	
                (uint16_t       )DATAPRO_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )DATAPRO_TASK_PRIO,	
                (TaskHandle_t*  )&DATAPROTask_Handler);   
    //创建显示任务
    xTaskCreate((TaskFunction_t )DisPlay_task,     
                (const char*    )"DisPlay_task",   
                (uint16_t       )DISPLAY_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )DISPLAY_TASK_PRIO,
                (TaskHandle_t*  )&DISPLAYTask_Handler);        
    //创建数据信息任务
    xTaskCreate((TaskFunction_t )DataInfo_task,     
                (const char*    )"DataInfo_task",   
                (uint16_t       )DATAINFO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )DATAINFO_TASK_PRIO,
                (TaskHandle_t*  )&DATAINFOTask_Handler);  
		//创建杂项任务						
		xTaskCreate((TaskFunction_t )Other_task,     
                (const char*    )"Other_task",   
                (uint16_t       )OTHER_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )OTHER_TASK_PRIO,
                (TaskHandle_t*  )&OTHERTask_Handler); 
		//创建系统信息任务						
		xTaskCreate((TaskFunction_t )SysInfo_task,     
                (const char*    )"SysInfo_task",   
                (uint16_t       )SYSINFO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )SYSINFO_TASK_PRIO,
                (TaskHandle_t*  )&SYSINFOTask_Handler); 
		//创建系统信息任务						
		xTaskCreate((TaskFunction_t )HeapSize_task,     
                (const char*    )"HeapSize_task",   
                (uint16_t       )HEAPSIZE_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )HEAPSIZE_TASK_PRIO,
                (TaskHandle_t*  )&HEAPSIZETask_Handler); 
		//目前该函数会创建两个进程，其中包括LWIP线程以及数据接收线程
		//因为对FreeRots及LWIP理解不够透彻的缘故，如果在中断中进行接收数据处理，会产生一些错误虽然不影响使用。
		if(Sys_ParaMeter.NETFlag == 1)
		{
			StrDis(0,1,"请稍等，LWIP初始化...");	
			delay_ms(200);		
			res=lwip_comm_init();						//初始化及创建ETH任务
			if(res) 		//lwip初始化
			{
				printf("LWIP Init Falied! %d\r\n",res);
				StrDis(0,1,"LWIP初始化失败...");	
				delay_ms(200);				
				if(res == 3)
				{
					printf("要使用NET服务请插入网线重启系统!\r\n");				
				}
			}
			else				//必须所有相关内容初始化成功之后才能开启相关线程
			{

				if((Sys_ParaMeter.DHCPFlag==1)&&(LWIP_DHCP))			//如果在开始任务之前创建该线程，因为优先级的问题，可能会导致一些错误。
				{
					printf("DHCP_Config.....\r\n");
					MyDhcp_Creat();				
				}
				else
				{
					printf("静态IP，TCP服务器.....\r\n");
					MyTCPS_Creat();					
				}
			}			
		}
		else
		{
			StrDis(0,1,"跳过LWIP初始化...");	
			delay_ms(200);		
		}
	
		printf("系统初始化完成！\r\n");
    vTaskDelete(STARTTask_Handler); //删除开始任务				
    taskEXIT_CRITICAL();            //退出临界区
}

//数据处理任务 
void DataPro_task(void *pvParameters)
{
	u16 i,j;
	u8 StartNum=0;
	u8 EndFlag1=0,EndFlag2=0;
	u8 CatchFlag[2] = {0};				//捕获开启、关闭标记
	u8 SteaDataNum=0;							//缓冲数据计数			
	u8 ChannelStatus[4] = {0};		//通道状态稳态非稳态判断。每两个为一组，例0、1代表通道1稳态与非稳态统计数据
	u32 NotifyValue=0;						//任务通知返回值
	s16 GetSteaBuf[2][32];				//每个通道最近32个缓冲数据
	s16 MaxValueBuf[2] = {0};			//捕获数据中的最大值
	s16 MinValueBuf[2] = {0};			//捕获数据中的最小值
	u16 MaxCatchnum[2] = {0};			//捕获数据中的最大值位置
	u16 MinCatchnum[2] = {0};			//捕获数据中的最小值位置
	u16 CatchNum[2] = {0};				//捕获数据个数
	u16 CatchChannelFlag = 0;			//捕获数据标识，发送数据使用
	u16 FWaitSendFlag = 0;
	u16 HWaitSendFlag = 0;
	while(1)
	{
		NotifyValue=ulTaskNotifyTake(pdTRUE,1000);	//获取任务通知
		if(StartNum < 64)
			StartNum++;
		//32位滑动滤波
		for(i=0;i<2;i++)
		{
			LvBoBuf[i][LvBoNum] = GetBuf[i];
			LoBoValue[i] = 0;
			for(j=0;j<32;j++)
			{
				LoBoValue[i] += LvBoBuf[i][j];
			}	
			LoBoValue[i] = LoBoValue[i]/32;			
		}
		LvBoNum++;
		LvBoNum = (LvBoNum > 31) ? 0:LvBoNum;
		//去零位，数据换算
		for(i=0;i<2;i++)
		{
			DataBuf[i] = LoBoValue[i] - Sys_ParaMeter.ZeroPara[i];  
			PDataBuf[i] = GetBuf[i] - Sys_ParaMeter.ZeroPara[i]; 
			if(GetBuf[i] > 0)
			{
				DataBuf[i] = (DataBuf[i]*Sys_ParaMeter.DemaPara[i])/32768;
				PDataBuf[i] = (PDataBuf[i]*Sys_ParaMeter.DemaPara[i])/32768;
			}
			else
			{
				DataBuf[i] = 0- DataBuf[i];
				DataBuf[i] = (DataBuf[i]*Sys_ParaMeter.DemaPara[i])/32768;
				DataBuf[i] = 0- DataBuf[i];
				
				PDataBuf[i] = 0- PDataBuf[i];
				PDataBuf[i] = (PDataBuf[i]*Sys_ParaMeter.DemaPara[i])/32768;
				PDataBuf[i] = 0- PDataBuf[i];				
			}		
		}
		//获取每一相当前状态

		CatchMenXian = Sys_ParaMeter.ReviseValue[6]; //暂时使用补偿参数6来修改捕获门限
		if((CatchMenXian < 50)||(CatchMenXian > 500))
		{
			CatchMenXian = 100;	
			Sys_ParaMeter.ReviseValue[6] = 100;			
		}
		WenDingMenXian = Sys_ParaMeter.ReviseValue[5]; //暂时使用补偿参数6来修改捕获门限
		if((WenDingMenXian < 50)||(WenDingMenXian > 500))
		{
			WenDingMenXian = 100;	
			Sys_ParaMeter.ReviseValue[5] = 100;			
		}
		
		if(StartNum >= 64)  //超过64点才开始进入工作
		{
			if(SteaDataNum < 32) //缓存当前数据的前32个点,未滤波数据
			{
				for(i=0;i<2;i++)
					GetSteaBuf[i][SteaDataNum] = PDataBuf[i];
				SteaDataNum++;
			}
			else
			{
				for(i=0;i<32;i++)
				{
					GetSteaBuf[0][i] = GetSteaBuf[0][i+1];
					GetSteaBuf[1][i] = GetSteaBuf[1][i+1];
				}
				GetSteaBuf[0][31] = PDataBuf[0];	
				GetSteaBuf[1][31] = PDataBuf[1];	
			}
			//稳态统计 A相分闸 1 ChannelStatus[0]非稳态计数，ChannelStatus[1]稳态计数器
			//突变判断，当原始数据振幅超过32滑动滤波值300kg认为触发，包含原始数据大于滤波后数据+300，或者原始数据小于滤波后数据-300
			if((PDataBuf[0] > (DataBuf[0] + CatchMenXian))||(PDataBuf[0] < (DataBuf[0] - CatchMenXian))) //突变条件
			{
				ChannelStatus[0]++; 
				if(ChannelStatus[0] >= 10) //变化连续超过10次，打破稳态
				{
					ChannelStatus[1] = 0;
					if(ChannelStatus[0] == 10) //给最大值最小值初值
					{
						MaxValueBuf[0] = DataBuf[0];
						MinValueBuf[0] = DataBuf[0];
					}
					CatchFlag[0] = 1;
				}
				if(ChannelStatus[0] > 200)
					ChannelStatus[0] = 50;				
			}	
			//else //前版本直接使用else，判断范围是相同的。。此处想减少触发几率的同时也同时降低判稳的几率。。
			//修订---在触发条件不满足时，另设条件判稳，稳定条件为振幅50kg以内
			else if((PDataBuf[0] < (DataBuf[0]+WenDingMenXian))&&(PDataBuf[0] > (DataBuf[0]-WenDingMenXian)))
			{
				ChannelStatus[1]++;				
				if(ChannelStatus[1] >= 100)//持续100次，恢复稳态
				{
					if(CatchNum[0] >= 1000)  //一次触发最少采集500个数据才能终止
						ChannelStatus[0] = 0;			
				}
				if(ChannelStatus[1] > 200)
					ChannelStatus[1] = 100;					
			}
			//稳态统计 A相合闸 2		----暂时屏蔽掉合闸弹簧捕获	
//			if((PDataBuf[1] > (DataBuf[1] + CatchMenXian))||(PDataBuf[1] < (DataBuf[1] - CatchMenXian)))
//			{
//				ChannelStatus[2]++;
//				if(ChannelStatus[2] >= 10) //变化连续超过10次，打破稳态
//				{
//					ChannelStatus[3] = 0;
//					if(ChannelStatus[2] > 200)
//						ChannelStatus[2] = 50;
//					CatchFlag[1] = 1;
//				}				
//			}	
//			else
//			{
//				ChannelStatus[3]++;
//				if(ChannelStatus[3] >= 50)//持续50次，恢复稳态
//				{
//					if(CatchNum[1] >= 500)  //一次触发最少采集500个数据才能终止
//						ChannelStatus[2] = 0;			
//				}
//				if(ChannelStatus[3] > 200)
//					ChannelStatus[3] = 50;					
//			}
			//稳态获取设备状态，非稳态缓存突变数据 A相分闸传感器 ------------------------A1
			if(ChannelStatus[0] >= 10) //缓存突变数据
			{
				if(FWaitSendFlag > 0) //新数据到来，老数据还未发送，直接放弃
				{
					CatchNum[0] = 0;
					CatchFlag[0] = 0;	
					FWaitSendFlag = 0;
				}
				if(CatchNum[0]<32) //插入前32个缓存数据
				{
					for(i=0;i<32;i++)
					{
						FCatchBuf[0][CatchNum[0]] = GetSteaBuf[0][i];
						FCatchBuf[1][CatchNum[0]] = GetSteaBuf[1][i];
						CatchNum[0]++;
					}
				}
				else if(CatchNum[0]<1980) //只能缓存1980个数据
				{
					FCatchBuf[0][CatchNum[0]] = PDataBuf[0];
					FCatchBuf[1][CatchNum[0]] = PDataBuf[1];
					CatchNum[0]++;					
				}		
			}
			else if(ChannelStatus[1] >= 100) //稳态，只在稳态判断设备状态
			{
				EndFlag1 = 0;
				EndFlag2 = 0;
				for(i=0;i<CatchNum[0];i++) //寻找关键数据点位置
				{
					//寻找两个通道捕获数据的最大值
					if(MaxValueBuf[0] < FCatchBuf[0][i])
					{
						MaxValueBuf[0] = FCatchBuf[0][i];
						MaxCatchnum[0] = i;
					}
					if(MaxValueBuf[1] < FCatchBuf[1][i])
					{
						MaxValueBuf[1] = FCatchBuf[1][i];
						MaxCatchnum[1] = i;
					}
					//通道1小于预设分闸值之后不再处理
					if((FCatchBuf[0][i] < Sys_ParaMeter.DefaultData[0])&&(EndFlag1 == 0))
					{
						EndFlag1 = 1;	
					}

					//通道2小于预设未储能值之后不再处理
					if((FCatchBuf[1][i] < Sys_ParaMeter.DefaultData[2])&&(EndFlag2 == 0))
					{
						MinValueBuf[1] = FCatchBuf[1][i];
						MinCatchnum[1] = i;
						EndFlag2 = 1;
					}
					//查找最小值
					if((MinValueBuf[0] > FCatchBuf[0][i])&&(EndFlag1 == 0))
					{
						MinValueBuf[0] = FCatchBuf[0][i];
						MinCatchnum[0]=i;
					}
					if((MinValueBuf[1] > (FCatchBuf[1][i]-200))&&(EndFlag2 == 0))
					{
						MinValueBuf[1] = FCatchBuf[1][i];
						MinCatchnum[1] = i;
					}
					
				}
				if((CatchFlag[0] == 1)&&(CatchNum[0] > 50))//假如捕获标记为1 且 捕获点数大于50，认为有效
				{
					if(FCatchBuf[0][CatchNum[0]-1] > (FCatchBuf[0][0]+200))				//上升突变
					{
						SpeedBuf[0] = (MinCatchnum[1])/10; //10k速度，依据点数获取时间
						printf("Speed1:%d \r\n",SpeedBuf[0]);
						CatchChannelFlag = 0xA1A1;	//A相分闸上升突变标识
					}
					else if((FCatchBuf[0][CatchNum[0]-1]+200) < FCatchBuf[0][0]) //下降突变
					{
						SpeedBuf[1] = (MinCatchnum[0]-MaxCatchnum[0])/10;
						printf("Speed2:%d \r\n",SpeedBuf[1]);
						CatchChannelFlag = 0xA2A2;	//A相分闸下降突变标识
					}
					else
					{
						SpeedBuf[0] = MinCatchnum[0]/10;
						printf("Speed3:%d \r\n",SpeedBuf[0]);
						CatchChannelFlag = 0xA3A3;	//A相分闸其他突变标识
					}
					//将有效的数据在允许的情况下存储到队列中
					if(FWaitSendList < 3) //列表中有空位时，进行填充，否则丢弃
					{						
						FCatchSaveBuf[2 - FWaitSendList][0] = 0xFFFF;
						FCatchSaveBuf[2 - FWaitSendList][1] = 0xAAAA;
						FCatchSaveBuf[2 - FWaitSendList][2] = CatchChannelFlag;
						for(i=3;i<CatchNum[0]+3;i++)
						{
							FCatchSaveBuf[2 - FWaitSendList][2*i - 3] =  FCatchBuf[0][i-3];
							FCatchSaveBuf[2 - FWaitSendList][2*i - 2] =  FCatchBuf[1][i-3];								
						}		
						CatchNum[0] = CatchNum[0]*2;						
						FCatchSaveBuf[2 - FWaitSendList][CatchNum[0]+3] = 0xF8F8;
						FCatchSaveBuf[2 - FWaitSendList][CatchNum[0]+4] = 0xF8F8;
						FCatchSaveBuf[2 - FWaitSendList][CatchNum[0]+5] = 0xF8F8;
						FCatchSaveBuf[2 - FWaitSendList][CatchNum[0]+6] = 0xF8F8;
						FCatchSaveBuf[2 - FWaitSendList][CatchNum[0]+7] = 0xF0F0;
						//WaitSendNum[2 - FWaitSendList] = CatchNum[0]+8;
						SendData_List[2-FWaitSendList].DataNum = 	CatchNum[0]+8;		
						SendData_List[2-FWaitSendList].WeiZhi = FCatchSaveBuf[2 - FWaitSendList];						
						
						FWaitSendList++;
												
						//清除捕获使用的变量
						CatchNum[0] = 0;
						CatchFlag[0] = 0;	
						FWaitSendFlag = 0;
					}
					else  	//待发送队列已满
					{
						//printf("Wait:%d \r\n",FWaitSendFlag);
						FWaitSendFlag++;
						if(FWaitSendFlag >= 10000) //最多等待1S,超时后清除数据
						{
							//清除捕获使用的变量
							CatchNum[0] = 0;
							CatchFlag[0] = 0;
							FWaitSendFlag = 0;
						}
					}
				}
				else //无效数据清除相关参数
				{
					//清除捕获使用的变量
					CatchNum[0] = 0;
					CatchFlag[0] = 0;
					FWaitSendFlag = 0;						
				}
				MinCatchnum[0] = 0;
				MaxCatchnum[0] = 0;
				MaxValueBuf[0] = 0;
				MinValueBuf[0] = 0;	

				//设备状态判断
				if((DataBuf[0] > (Sys_ParaMeter.DefaultData[1]-Sys_ParaMeter.SafeThreshold[0]))&&
					(DataBuf[0] < (Sys_ParaMeter.DefaultData[1]+Sys_ParaMeter.SafeThreshold[0])))//在合闸安全区域内		
					DeviceStatus[0] = 1;
				else if((DataBuf[0] > (Sys_ParaMeter.DefaultData[0]-Sys_ParaMeter.SafeThreshold[0]))&&
					(DataBuf[0] < (Sys_ParaMeter.DefaultData[0]+Sys_ParaMeter.SafeThreshold[0])))//在分闸安全区域内
					DeviceStatus[0] = 2;
				else //危险状态
					DeviceStatus[0] = 3;

				if((DataBuf[1] > (Sys_ParaMeter.DefaultData[3]-Sys_ParaMeter.SafeThreshold[1]))&&
					(DataBuf[1] < (Sys_ParaMeter.DefaultData[3]+Sys_ParaMeter.SafeThreshold[1])))//安全储能区域内				
					DeviceStatus[1] = 1;
				else if((DataBuf[1] > (Sys_ParaMeter.DefaultData[2]-Sys_ParaMeter.SafeThreshold[1]))&&
					(DataBuf[1] < (Sys_ParaMeter.DefaultData[2]+Sys_ParaMeter.SafeThreshold[1])))//安全未储能区域内
					DeviceStatus[1] = 2;
				else										//自由状态或者故障
					DeviceStatus[1] = 3;
					
					
			}
			//稳态获取设备状态，非稳态缓存突变数据 A相合闸传感器 ------------A2
			//暂时屏蔽到合闸
//			if(ChannelStatus[2] >= 10)
//			{
//				if(HWaitSendFlag > 0)//有数据未发送，清除信息
//				{
//					CatchNum[1] = 0;
//					CatchFlag[1] = 0;
//					HWaitSendFlag = 0;							
//				}
//				if(CatchNum[1]<20) //插入前20个缓存数据
//				{
//					for(i=0;i<20;i++)
//					{
//						HCatchBuf[0][CatchNum[1]] = GetSteaBuf[0][i];		
//						HCatchBuf[1][CatchNum[1]] = GetSteaBuf[1][i];	
//						CatchNum[1]++;							
//					}
//				}
//				else if(CatchNum[1]<1980) //只能缓存1980个数据
//				{
//					HCatchBuf[0][CatchNum[1]] = PDataBuf[0];	
//					HCatchBuf[1][CatchNum[1]] = PDataBuf[1];	

//					//获取最大值及其位置
//					if(MaxValueBuf[1] < HCatchBuf[0][CatchNum[1]])
//					{
//						MaxValueBuf[1] = HCatchBuf[0][CatchNum[1]];
//						MaxCatchnum[1] = CatchNum[1];
//					}
//					//获取最小值及其位置,不计算负值
//					if((MinValueBuf[1] > HCatchBuf[0][CatchNum[1]])&&(HCatchBuf[0][CatchNum[1]] >= 0))
//					{
//						MinValueBuf[1] = HCatchBuf[0][CatchNum[1]];
//						MinCatchnum[1] = CatchNum[1];
//					}	
//					CatchNum[1]++;					
//				}			
//			}
//			else if(ChannelStatus[3] >= 20) //稳态，只在稳态判断设备状态
//			{
//				if((CatchFlag[1] == 1)&&(CatchNum[1] > 50))//假如捕获标记为1 且 捕获点数大于50，认为有效
//				{
//					if(HCatchBuf[0][CatchNum[1]-1] > (HCatchBuf[0][0]+200))				//上升突变
//					{
//						SpeedBuf[1] = MaxCatchnum[1]/10; //10k速度，依据点数获取时间
//						CatchChannelFlag = 0xA4A4;	//A相合闸上升突变标识
//					}
//					else if((HCatchBuf[0][CatchNum[1]-1]+200) < HCatchBuf[0][0]) //下降突变
//					{
//						SpeedBuf[1] = MinCatchnum[1]/10;
//						CatchChannelFlag = 0xA5A5;	//A相合闸下降突变标识
//					}
//					else
//					{
//						SpeedBuf[1] = CatchNum[1]/10;
//						CatchChannelFlag = 0xA6A6;	//A相合闸其他突变标识
//					}	
//					if(HWaitSendList == 0) //合闸待发送队列为空
//					{
//						HCatchSaveBuf[0] = 0xFFFF;
//						HCatchSaveBuf[1] = 0xAAAA;
//						HCatchSaveBuf[2] = CatchChannelFlag;	
//						for(i=3;i<CatchNum[1]+3;i++)
//						{
//							HCatchSaveBuf[2*i - 3] = HCatchBuf[0][i-3];
//							HCatchSaveBuf[2*i - 2] = HCatchBuf[1][i-3];
//						}
//						CatchNum[1] = CatchNum[1]*2;
//						HCatchSaveBuf[CatchNum[1]+3] = 0xF8F8;
//						HCatchSaveBuf[CatchNum[1]+4] = 0xF8F8;	
//						HCatchSaveBuf[CatchNum[1]+5] = 0xF8F8;
//						HCatchSaveBuf[CatchNum[1]+6] = 0xF8F8;	
//						HCatchSaveBuf[CatchNum[1]+7] = 0xF8F8;								
//						HWaitSendNum = CatchNum[1]+8;	
//						HWaitSendList = 1;
//						//清除捕获使用的变量
//						CatchNum[1] = 0;
//						CatchFlag[1] = 0;
//						HWaitSendFlag = 0;							
//					}
//					else
//					{
//						HWaitSendFlag++;
//						if(HWaitSendFlag >= 10000) //等待时间超过1S，清除数据
//						{
//							//清除捕获使用的变量
//							CatchNum[1] = 0;
//							CatchFlag[1] = 0;
//							HWaitSendFlag = 0;							
//						}
//					}
//				}
//				else //无效数据清除
//				{
//					//清除捕获使用的变量
//					CatchNum[1] = 0;
//					CatchFlag[1] = 0;
//					HWaitSendFlag = 0;					
//				}
//				MinCatchnum[1] = 0;
//				MaxCatchnum[1] = 0;
//				MaxValueBuf[1] = 0;
//				MinValueBuf[1] = 0;						
//			}	
		}
		GetADNum++;		
	}
}
//显示任务
void DisPlay_task(void *pvParameters)
{		
		s32 CheckNum = 0;
		delay_ms(500);		
		StrDis(0,1,"完成，进入系统...");	
		delay_ms(500);		
		ChangePic(1);										//切换到主页
		PICNO = 1;
		delay_ms(500);			
		while(1)
		{	
			CheckNum++;
			switch(PICNO)
			{
				case 1:		//主页1
					MainDis1();
					break;
				case 2:		//主页2
					MainDis2();
					break;
				case 4:		//通信页面
					CommDis();
					break;
				case 5:		//标定页面
					DemaDis();
					break;
				case 6:		//参数页面
					ParaDis();
					break;
				case 7:		//时钟页面
					TimeDis();
					break;
				case 9:		//存储页面
					SaveDis();
					break;
				case 10:	//帮助页面
					HelpDis();
					break;
				default:
					break;
			}
			delay_ms(5);	
			if(CheckNum%20 == 0) //十秒获取一次当前页面
				GetPicNo();
			vTaskDelay(500); 
		}
}
//交互信息任务
void DataInfo_task(void *pvParameters)
{
		u8 i,j;
		u8 PsNo=0;
		u8 PWDCheck=0,PWDCheck1=0,PWDCheck2=0;
		u8 PWDBuf1[7],PWDBuf2[7];//密码缓存
		char DisInfo[100];
		u8 DGetBuf[40];
		u8 DGetBufNum=0;
		while(1)
		{	
			if(DISCmdLen>0)//接收到串口屏指令
			{
				for(i=8;i<DISCmdLen-5;i++) //获取接收的数据内容从第八位开始,DISCmdLen-5结束
				{
					DGetBuf[DGetBufNum++] = U3RX_BUF[i];
				}
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x02))//主页2--------------2
				{
					if(U3RX_BUF[6] == 0x03)  //增加行程
					{
						if(Sys_ParaMeter.RunLenth < 5000)
							Sys_ParaMeter.RunLenth++;					
					}
					if(U3RX_BUF[6] == 0x04)  //减少行程
					{
						if(Sys_ParaMeter.RunLenth>10)
							Sys_ParaMeter.RunLenth--;							
					}			
				}
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x03))//导航页-------------3
				{
					if((U3RX_BUF[6] == 0x08)&&(U3RX_BUF[8] == 0x00))//取消保存
					{
						ChangePic(1);
						PICNO = 1;
						delay_ms(5);
					}
					if((U3RX_BUF[6] == 0x08)&&(U3RX_BUF[8] == 0x01))//保存参数
					{
						BEEP = !BEEP;
						SysParaToArr(SysParaArr);
						W25QXX_Write(SysParaArr,FLASH_SIZE-200,200);
						delay_ms(200);
						BEEP = !BEEP;
						ChangePic(1);
						PICNO = 1;
						delay_ms(5);
					}
				}
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x04))//通信页------------4
				{
					switch(U3RX_BUF[6])
					{
						case 6:	//DHCP开关
							if(U3RX_BUF[8] == 0)
								Sys_ParaMeter.DHCPFlag = 1;
							else
								Sys_ParaMeter.DHCPFlag = 0;
							break;
						case 7: //辅助串口开关
							if(U3RX_BUF[8] == 0)
								Sys_ParaMeter.UartDebugFlag = 1;
							else
								Sys_ParaMeter.UartDebugFlag = 0;
							break;
						case 10://规约转换开关
							if(U3RX_BUF[8] == 0)
								Sys_ParaMeter.ProtocolChange	= 1;
							else
								Sys_ParaMeter.ProtocolChange = 0;
							break;
						case 13://网路功能开关
							if(U3RX_BUF[8] == 0)
								Sys_ParaMeter.NETFlag	= 1;
							else
								Sys_ParaMeter.NETFlag = 0;
							break;
						case 17://网络端口
							Sys_ParaMeter.NetPort = 0;
							for(i=0;i<DGetBufNum;i++)
							{
								Sys_ParaMeter.NetPort = Sys_ParaMeter.NetPort*10+(GetBuf[i]-0x30);								
							}
							if(Sys_ParaMeter.UartDebugFlag == 1)
								printf("NetPort:%d\r\n",Sys_ParaMeter.NetPort);
							break;
						case 14://IP地址
							j=0;
							Sys_ParaMeter.IPAddress[j] = 0;
							for(i=0;i<DGetBufNum;i++)
							{
								if(DGetBuf[i] == '.')//分割点
								{
									Sys_ParaMeter.IPAddress[++j] = 0;									
								}
								else
								{
									Sys_ParaMeter.IPAddress[j] = Sys_ParaMeter.IPAddress[j]*10+(DGetBuf[i]-0x30);
								}
							}
							if(Sys_ParaMeter.UartDebugFlag == 1)
								printf("IPAddress: %d.%d.%d.%d\r\n",Sys_ParaMeter.IPAddress[0],Sys_ParaMeter.IPAddress[1],Sys_ParaMeter.IPAddress[2],Sys_ParaMeter.IPAddress[3]);
							break;
						case 15://子网掩码
							j=0;
							Sys_ParaMeter.NetMask[j] = 0;
							for(i=0;i<DGetBufNum;i++)
							{
								if(DGetBuf[i] == '.')//分割点
								{
									Sys_ParaMeter.NetMask[++j] = 0;									
								}
								else
								{
									Sys_ParaMeter.NetMask[j] = Sys_ParaMeter.NetMask[j]*10+(DGetBuf[i]-0x30);
								}
							}
							if(Sys_ParaMeter.UartDebugFlag == 1)
								printf("IPAddress: %d.%d.%d.%d\r\n",Sys_ParaMeter.NetMask[0],Sys_ParaMeter.NetMask[1],Sys_ParaMeter.NetMask[2],Sys_ParaMeter.NetMask[3]);
							break;
						case 16://网关地址
							j=0;
							Sys_ParaMeter.GateWay[j] = 0;
							for(i=0;i<DGetBufNum;i++)
							{
								if(DGetBuf[i] == '.')//分割点
								{
									Sys_ParaMeter.GateWay[++j] = 0;									
								}
								else
								{
									Sys_ParaMeter.GateWay[j] = Sys_ParaMeter.GateWay[j]*10+(DGetBuf[i]-0x30);
								}
							}
							if(Sys_ParaMeter.UartDebugFlag == 1)
								printf("IPAddress: %d.%d.%d.%d\r\n",Sys_ParaMeter.GateWay[0],Sys_ParaMeter.GateWay[1],Sys_ParaMeter.GateWay[2],Sys_ParaMeter.GateWay[3]);
							break;							
						default:
							break;
					}
				}
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x05))//标定页------------5
				{
					if(U3RX_BUF[6] == 2) //目标值
					{
						DemaData = 0;
						for(i=0;i<DGetBufNum;i++)
						{
							DemaData = DemaData*10+(DGetBuf[i]-0x30);								
						}					
					}
					if(U3RX_BUF[6] == 15) //置零操作
					{
						BEEP = !BEEP;
						if((ChoseChannel > 0)&&(ChoseChannel<7))
							;
						else
							ChoseChannel = 1;
						ZeroBuf[ChoseChannel-1] = 0;
						for(i=0;i<10;i++)
						{
							ZeroBuf[ChoseChannel-1] += LoBoValue[ChoseChannel-1];
							delay_ms(10);
						}
						Sys_ParaMeter.ZeroPara[ChoseChannel-1] = ZeroBuf[ChoseChannel-1]/10;
						BEEP = !BEEP;
					}
					if(U3RX_BUF[6] == 16) //标定
					{
						BEEP = !BEEP;
						if((ChoseChannel > 0)&&(ChoseChannel<7))
							;
						else
							ChoseChannel = 1;
						BDBuf[ChoseChannel-1] = 0;
						for(i=0;i<10;i++)
						{
							BDBuf[ChoseChannel-1] += LoBoValue[ChoseChannel-1];
							delay_ms(10);
						}	
						BDBuf[ChoseChannel-1] = BDBuf[ChoseChannel-1]/10;
						Sys_ParaMeter.DemaPara[ChoseChannel-1] = (DemaData*32768)/(BDBuf[ChoseChannel-1] - Sys_ParaMeter.ZeroPara[ChoseChannel-1]);
						BEEP = !BEEP;
					}		
					if((U3RX_BUF[6]>2)&&(U3RX_BUF[6]<5))//标定系数输入
					{
						Sys_ParaMeter.DemaPara[U3RX_BUF[6]-3] = 0;
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.DemaPara[U3RX_BUF[6]-3] = Sys_ParaMeter.DemaPara[U3RX_BUF[6]-3]*10+(DGetBuf[i]-0x30);								
						}							
					}
					if((U3RX_BUF[6]>18)&&(U3RX_BUF[6]<21))//通道选择
					{
						ChoseChannel = U3RX_BUF[6] - 18;
					}
					//存储初始值
					if((U3RX_BUF[6] >= 6)&&(U3RX_BUF[6] <= 8))
					{
						Sys_ParaMeter.DefaultData[U3RX_BUF[6]-6] = 0;
						if(DGetBuf[0] == '-')
						{
							for(i=1;i<DGetBufNum;i++)
							{
								Sys_ParaMeter.DefaultData[U3RX_BUF[6]-6] = Sys_ParaMeter.DefaultData[U3RX_BUF[6]-6]*10+(DGetBuf[i]-0x30);								
							}		
							Sys_ParaMeter.DefaultData[U3RX_BUF[6]-6] = 0-Sys_ParaMeter.DefaultData[U3RX_BUF[6]-6];							
						}						
						else
						{
							for(i=0;i<DGetBufNum;i++)
							{
								Sys_ParaMeter.DefaultData[U3RX_BUF[6]-6] = Sys_ParaMeter.DefaultData[U3RX_BUF[6]-6]*10+(DGetBuf[i]-0x30);								
							}							
						}						
					}
					if(U3RX_BUF[6] == 11)
					{
						Sys_ParaMeter.DefaultData[3] = 0;
						if(DGetBuf[0] == '-')
						{
							for(i=1;i<DGetBufNum;i++)
							{
								Sys_ParaMeter.DefaultData[3] = Sys_ParaMeter.DefaultData[3]*10+(DGetBuf[i]-0x30);								
							}	
							Sys_ParaMeter.DefaultData[3] = 0 - Sys_ParaMeter.DefaultData[3];
						}
						else
						{
							for(i=0;i<DGetBufNum;i++)
							{
								Sys_ParaMeter.DefaultData[3] = Sys_ParaMeter.DefaultData[3]*10+(DGetBuf[i]-0x30);								
							}	
						}						
					}				
				}
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x06))//参数页-------------6
				{				
					if(U3RX_BUF[6] == 25) //相位设置
					{
							Sys_ParaMeter.ChannelConfig = U3RX_BUF[8];
					}
					if(U3RX_BUF[6] == 3) //温度补偿开关
					{
						Sys_ParaMeter.ReviseValue[4]= 0; //补偿参数4作为弹簧K值1
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.ReviseValue[4] = Sys_ParaMeter.ReviseValue[4]*10+(DGetBuf[i]-0x30);								
						}	
					}						
					if(U3RX_BUF[6] == 4) //蠕变检测开关
					{
						Sys_ParaMeter.ReviseValue[3]= 0; //补偿参数3作为弹簧K值2
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.ReviseValue[3] = Sys_ParaMeter.ReviseValue[3]*10+(DGetBuf[i]-0x30);								
						}	
					}	
					
					if(U3RX_BUF[6] == 6) //触发门限
					{
						Sys_ParaMeter.ReviseValue[6]= 0; //补偿参数6作为触发门限
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.ReviseValue[6] = Sys_ParaMeter.ReviseValue[6]*10+(DGetBuf[i]-0x30);								
						}						
					}
					if(U3RX_BUF[6] == 7) //稳定门限
					{
						Sys_ParaMeter.ReviseValue[5]= 0; //补偿参数5作为稳定门限
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.ReviseValue[5] = Sys_ParaMeter.ReviseValue[5]*10+(DGetBuf[i]-0x30);								
						}						
					}					
					
					if(U3RX_BUF[6] == 8) //速度补偿
					{
						Sys_ParaMeter.ReviseValue[7]= 0; //补偿参数7做为速度补偿
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.ReviseValue[7] = Sys_ParaMeter.ReviseValue[7]*10+(DGetBuf[i]-0x30);								
						}						
					}
					if(U3RX_BUF[6] == 16)//工作模式
						Sys_ParaMeter.WorkMode = U3RX_BUF[8];
					if(U3RX_BUF[6] == 22)//无线开关
					{
						if(U3RX_BUF[8] == 0)
							Sys_ParaMeter.WireLessFlag = 1;
						else
							Sys_ParaMeter.WireLessFlag = 0;
					}
					if(U3RX_BUF[6] == 20)//串口速度
						Sys_ParaMeter.UartSpeed = U3RX_BUF[8];
					if(U3RX_BUF[6] == 21)//温度补偿
					{
						if(U3RX_BUF[8] == 0)
							Sys_ParaMeter.TempComp = 1;
						else
							Sys_ParaMeter.TempComp = 0;
					}					
					if(U3RX_BUF[6] == 13) //拉力阈值 safe0
					{
						Sys_ParaMeter.SafeThreshold[0] = 0; 
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.SafeThreshold[0] = Sys_ParaMeter.SafeThreshold[0]*10+(DGetBuf[i]-0x30);								
						}						
					}
					if(U3RX_BUF[6] == 14) //压力阈值 safe1
					{
						Sys_ParaMeter.SafeThreshold[1] = 0; 
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.SafeThreshold[1] = Sys_ParaMeter.SafeThreshold[1]*10+(DGetBuf[i]-0x30);								
						}						
					}	
					if(U3RX_BUF[6] == 23) //开关行程
					{
						Sys_ParaMeter.RunLenth= 0; 
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.RunLenth = Sys_ParaMeter.RunLenth*10+(DGetBuf[i]-0x30);								
						}						
					}
          if(U3RX_BUF[6] == 24)	//分度值
					{
						Sys_ParaMeter.FenDu= 0; 
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.FenDu = Sys_ParaMeter.FenDu*10+(DGetBuf[i]-0x30);								
						}	

						if((Sys_ParaMeter.FenDu != 1)&&(Sys_ParaMeter.FenDu != 2))
							Sys_ParaMeter.FenDu = (Sys_ParaMeter.FenDu/5)*5;
						if(Sys_ParaMeter.FenDu < 1)
							Sys_ParaMeter.FenDu = 1;
					}
				}
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x07))//时钟页------------7
				{
					if(U3RX_BUF[6] == 3) //时钟输入	
					{
						RTC_DateSet.RTC_Year = (DGetBuf[2]-0x30)*10+(DGetBuf[3]-0x30);
						RTC_DateSet.RTC_Month = (DGetBuf[4]-0x30)*10+(DGetBuf[5]-0x30);
						RTC_DateSet.RTC_Date = (DGetBuf[6]-0x30)*10+(DGetBuf[7]-0x30);
						
						RTC_TimeSet.RTC_Hours = (DGetBuf[8]-0x30)*10+(DGetBuf[9]-0x30);
						RTC_TimeSet.RTC_Minutes = (DGetBuf[10]-0x30)*10+(DGetBuf[11]-0x30);
						RTC_TimeSet.RTC_Seconds = (DGetBuf[12]-0x30)*10+(DGetBuf[13]-0x30);
						
						RTC_SetDate(RTC_Format_BIN,&RTC_DateSet);
						RTC_SetTime(RTC_Format_BIN,&RTC_TimeSet);
					}
					if(U3RX_BUF[6] == 4) //快速同步
					{
						;
					}
					if(U3RX_BUF[6] == 5) //远程同步
					{
						;
					}						
					if(U3RX_BUF[6] == 6) //定时同步	
					{
						;
					}						
				}			
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x08))//密码页-----------8
				{
					if(U3RX_BUF[6] == 2) //旧密码
					{
						for(i=0;i<DGetBufNum;i++)
						{
								if((DGetBuf[i] == (Sys_ParaMeter.PassWord[i]+0x30))||(DGetBuf[i] == SPassWord[i]))
									PsNo++;
						}
						if((DGetBufNum < 7)||(PsNo<7))
						{
							StrDis(8,7,"密码错误！");
							PWDCheck1 = 0;
						}
						else if(PsNo == 7)
						{
							StrDis(8,7,"密码正确！");
							PWDCheck1 = 1;
						}
						PsNo = 0;
					}
					if(U3RX_BUF[6] == 3) //新密码1
					{
						for(i=0;i<DGetBufNum;i++)
						{
							PWDBuf1[i] = DGetBuf[i];
						}	
						if(DGetBufNum != 7)
						{
							StrDis(8,7,"密码长度错误！");
							PWDCheck2=0;
						}
						else
							PWDCheck2=1;
					}
					if(U3RX_BUF[6] == 4) //新密码2
					{
						for(i=0;i<DGetBufNum;i++)
						{
							PWDBuf2[i] = DGetBuf[i];
						}	
						if(DGetBufNum != 7)
						{
							StrDis(8,7,"密码长度错误！");
							PWDCheck2=0;							
						}
						else
							PWDCheck2=1;
					}	
					if(U3RX_BUF[6] == 5) //忘记密码
					{
						sprintf(DisInfo,"加密内码：%x-%x",MCUID[1],MCUID[2]);
						StrDis(8,7,DisInfo);						
					}
					if(U3RX_BUF[6] == 6) //确认修改
					{
						if((PWDCheck1 == 1)&&(PWDCheck2 == 1))
						{
							for(i=0;i<7;i++)
							{
								if(PWDBuf1[i] == PWDBuf2[i])
									PsNo++;
							}
							if(PsNo == 7)
							{
								for(i=0;i<7;i++)
									Sys_ParaMeter.PassWord[i] = PWDBuf1[i] - 0x30;	
								StrDis(8,7,"密码修改完成！");	
							}
							else
								StrDis(8,7,"输入密码不一致！");	
						}
						else
						{
							if(PWDCheck1 == 0)
								StrDis(8,7,"原密码错误！");	
						}
					}	
					delay_ms(5);					
				}			
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x09))//存储页--------------9
				{
					if(U3RX_BUF[6] == 3) //导出信息
					{
						DataUpdataOver = 1;
						StrDis(9,7,"开始导出数据...");	
					}
					if(U3RX_BUF[6] == 6) //捕获开关
					{
						if(U3RX_BUF[8] == 0)
							Sys_ParaMeter.CatchFlag = 1;
						else
							Sys_ParaMeter.CatchFlag = 0;
					}
					if(U3RX_BUF[6] == 9) //恢复出厂设置
					{
						if(U3RX_BUF[8] == 0)
							Sys_ParaMeter.FirstFlag = 1;
						else
							Sys_ParaMeter.FirstFlag = 2; //为2时不恢复数据
					}					
					delay_ms(5);
				}						
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x0B))//密码校验页----------11
				{
					if(U3RX_BUF[6] == 0x04) //输入密码
					{
						for(i=0;i<7;i++)
						{
							if((U3RX_BUF[8+i] == (Sys_ParaMeter.PassWord[i]+0x30))||(U3RX_BUF[8+i] == SPassWord[i]))//核对密码
								PsNo++;
							if(Sys_ParaMeter.UartDebugFlag == 1)//允许打印测试信息
								printf("ps%d:%d\r\n",i,U3RX_BUF[8+i]);
						}
						if(PsNo == 7)//密码正确
						{
							if(Sys_ParaMeter.UartDebugFlag == 1)//允许打印测试信息
								printf("密码正确！\r\n");
							StrDis(11,5,"密码正确...");
							PWDCheck = 1;		
							delay_ms(5);							
						}
						else
						{
							if(Sys_ParaMeter.UartDebugFlag == 1)//允许打印测试信息
								printf("密码错误！\r\n");
							StrDis(11,5,"密码错误...");
							PWDCheck = 0;							
							delay_ms(5);
						}
						PsNo=0;	
					}			
					if(U3RX_BUF[6] == 0x02) //确认
					{
						if(PWDCheck == 1) //密码正确
						{
							delay_ms(5);		
							ChangePic(3);	
							delay_ms(5);							
						}
						PWDCheck = 0;
					}						
					if(U3RX_BUF[6] == 0x03) //忘记密码
					{
						sprintf(DisInfo,"加密内码：%x-%x",MCUID[1],MCUID[2]);
						StrDis(11,5,DisInfo);
						delay_ms(5);
					}
				}
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[2] == 0x01)&&(DISCmdLen == 9))
				{
					PICNO = U3RX_BUF[3]<<8|U3RX_BUF[4];
					if(Sys_ParaMeter.UartDebugFlag == 1)//允许打印测试信息
						printf("当前页面：%d\r\n",PICNO);
				}
				DISCmdLen = 0;
				DGetBufNum = 0; //清零获取数据个数
			}			
			//OUT1=!OUT1;
			//OUT2=!OUT2;
			//OUT3=!OUT3;
			vTaskDelay(100); 
		}
}
void Other_task(void *pvParameters)
{
		u8 *p=0,i;
		p=mymalloc(SRAMIN,1024);//申请2K字节	
		while(1)
		{
			if(p!=NULL)sprintf((char*)p,"Memory Malloc Test%03d",i);//向p写入一些内容		
			test = p;
			USBH_Process(&USB_OTG_Core, &USB_Host);
			delay_ms(1);
			i++;
			vTaskDelay(200); 
		}
}
//系统信息任务
void SysInfo_task(void *pvParameters)
{
	u32 SysRunNum = 0;
	while(1)
	{
		if(DataUpdataOver == 1)
			DataUpdataOver = 2;
		if(FWaitSendList > 0)	//分闸待发送队列不为空
		{
			FCatchDataSend();		//分闸捕获数据发送		
		}
		else if(HWaitSendList >0)	//合闸待发送队列不为空
		{
			HCatchDataSend();		//合闸捕获数据发送		
		}
		if((FWaitSendList == 0)&&(HWaitSendList == 0)) //没有待发送的捕获数据
		{
			if((SysRunNum%5) == 0)
				ADDataSend();				 //发送一次力值数据	
		}
		if((SysRunNum%50) == 0) //5秒执行一次		
		{
			taskENTER_CRITICAL();           //进入临界区
			Temp=DS18B20_Get_Temp();	
			taskEXIT_CRITICAL();            //退出临界区				
			if(Sys_ParaMeter.UartDebugFlag == 1)//允许打印测试信息
				printf("AD转换计数:%d/秒--捕获门限：%d -- 稳定门限：%d\r\n",GetADNum/5,CatchMenXian,WenDingMenXian);
			GetADNum = 0;			
		}
		if((SysRunNum%10) == 0)
			LED0=!LED0;//闪烁LED,提示系统正在运行.			
		SysRunNum++;
		if(SysRunNum > 200)
			SysRunNum = 1;
		vTaskDelay(100);   //循环时间为100ms。
	}
}
//**HeapSize_task			堆栈检查任务
void HeapSize_task(void *pvParameters)
{
	u8 tbuf[40];
	while(1)
	{
			tcp_server_flag |= LWIP_SEND_DATA; //标记LWIP有数据要发送
			RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);			
			sprintf((char*)tbuf,"Time:%02d:%02d:%02d",RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds); 
//			printf("%s\r\n",tbuf);
			RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);			
			sprintf((char*)tbuf,"Date:20%02d-%02d-%02d",RTC_DateStruct.RTC_Year,RTC_DateStruct.RTC_Month,RTC_DateStruct.RTC_Date); 
//			printf("%s\r\n",tbuf);

//			vTaskList(InfoBuffer);								//获取所有任务的信息
//			printf("任务名\t任务状态\t优先级\t剩余堆栈\t任务序号\r\n");
//			printf("%s\r\n",InfoBuffer);					//通过串口打印所有任务的信息
			
     vTaskDelay(1000);                     //延时1s，也就是1000个时钟节拍	
	}
}  
///////////////////////////////////////////
//数字显示格式化
//PicNo:界面编号
//CtrNo：控件编号
//data：显示数据
//Dbit: 小数点位置 
///////////////////////////////////////////
void DisValue(u16 PicNo,u16 CtrNo,s32 data,u8 Dbit)
{
	u8 DisValArr[20],bitnum=0,i,ZeroFlag=0,Indno=0;
	DisValArr[bitnum++]=0xEE;
	DisValArr[bitnum++]=0xB1;
	DisValArr[bitnum++]=0x10;	
	DisValArr[bitnum++]=PicNo>>8;
	DisValArr[bitnum++]=PicNo;	
	DisValArr[bitnum++]=CtrNo>>8;
	DisValArr[bitnum++]=CtrNo;	
	if(data<0)//负数增加负号
	{
		DisValArr[bitnum++]='-';	
		data = 0-data;
	}
	if(data>9999999)//大于9999999的数据不显示
		DisValArr[bitnum++]='F';
	else
	{
		if(Dbit > 5)//最大支持5位小数
			Dbit = 0;	
		if((data/1000000) > 0)
		{
			DisValArr[bitnum++]=(data%10000000)/1000000+0x30;
			ZeroFlag++;	
		}
		if((data/100000) > 0)
		{
			DisValArr[bitnum++]=(data%1000000)/100000+0x30;	
			ZeroFlag++;	
		}
		if((Dbit == 5)&&(Indno == 0))
		{
			if(ZeroFlag != 0)
			{
				 DisValArr[bitnum++]='.';
			}
			else
			{
				 DisValArr[bitnum++]='0';
				 DisValArr[bitnum++]='.';				
			}
			Indno = Dbit;
		}		
		if((data/10000) > 0)
		{
			DisValArr[bitnum++]=(data%100000)/10000+0x30;	
			ZeroFlag++;			
		}
		else
		{
			if(Indno !=0 )
				DisValArr[bitnum++]='0';
		}		
		if((Dbit == 4)&&(Indno == 0))
		{
			if(ZeroFlag != 0)
			{
				 DisValArr[bitnum++]='.';
			}
			else
			{
				 DisValArr[bitnum++]='0';
				 DisValArr[bitnum++]='.';				
			}
			Indno = Dbit;
		}				
		if((data/1000) > 0)
		{
			DisValArr[bitnum++]=(data%10000)/1000+0x30;	
			ZeroFlag++;			
		}
		else
		{
			if(Indno !=0 )
				DisValArr[bitnum++]='0';
		}		
		if((Dbit == 3)&&(Indno == 0))
		{
			if(ZeroFlag != 0)
			{
				 DisValArr[bitnum++]='.';
			}
			else
			{
				 DisValArr[bitnum++]='0';
				 DisValArr[bitnum++]='.';				
			}
			Indno = Dbit;
		}			
		if((data/100) > 0)
		{
			DisValArr[bitnum++]=(data%1000)/100+0x30;	
			ZeroFlag++;	
		}
		else
		{
			if(Indno !=0 )
				DisValArr[bitnum++]='0';
		}		
		if((Dbit == 2)&&(Indno == 0))
		{
			if(ZeroFlag != 0)
			{
				 DisValArr[bitnum++]='.';
			}
			else
			{
				 DisValArr[bitnum++]='0';
				 DisValArr[bitnum++]='.';				
			}
			Indno = Dbit;
		}			
		if((data/10) > 0)
		{
			DisValArr[bitnum++]=(data%100)/10+0x30;
			ZeroFlag++;			
		}
		else
		{
			if(Indno !=0 )
				DisValArr[bitnum++]='0';
		}
		if((Dbit == 1)&&(Indno == 0))
		{
			if(ZeroFlag != 0)
			{
				 DisValArr[bitnum++]='.';
			}
			else
			{
				 DisValArr[bitnum++]='0';
				 DisValArr[bitnum++]='.';				
			}
			Indno = Dbit;
		}			
		DisValArr[bitnum++]=data%10+0x30;
	}
	DisValArr[bitnum++]=0xFF;
	DisValArr[bitnum++]=0xFC;
	DisValArr[bitnum++]=0xFF;
	DisValArr[bitnum++]=0xFF;	


	for(i=0;i<bitnum;i++)
	{
			U3TX_BUF[i] = DisValArr[i];
	}
	while(1)
	{
		if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//等待DMA1_Steam3传输完成
		{
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA1_Steam3传输完成标志		
			break;
		}		
	}
	MYDMA_Enable(DMA1_Stream3,bitnum); 
//	if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//等待DMA1_Steam4传输完成
//		DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);//清除DMA1_Steam4传输完成标志	
//	MYDMA_Enable(DMA1_Stream4,bitnum); 
//	for(i=0;i<bitnum;i++)
//	{
//			USART_SendData(USART3, DisValArr[i]);         					//向串口3发送数据
//			while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);	//等待发送结束		
//	}
}

///////////////////////////////////////////
//字符串显示
//PicNo:界面编号
//CtrNo：控件编号
//str：显示字符串
///////////////////////////////////////////
void StrDis(u16 PicNo,u16 CtrNo,char * str)
{
	u8 DisStrArr[100],bitnum=0,i,StrSize=0;
	StrSize = strlen(str);
	DisStrArr[bitnum++]=0xEE;
	DisStrArr[bitnum++]=0xB1;
	DisStrArr[bitnum++]=0x10;	
	DisStrArr[bitnum++]=PicNo>>8;
	DisStrArr[bitnum++]=PicNo;	
	DisStrArr[bitnum++]=CtrNo>>8;
	DisStrArr[bitnum++]=CtrNo;
	for(i=0;i<StrSize;i++)
			DisStrArr[bitnum++]=str[i];
	DisStrArr[bitnum++]=0xFF;
	DisStrArr[bitnum++]=0xFC;
	DisStrArr[bitnum++]=0xFF;
	DisStrArr[bitnum++]=0xFF;	
	for(i=0;i<bitnum;i++)
		U3TX_BUF[i] = DisStrArr[i];
	while(1)
	{
		if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//等待DMA1_Steam3传输完成
		{
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA1_Steam3传输完成标志		
			break;
		}		
	}	
	MYDMA_Enable(DMA1_Stream3,bitnum);	
}
///////////////////////////////////////////
//软件版本号显示格式化
//PicNo:界面编号
//CtrNo：控件编号
//data：显示数据
///////////////////////////////////////////
void SoftVerDis(u16 PicNo,u16 CtrNo)
{
	u8 DisValArr[20],bitnum=0,i;
	DisValArr[bitnum++]=0xEE;
	DisValArr[bitnum++]=0xB1;
	DisValArr[bitnum++]=0x10;	
	DisValArr[bitnum++]=PicNo>>8;
	DisValArr[bitnum++]=PicNo;	
	DisValArr[bitnum++]=CtrNo>>8;
	DisValArr[bitnum++]=CtrNo;
	DisValArr[bitnum++]='V';
	DisValArr[bitnum++]='e';
	DisValArr[bitnum++]='r';
	DisValArr[bitnum++]=':';		
	if(SoftVer[0] > 100)
		DisValArr[bitnum++]=SoftVer[0]/100+0x30;	
	if(SoftVer[0] > 10)
		DisValArr[bitnum++]=(SoftVer[0]%100)/10+0x30;
	DisValArr[bitnum++]=SoftVer[0]%10+0x30;
	DisValArr[bitnum++]='.';	
	if(SoftVer[1] > 100)
		DisValArr[bitnum++]=SoftVer[1]/100+0x30;	
	if(SoftVer[1] > 10)
		DisValArr[bitnum++]=(SoftVer[1]%100)/10+0x30;
	DisValArr[bitnum++]=SoftVer[1]%10+0x30;
	DisValArr[bitnum++]='.';
	if(SoftVer[2] > 100)
		DisValArr[bitnum++]=SoftVer[2]/100+0x30;	
	if(SoftVer[2] > 10)
		DisValArr[bitnum++]=(SoftVer[2]%100)/10+0x30;
	DisValArr[bitnum++]=SoftVer[2]%10+0x30;
	DisValArr[bitnum++]=0xFF;
	DisValArr[bitnum++]=0xFC;
	DisValArr[bitnum++]=0xFF;
	DisValArr[bitnum++]=0xFF;	
	for(i=0;i<bitnum;i++)
		U3TX_BUF[i] = DisValArr[i];
	while(1)
	{
		if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//等待DMA1_Steam3传输完成
		{
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA1_Steam3传输完成标志		
			break;
		}		
	}	
	MYDMA_Enable(DMA1_Stream3,bitnum); 		
}
//获取图片ID
void GetPicNo(void)
{
	u8 DisValArr[20],bitnum=0,i;
	DisValArr[bitnum++]=0xEE;
	DisValArr[bitnum++]=0xB1;
	DisValArr[bitnum++]=0x01;	
	DisValArr[bitnum++]=0xFF;
	DisValArr[bitnum++]=0xFC;
	DisValArr[bitnum++]=0xFF;
	DisValArr[bitnum++]=0xFF;	
	for(i=0;i<bitnum;i++)
		U3TX_BUF[i] = DisValArr[i];
	while(1)
	{
		if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//等待DMA1_Steam3传输完成
		{
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA1_Steam3传输完成标志		
			break;
		}		
	}	
	MYDMA_Enable(DMA1_Stream3,bitnum); 			
}
//改变图片ID
void ChangePic(u16 PicNo)
{
	u8 DisValArr[20],bitnum=0,i;
	DisValArr[bitnum++]=0xEE;
	DisValArr[bitnum++]=0xB1;
	DisValArr[bitnum++]=0x00;	
	DisValArr[bitnum++]=PicNo>>8;	
	DisValArr[bitnum++]=PicNo;		
	DisValArr[bitnum++]=0xFF;
	DisValArr[bitnum++]=0xFC;
	DisValArr[bitnum++]=0xFF;
	DisValArr[bitnum++]=0xFF;
	for(i=0;i<bitnum;i++)
		U3TX_BUF[i] = DisValArr[i];
	while(1)
	{
		if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//等待DMA1_Steam3传输完成
		{
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA1_Steam3传输完成标志		
			break;
		}		
	}	
	MYDMA_Enable(DMA1_Stream3,bitnum); 		
}
//主页1显示函数
void MainDis1(void) 
{
	char *str1,*str2,*str3;
	s32 MainDataDis[6]={0};
	MainDataDis[0] = (DataBuf[0]*98)/10;
	MainDataDis[1] = (DataBuf[1]*98)/10;		
	DisValue(1,3,(MainDataDis[0]/Sys_ParaMeter.FenDu)*Sys_ParaMeter.FenDu,0);
	delay_ms(2);
	DisValue(1,4,(MainDataDis[1]/Sys_ParaMeter.FenDu)*Sys_ParaMeter.FenDu,0);	
	delay_ms(2);			
	DisValue(1,9,(GetBuf[0]*10100)/32768,3);
	delay_ms(2);	
	DisValue(1,10,(GetBuf[1]*10100)/32768,3);	
	delay_ms(2);
	
	if(SpeedBuf[0] > 0)
	{
		DisValue(1,5,(Sys_ParaMeter.RunLenth)/SpeedBuf[0],0);
		delay_ms(2);		
	}
	if(SpeedBuf[1] > 0)
	{
		DisValue(1,6,(Sys_ParaMeter.RunLenth)/SpeedBuf[1],0);	
		delay_ms(2);			
	}		
	switch(DeviceStatus[0])
	{
		case 1:
			str1 = "已储能";		
			break;
		case 2:
			str1 = "未储能";	
			break;
		case 3:
			str1 = "未知";		
			break;	
		default:
			str1 = "未知";	
			break;
	}
	if(DeviceStatus[0] == 3) //未知状态持续100S
	{

		if(ErrFlagF >= 200)
			str1 = "故障";
		else
			ErrFlagF++;
	}
	else
		ErrFlagF=0;

	switch(DeviceStatus[1])
	{
		case 1:
			str2 = "已储能";		
			break;
		case 2:
			str2 = "未储能";	
			break;
		case 3:
			str2 = "未知";		
			break;	
		default:
			str2 = "未知";	
			break;
	}
	if(DeviceStatus[1] == 3) //未知状态持续100S
	{

		if(ErrFlagH >= 200)
			str2 = "故障";
		else
			ErrFlagH++;
	}
	else
		ErrFlagH=0;	
//	str3 = (char *) malloc(strlen(str1) + strlen(str2));
//	strcpy(str3,str1);
//	strcat(str3,str2);	
//	StrDis(1,17,str3);	
//	free(str3);
	StrDis(1,17,str1);
	delay_ms(2);	
	StrDis(1,7,str2);
	delay_ms(2);		
	switch(Sys_ParaMeter.ChannelConfig)
	{
		case 0:
			StrDis(1,18,"A相");		
			break;			
		case 1:
			StrDis(1,18,"B相");		
			break;					
		case 2:
			StrDis(1,18,"C相");		
			break;					
		default:
			StrDis(1,18,"其他");		
			break;					
	}
	delay_ms(2);	
	DisValue(1,15,Temp,1);	
	delay_ms(2);	
	if((ErrFlagH>=200)||(ErrFlagF>=200))
		StrDis(1,19,"超出预设范围！");
	else
		StrDis(1,19,"无");
	delay_ms(2);	
}
//主页2显示函数
void MainDis2(void)
{
	char *str1,*str2,*str3;
	s32 MainDataDis[6]={0};
	MainDataDis[0] = (DataBuf[0]*98)/10;
	MainDataDis[1] = (DataBuf[1]*98)/10;		
	DisValue(2,7,(MainDataDis[0]/Sys_ParaMeter.FenDu)*Sys_ParaMeter.FenDu,0);
	delay_ms(2);
	DisValue(2,8,(MainDataDis[1]/Sys_ParaMeter.FenDu)*Sys_ParaMeter.FenDu,0);	
	delay_ms(2);	
	DisValue(2,5,SpeedBuf[0],0);
	delay_ms(2);
	DisValue(2,6,SpeedBuf[1],0);	
	delay_ms(2);	
	if(SpeedBuf[0] > 0)
	{
		DisValue(2,11,(Sys_ParaMeter.RunLenth)/SpeedBuf[0],0);
		delay_ms(2);		
	}
	if(SpeedBuf[1] > 0)
	{
		DisValue(2,12,(Sys_ParaMeter.RunLenth)/SpeedBuf[1],0);	
		delay_ms(2);			
	}
	switch(DeviceStatus[0])
	{
		case 1:
			str1 = "已储能";		
			break;
		case 2:
			str1 = "未储能";	
			break;
		case 3:
			str1 = "未知";		
			break;	
		default:
			str1 = "未知";	
			break;
	}
	if(DeviceStatus[0] == 3) //未知状态持续100S
	{

		if(ErrFlagF >= 200)
			str1 = "故障";
		else
			ErrFlagF++;
	}
	else
		ErrFlagF=0;

	switch(DeviceStatus[1])
	{
		case 1:
			str2 = "已储能";		
			break;
		case 2:
			str2 = "未储能";	
			break;
		case 3:
			str2 = "未知";		
			break;	
		default:
			str2 = "未知";	
			break;
	}
	if(DeviceStatus[1] == 3) //未知状态持续100S
	{

		if(ErrFlagH >= 200)
			str2 = "故障";
		else
			ErrFlagH++;
	}
	else
		ErrFlagH=0;	
//	str3 = (char *) malloc(strlen(str1) + strlen(str2));
//	strcpy(str3,str1);
//	strcat(str3,str2);	
//	StrDis(2,19,str3);	
//	free(str3);
	StrDis(2,19,str1);
	delay_ms(2);	
	StrDis(2,9,str2);
	delay_ms(2);			
	DisValue(2,17,Sys_ParaMeter.RunLenth,0);	
	delay_ms(2);	
	DisValue(2,15,Temp,1);	
	delay_ms(2);	
	if((ErrFlagH>=200)||(ErrFlagF>=200))
		StrDis(2,21,"超出预设范围！");
	else
		StrDis(2,21,"无");
	delay_ms(2);	
}
//标定界面显示函数
void DemaDis(void)
{
		DisValue(5,3,Sys_ParaMeter.DemaPara[0],0);
		delay_ms(2);
		DisValue(5,4,Sys_ParaMeter.DemaPara[1],0);	
		delay_ms(2);	
			
		DisValue(5,9,DataBuf[0],0);
		delay_ms(2);	
		DisValue(5,10,DataBuf[1],0);	
		delay_ms(2);	

		DisValue(5,2,DemaData,0);	
		delay_ms(2);

		DisValue(5,6,Sys_ParaMeter.DefaultData[0],0);
		delay_ms(2);
		DisValue(5,7,Sys_ParaMeter.DefaultData[1],0);
		delay_ms(2);
		DisValue(5,8,Sys_ParaMeter.DefaultData[2],0);
		delay_ms(2);
		DisValue(5,11,Sys_ParaMeter.DefaultData[3],0);
		delay_ms(2);	
		
		if(ChoseChannel == 1)
		{
			StrDis(5,18,"分闸");
			delay_ms(2);
			DisValue(5,5,GetBuf[0],0);	
			delay_ms(2);
		}
		else if(ChoseChannel == 2)
		{
			StrDis(5,18,"合闸");	
			delay_ms(2);
			DisValue(5,5,GetBuf[1],0);	
			delay_ms(2);			
		}
		else
		{
			StrDis(5,18,"未选中");	
			delay_ms(2);
			DisValue(5,5,0,0);	
			delay_ms(2);			
		}
		
}
//参数界面显示
void ParaDis(void)
{
		char * DisStr;
		DisValue(6,8,Sys_ParaMeter.ReviseValue[7],0); //补偿参数7为速度补偿
		delay_ms(2);
		switch(Sys_ParaMeter.WorkMode)
		{
			case 0 :
				DisStr ="单机";
				break;
			case 1 :
				DisStr ="并网";
				break;		
			case 2 :
				DisStr ="测试";
				break;
			case 3 :
				DisStr ="备用";
				break;
			default:
				Sys_ParaMeter.WorkMode = 0;
				DisStr ="单机";
				break;				
		}
		StrDis(6,9,DisStr);	
		delay_ms(2);	
		if(Sys_ParaMeter.WireLessFlag == 1)
			StrDis(6,10,"打开");
		else
			StrDis(6,10,"关闭");			
		delay_ms(2);	
		switch(Sys_ParaMeter.UartSpeed)
		{
			case 0 :
				DisStr ="9600";
				break;
			case 1 :
				DisStr ="115200";
				break;		
			case 2 :
				DisStr ="230400";
				break;
			default:
				Sys_ParaMeter.UartSpeed = 1;
				DisStr ="115200";
				break;				
		}
		StrDis(6,11,DisStr);	
		delay_ms(2);	
		if(Sys_ParaMeter.TempComp == 1)
			StrDis(6,12,"打开");
		else
			StrDis(6,12,"关闭");			
		delay_ms(2);	
		
		DisValue(6,13,Sys_ParaMeter.SafeThreshold[0],0);	
		delay_ms(2);	
		DisValue(6,14,Sys_ParaMeter.SafeThreshold[1],0);	
		delay_ms(2);	
		DisValue(6,23,Sys_ParaMeter.RunLenth,0);	
		delay_ms(2);	
		DisValue(6,24,Sys_ParaMeter.FenDu,0);	
		delay_ms(2);			
		switch(Sys_ParaMeter.ChannelConfig)
		{
			case 0:
				StrDis(6,5,"A相");		
				break;			
			case 1:
				StrDis(6,5,"B相");		
				break;					
			case 2:
				StrDis(6,5,"C相");		
				break;					
			default:
				StrDis(6,5,"其他");		
				break;					
		}		
		delay_ms(2);	
		DisValue(6,6,Sys_ParaMeter.ReviseValue[6],0);//突变门限
		delay_ms(2);	
		DisValue(6,7,Sys_ParaMeter.ReviseValue[5],0);//稳定门限
		delay_ms(2);	
		DisValue(6,3,Sys_ParaMeter.ReviseValue[4],0);//弹簧K值1
		delay_ms(2);	
		DisValue(6,4,Sys_ParaMeter.ReviseValue[3],0);//弹簧K值2
		delay_ms(2);		
		delay_ms(2);		
}
//时钟页面显示
void TimeDis(void)
{
	char DisStr[100];
	sprintf(DisStr,"20%d-%d-%d %d:%d:%d\r\n",RTC_DateStruct.RTC_Year,RTC_DateStruct.RTC_Month,RTC_DateStruct.RTC_Date,RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
	StrDis(7,2,DisStr);
	delay_ms(2);
}
//存储页面显示
void SaveDis(void)
{
	char * DisStr;
	if(Sys_ParaMeter.CatchFlag == 1)
		StrDis(9,5,"打开");
	else
		StrDis(9,5,"关闭");
	delay_ms(2);
	
	if(FindUSBDeviceFlag == 1)
		StrDis(9,2,"已连接");
	else
		StrDis(9,2,"未发现");
	delay_ms(2);

	if(DataUpdataOver == 2)
	{
		StrDis(9,7,"数据导出完成！");	
		DataUpdataOver = 0;
	}
	delay_ms(2);
}
//通信页面显示
void CommDis(void)
{
	char DisStr[20];
	if(Sys_ParaMeter.UartDebugFlag == 1)
		StrDis(4,3,"打开");
	else
		StrDis(4,3,"关闭");
	delay_ms(2);
	if(Sys_ParaMeter.DHCPFlag == 1)
		StrDis(4,5,"打开");
	else
		StrDis(4,5,"关闭");
	delay_ms(2);
	if(Sys_ParaMeter.ProtocolChange == 1)
		StrDis(4,9,"打开");
	else
		StrDis(4,9,"关闭");
	delay_ms(2);	
	if(Sys_ParaMeter.NETFlag == 1)
		StrDis(4,12,"打开");
	else
		StrDis(4,12,"关闭");
	delay_ms(2);
	
	DisValue(4,17,Sys_ParaMeter.NetPort,0);	
	delay_ms(2);		
	sprintf(DisStr,"%d.%d.%d.%d",Sys_ParaMeter.IPAddress[0],Sys_ParaMeter.IPAddress[1],Sys_ParaMeter.IPAddress[2],Sys_ParaMeter.IPAddress[3]);
	StrDis(4,14,DisStr);	
	delay_ms(2);	
	sprintf(DisStr,"%d.%d.%d.%d",Sys_ParaMeter.NetMask[0],Sys_ParaMeter.NetMask[1],Sys_ParaMeter.NetMask[2],Sys_ParaMeter.NetMask[3]);
	StrDis(4,15,DisStr);	
	delay_ms(2);		
	sprintf(DisStr,"%d.%d.%d.%d",Sys_ParaMeter.GateWay[0],Sys_ParaMeter.GateWay[1],Sys_ParaMeter.GateWay[2],Sys_ParaMeter.GateWay[3]);
	StrDis(4,16,DisStr);	
	delay_ms(2);			
}
//帮助页面显示
void HelpDis(void)
{
	StrDis(10,3,TelPhone);
	delay_ms(2);
	SoftVerDis(10,2);
	delay_ms(2);
}
//用户测试主程序
//返回值:0,正常
//       1,有问题
u8 USH_User_App(void)
{ 
	u32 total,free;
	u8 res=0;
	printf("设备连接成功!\r\n");	 
	res=exf_getfree((u8 *)"2:",&total,&free);
	if(res==0)
	{ 
		printf("FATFS OK!\r\n");	
		printf("U Disk Total Size:%d MB\r\n",total);	 
		printf("U Disk  Free Size:%d MB\r\n",free); 	    
	
	} 
 
	while(HCD_IsDeviceConnected(&USB_OTG_Core))//设备连接成功
	{	
		LED1=!LED1;
		delay_ms(200);
	} 
	printf("设备连接中...\r\n");
	return res;
}
//重置默认参数
void DefautSet(void)
{
	u8 i=0;
	Sys_ParaMeter.FirstFlag = 2;
	Sys_ParaMeter.CatchFlag	= 0;
	Sys_ParaMeter.ChannelConfig = 0;
	Sys_ParaMeter.WorkMode = 0;
	Sys_ParaMeter.WireLessFlag = 0;
	Sys_ParaMeter.USBDeviceFlag = 0;
	Sys_ParaMeter.UartSpeed = 1;
	Sys_ParaMeter.UartDebugFlag = 0;
	Sys_ParaMeter.TimeUpdata = 0;
	Sys_ParaMeter.DHCPFlag = 0;
	Sys_ParaMeter.FenDu = 5;
	Sys_ParaMeter.TempComp = 0;
	Sys_ParaMeter.CreepageComp = 0;
	Sys_ParaMeter.GateWay[0] = 192;
	Sys_ParaMeter.GateWay[1] = 168;
	Sys_ParaMeter.GateWay[2] = 1;
	Sys_ParaMeter.GateWay[3] = 1;
	Sys_ParaMeter.IPAddress[0] = 192;
	Sys_ParaMeter.IPAddress[1] = 168;
	Sys_ParaMeter.IPAddress[2] = 1;
	Sys_ParaMeter.IPAddress[3] = 31;
	Sys_ParaMeter.NetMask[0] = 255;
	Sys_ParaMeter.NetMask[1] = 255;
	Sys_ParaMeter.NetMask[2] = 255;
	Sys_ParaMeter.NetMask[3] = 1;
	Sys_ParaMeter.NETFlag = 0;
	Sys_ParaMeter.NetPort = 8088;	
	Sys_ParaMeter.RunLenth = 100;
	for(i=0;i<5;i++)
		Sys_ParaMeter.CompPara[i] = 1000;
	Sys_ParaMeter.DebugInfo = 1;
	for(i=0;i<8;i++)	
		Sys_ParaMeter.DemaPara[i] = 20000;	
	for(i=0;i<7;i++)		
		Sys_ParaMeter.PassWord[i] = 8; 
	Sys_ParaMeter.ProtocolChange = 0;
	for(i=0;i<8;i++)		
		Sys_ParaMeter.ReviseValue[i] = 1000;
	for(i=0;i<8;i++)				
		Sys_ParaMeter.SafeThreshold[i] = 300;
	for(i=0;i<8;i++)	
		Sys_ParaMeter.ZeroPara[i] = 0;	
	for(i=0;i<12;i++)
		Sys_ParaMeter.DefaultData[i] = 120;
}
//系统参数转移到数组中
void SysParaToArr(u8 * paraArr)
{
	u8 i;
	paraArr[0] = Sys_ParaMeter.FirstFlag;
	paraArr[1] = Sys_ParaMeter.CatchFlag;
	paraArr[2] = Sys_ParaMeter.ChannelConfig;
	paraArr[3] = Sys_ParaMeter.WorkMode;
	paraArr[4] = Sys_ParaMeter.WireLessFlag;
	paraArr[5] = Sys_ParaMeter.USBDeviceFlag;
	paraArr[6] = Sys_ParaMeter.UartSpeed;
	paraArr[7] = Sys_ParaMeter.UartDebugFlag;
	paraArr[8] = Sys_ParaMeter.TimeUpdata;
	paraArr[9] = Sys_ParaMeter.DHCPFlag;
	paraArr[10] = Sys_ParaMeter.GateWay[0];
	paraArr[11] = Sys_ParaMeter.GateWay[1];
	paraArr[12] = Sys_ParaMeter.GateWay[2];
	paraArr[13] = Sys_ParaMeter.GateWay[3];
	paraArr[14] = Sys_ParaMeter.IPAddress[0];
	paraArr[15] = Sys_ParaMeter.IPAddress[1];
	paraArr[16] = Sys_ParaMeter.IPAddress[2];
	paraArr[17] = Sys_ParaMeter.IPAddress[3];
	paraArr[18] = Sys_ParaMeter.NetMask[0];
	paraArr[19] = Sys_ParaMeter.NetMask[1];
	paraArr[20] = Sys_ParaMeter.NetMask[2];
	paraArr[21] = Sys_ParaMeter.NetMask[3];
	paraArr[22] = Sys_ParaMeter.NETFlag;
	paraArr[23] = Sys_ParaMeter.NetPort>>8;
	paraArr[24] = Sys_ParaMeter.NetPort;
	paraArr[25] = Sys_ParaMeter.RunLenth>>8;
	paraArr[26] = Sys_ParaMeter.RunLenth;	
	for(i=0;i<8;i++)
	{
		paraArr[27+2*i] = Sys_ParaMeter.ZeroPara[i]>>8;
		paraArr[27+2*i+1] = Sys_ParaMeter.ZeroPara[i];		
	}	
	for(i=0;i<8;i++)
	{
		paraArr[43+2*i] = Sys_ParaMeter.SafeThreshold[i]>>8;
		paraArr[43+2*i+1] = Sys_ParaMeter.SafeThreshold[i];		
	}	
	for(i=0;i<8;i++)
	{
		paraArr[59+2*i] = Sys_ParaMeter.ReviseValue[i]>>8;
		paraArr[59+2*i+1] = Sys_ParaMeter.ReviseValue[i];		
	}		
	for(i=0;i<8;i++)
	{
		paraArr[75+2*i] = Sys_ParaMeter.DemaPara[i]>>8;
		paraArr[75+2*i+1] = Sys_ParaMeter.DemaPara[i];		
	}	
	for(i=0;i<5;i++)
	{
		paraArr[91+2*i] = Sys_ParaMeter.CompPara[i]>>8;
		paraArr[91+2*i+1] = Sys_ParaMeter.CompPara[i];		
	}		
	for(i=0;i<7;i++)
	{
		paraArr[101+i] = Sys_ParaMeter.PassWord[i];	
	}		
	paraArr[108] = Sys_ParaMeter.FenDu;
	for(i=0;i<12;i++)
	{
		paraArr[109+2*i] = Sys_ParaMeter.DefaultData[i]>>8;
		paraArr[109+2*i+1] = Sys_ParaMeter.DefaultData[i];
	}
	paraArr[133] = Sys_ParaMeter.TempComp;
	paraArr[134] = Sys_ParaMeter.CreepageComp;
}
//系统参数转移到数组中
void SysParaToStruct(u8 * paraArr)
{
	u8 i;
	Sys_ParaMeter.FirstFlag = paraArr[0];
	Sys_ParaMeter.CatchFlag = paraArr[1];
	Sys_ParaMeter.ChannelConfig = paraArr[2];
	Sys_ParaMeter.WorkMode = paraArr[3];
	Sys_ParaMeter.WireLessFlag = paraArr[4];
	Sys_ParaMeter.USBDeviceFlag = paraArr[5];
	Sys_ParaMeter.UartSpeed = paraArr[6];
	Sys_ParaMeter.UartDebugFlag = paraArr[7];
	Sys_ParaMeter.TimeUpdata = paraArr[8];
	Sys_ParaMeter.DHCPFlag = paraArr[9];
	Sys_ParaMeter.GateWay[0] = paraArr[10];
	Sys_ParaMeter.GateWay[1] = paraArr[11];
	Sys_ParaMeter.GateWay[2] = paraArr[12];
	Sys_ParaMeter.GateWay[3] = paraArr[13];
	Sys_ParaMeter.IPAddress[0] = paraArr[14];
	Sys_ParaMeter.IPAddress[1] = paraArr[15];
	Sys_ParaMeter.IPAddress[2] = paraArr[16];
	Sys_ParaMeter.IPAddress[3] = paraArr[17];
	Sys_ParaMeter.NetMask[0] = paraArr[18];
	Sys_ParaMeter.NetMask[1] = paraArr[19];
	Sys_ParaMeter.NetMask[2] = paraArr[20];
	Sys_ParaMeter.NetMask[3] = paraArr[21];
	Sys_ParaMeter.NETFlag = paraArr[22];
	Sys_ParaMeter.NetPort = (paraArr[23]<<8)|paraArr[24];
	Sys_ParaMeter.RunLenth = (paraArr[25]<<8)|paraArr[26];

	for(i=0;i<8;i++)
	{
		Sys_ParaMeter.ZeroPara[i] = (paraArr[27+2*i]<<8)|paraArr[27+2*i+1];	
	}	
	for(i=0;i<8;i++)
	{
		Sys_ParaMeter.SafeThreshold[i] = (paraArr[43+2*i]<<8)|paraArr[43+2*i+1];	
	}	
	for(i=0;i<8;i++)
	{
		Sys_ParaMeter.ReviseValue[i] = (paraArr[59+2*i]<<8)|paraArr[59+2*i+1];	
	}		
	for(i=0;i<8;i++)
	{
		Sys_ParaMeter.DemaPara[i] = (paraArr[75+2*i]<<8)|paraArr[75+2*i+1];			
	}	
	for(i=0;i<5;i++)
	{
		Sys_ParaMeter.CompPara[i] = (paraArr[91+2*i]<<8)|paraArr[91+2*i+1];		
	}		
	for(i=0;i<7;i++)
	{
		Sys_ParaMeter.PassWord[i] = paraArr[101+i];	
	}
  
	Sys_ParaMeter.FenDu = paraArr[108];
	for(i=0;i<12;i++)
		Sys_ParaMeter.DefaultData[i] = (paraArr[109+2*i]<<8)|paraArr[109+2*i+1];
	
	Sys_ParaMeter.TempComp = paraArr[133];
	Sys_ParaMeter.CreepageComp = paraArr[134];
}
//获取MCUID
void cpuidGetId(void)
{
    MCUID[0] = *(__IO u32*)(0x1FFF7A10);
    MCUID[1] = *(__IO u32*)(0x1FFF7A14);
    MCUID[2] = *(__IO u32*)(0x1FFF7A18);
}
//发送实时力值
void ADDataSend(void)
{
	u8 bitnum=0,i=0;
	u8 SendData[32];
	
	SendData[bitnum++] = 0xAA;
	SendData[bitnum++] = 0xBB;
	for(i=0;i<4;i++) //实时4通道数据
	{
		SendData[bitnum++] = DataBuf[i] >> 8;	
		SendData[bitnum++] = DataBuf[i];
	}
	for(i=0;i<4;i++) //前两个通道保存的默认值
	{
		SendData[bitnum++] = Sys_ParaMeter.DefaultData[i] >> 8;	
		SendData[bitnum++] = Sys_ParaMeter.DefaultData[i];
	}	
	SendData[bitnum++] = 0xCC;
	SendData[bitnum++] = 0xDD;	
	
	for(i=0;i<bitnum;i++)
	{
		U6TX_BUF[i] = SendData[i];
	}	
	
	while(1)
	{
		if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
		{
				DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志	
				break;
		}	
	}
	MYDMA_Enable(DMA2_Stream7,bitnum); 
	
}
//发送分闸捕获数据
void FCatchDataSend(void)
{
	u16 bitnum=0,i=0,j=0,Sendnum = 0;
		
	if(SendData_List[2].DataNum > 0) //始终读取末尾的数据组
	{
		taskENTER_CRITICAL();           //进入临界区
		for(i=0;i<SendData_List[2].DataNum;i++)
		{
			U6TX_BUF[bitnum++] = SendData_List[2].WeiZhi[i]>>8;
			U6TX_BUF[bitnum++] = SendData_List[2].WeiZhi[i];					
		}
		
		for(j=2;j>0;j--)
		{
			SendData_List[j].DataNum = SendData_List[j-1].DataNum;
			SendData_List[j].WeiZhi = SendData_List[j-1].WeiZhi;
		}
		SendData_List[0].DataNum = 0;
		FWaitSendList --; //队列数据减一
		taskEXIT_CRITICAL();            //退出临界区			
		while(1)
		{
			if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
			{
					DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志	
					break;
			}	
		}
		MYDMA_Enable(DMA2_Stream7,bitnum); 
		delay_ms((bitnum/100)*7);//防止数据堵塞，设置延时，115200 时 关系为7%
		if(Sys_ParaMeter.UartDebugFlag == 1)
			printf("Send:%x - %x : %d -- %d\r\n",U6TX_BUF[1],U6TX_BUF[3],bitnum,FWaitSendList+1);
	}
}
//发送合闸捕获数据
void HCatchDataSend(void)
{
	u16 bitnum=0,i=0,Sendnum = 0;
	if(HWaitSendNum > 0)
	{
		for(i=0;i<HWaitSendNum;i++)
		{
			U6TX_BUF[bitnum++] = HCatchSaveBuf[i]>>8;
			U6TX_BUF[bitnum++] = HCatchSaveBuf[i];				
		}
		while(1)
		{
			if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
			{
					DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志	
					break;
			}	
		}
		MYDMA_Enable(DMA2_Stream7,bitnum); 
		delay_ms((bitnum/100)*7);//防止数据堵塞，设置延时，115200 时 关系为7%
		if(Sys_ParaMeter.UartDebugFlag == 1)		
			printf("Send:%x - %x : %d -- %d\r\n",U6TX_BUF[1],U6TX_BUF[3],bitnum,HWaitSendList);
		HWaitSendNum = 0;	
		HWaitSendList = 0;		
	}
}
	