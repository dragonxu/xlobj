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
/********************************��������**********************************
***FreeRTOS���ȼ�����Խ�����ȼ�Խ��
***start_task           ��ʼ����
***DataProc_task				���ݴ�������
***DisPlay_task					��ʾ����
***DisPlay_task					��ʾ����
***DataInfo_task				������Ϣ����
***Other_task						������������
***SysInfo_task					ϵͳ��Ϣ����
***HeapSize_task				��ջ�������
**************************************************************************/

//�������ȼ�
#define START_TASK_PRIO		configMAX_PRIORITIES - 2
//�����ջ��С	
#define START_STK_SIZE 		128  
//������
TaskHandle_t STARTTask_Handler;
//������
void start_task(void *pvParameters);

//�������ȼ�
#define DATAPRO_TASK_PRIO		17
//�����ջ��С	
#define DATAPRO_STK_SIZE 		128  
//������
TaskHandle_t DATAPROTask_Handler;
//������
void DataPro_task(void *pvParameters);

//�������ȼ�
#define DISPLAY_TASK_PRIO		10
//�����ջ��С	
#define DISPLAY_STK_SIZE 		128  
//������
TaskHandle_t DISPLAYTask_Handler;
//������
void DisPlay_task(void *pvParameters);

//�������ȼ�
#define DATAINFO_TASK_PRIO		9
//�����ջ��С	
#define DATAINFO_STK_SIZE 		128
//������
TaskHandle_t DATAINFOTask_Handler;
//������
void DataInfo_task(void *pvParameters);

//�������ȼ�
#define OTHER_TASK_PRIO		8
//�����ջ��С	
#define OTHER_STK_SIZE 		128
//������
TaskHandle_t OTHERTask_Handler;
//������
void Other_task(void *pvParameters);

//�������ȼ�
#define SYSINFO_TASK_PRIO		7
//�����ջ��С
#define SYSINFO_STK_SIZE 		256
//������
TaskHandle_t SYSINFOTask_Handler;
//������
void SysInfo_task(void *pvParameters);

//�������ȼ�
#define HEAPSIZE_TASK_PRIO		6
//�����ջ��С
#define HEAPSIZE_STK_SIZE 		300
//������
TaskHandle_t HEAPSIZETask_Handler;
//������
void HeapSize_task(void *pvParameters);

RTC_TimeTypeDef RTC_TimeStruct;
RTC_DateTypeDef RTC_DateStruct;
RTC_TimeTypeDef RTC_TimeSet;
RTC_DateTypeDef RTC_DateSet;
SYS_PARA	Sys_ParaMeter;
volatile SEND_LIST SendData_List[3];
u8	SysParaArr[200];
char InfoBuffer[1000];										//������Ϣ������	
short Temp;																//�¶�	
u16 FlashID;															//FlashID
u8 *test;
u8 ChoseChannel = 0;
u8 FindUSBDeviceFlag = 0;
u8 DataUpdataOver = 0;
u8 LvBoNum=0;
u8 ErrFlagF=0;
u8 ErrFlagH=0;
volatile u8 FWaitSendList = 0;													//�ȴ����Ͷ������ ��բ
volatile u8 HWaitSendList = 0;													//�ȴ����Ͷ������ ��բ
u8 DeviceStatus[3];																			//�豸״̬
u8 FileBuf[2000]; 		 																	//�����ļ���������
u16 DemaData = 2000;
char *TelPhone = "029-88326721";
const u8 SPassWord[7] = {'8','1','2','4','3','8','8',};
const u8 SoftVer[3] = {1,0,15};
s32 BDBuf[6];
s32 ZeroBuf[6];
s16 LvBoBuf[6][32];
s32 LoBoValue[6];
u16 CatchMenXian = 100; 																//���ò�������
u16 WenDingMenXian = 100; 															//�����ȶ�����
u16 WaitSendNum[3];																			//�ȴ����ͻ������� ��բ
u16 HWaitSendNum;																				//�ȴ����ͻ������� ��բ
volatile s16 GetBuf[BUFFER_SIZE];												//ԭʼADֵ
s16 DataBuf[BUFFER_SIZE];																//�˲������ֵ
s16 PDataBuf[BUFFER_SIZE];															//δ�˲�����ֵ
s16 SpeedBuf[BUFFER_SIZE];															//�ٶȻ���
u32 FLASH_SIZE=16*1024*1024;									 					//FLASH ��СΪ16M�ֽ�	
u32 MCUID[3];	
USBH_HOST 					 USB_Host;	
USB_OTG_CORE_HANDLE  USB_OTG_Core;

u16 PICNO=0;	//�������
__align(32) u8 FileSaveBuf[4000] __attribute__((at(0X1000D000))); 		 	//�����ļ��������鵽RAM3
__align(32) s16 FCatchSaveBuf[3][4000] __attribute__((at(0X10003000))); //���岶�����鵽RAM3��բ
__align(32) s16 HCatchSaveBuf[4000] 	 __attribute__((at(0X10009000))); //���岶�����鵽RAM3��բ
__align(32) s16 HCatchBuf[2][2000] 		 __attribute__((at(0X1000B000))); 	//��բ���ɻ�������
s16 FCatchBuf[2][2000]; 																									//��բ���ɻ�������
void DisValue(u16 PicNo,u16 CtrNo,s32 data,u8 Dbit);	  //��ʾ���֣�֧����������5λ����С��
void MainDis1(void);																		//��ҳ1��ʾ����
void MainDis2(void);																		//��ҳ2��ʾ����	
void DemaDis(void);																			//�궨ҳ����ʾ����	
void ParaDis(void);																			//����ҳ����ʾ
void TimeDis(void);																			//ʱ��ҳ����ʾ
void SaveDis(void);																			//�洢ҳ����ʾ
void CommDis(void);																			//ͨ��ҳ����ʾ
void HelpDis(void);																			//����ҳ����ʾ
void SoftVerDis(u16 PicNo,u16 CtrNo);										//�汾����ʾ����
void GetPicNo(void);																		//��ȡ��ǰҳ����	
void ChangePic(u16 PicNo);															//�л�ҳ��
void StrDis(u16 PicNo,u16 CtrNo,char * str);						//��ʾ�ַ���
void DefautSet(void);																		//�ָ�Ĭ�ϲ���
void SysParaToStruct(u8 * paraArr);											//ϵͳ����д�뵽�ṹ��
void SysParaToArr(u8 * paraArr);												//ϵͳ����д�뵽����
void cpuidGetId(void);																	//��ȡMCUID
void ADDataSend(void);																	//ADת�������ݷ���
void FCatchDataSend(void);															//���񵽵����ݷ��� ��բ
void HCatchDataSend(void);															//���񵽵����ݷ��� ��բ

int main(void)
{
	u8 i,res;
 	u32 total,free;		
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	delay_init(168);															//��ʱ��ʼ�� 
	uart_init(115200);														//����1��ʼ��������Ϊ115200
	uart3_init(115200);														//����2��ʼ��������Ϊ115200
  uart4_init(115200);														//����4��ʼ��������Ϊ115200
	uart6_init(115200);														//����6��ʼ��������Ϊ115200
	delay_xms(500);
	ChangePic(0);																	//�л�����ҳ
	delay_xms(500);
	SoftVerDis(0,2);															//��ʾ����汾
	delay_xms(500);	
	LED_Init();		  															//��ʼ����LED���ӵ�Ӳ���ӿ� 
	BEEP_Init();																	//��ʼ����BEEP���ӵ�Ӳ���ӿ� 
	OUTCTL_Init();																//��ʼ���̵������Ƶ�Ӳ���ӿ� 
	My_RTC_Init();																//��ʼ���ڲ�RTC
	cpuidGetId();																	//��ȡMCUID
	printf("CPUID 0x:%x %x %x\r\n",MCUID[0],MCUID[1],MCUID[2]);
	StrDis(0,1,"ϵͳ��ʼ����...");	
	delay_xms(500);	
	W25QXX_Init();																//W25Q128��ʼ��	
//	TIM3_Int_Init(999,839); 										//100khz��Ƶ��,����1000Ϊ10ms			
 	if(DS18B20_Init())														//DS18B20��ʼ��	
	{
		printf("DS18B20 Error!\r\n");
		delay_ms(200);
	} 
	FlashID  = W25QXX_ReadID();										//��ȡFLASHоƬID
	if(FlashID!=W25Q128)													//��ⲻ��W25Q128
	{
		printf("W25Q128 Check Failed!\r\n");
		StrDis(0,1,"SPI-Flash ��ʼ��ʧ�ܣ�");	
		delay_xms(500);			
	}
	else
	{
		printf("Flash ID: %x!\r\n",FlashID);
		StrDis(0,1,"SPI-Flash ��ʼ���ɹ���");	
		delay_xms(500);			
	}
	my_mem_init(SRAMIN);													//��ʼ���ڲ��ڴ��
	my_mem_init(SRAMCCM);													//��ʼ��CCM�ڴ��
 	exfuns_init();																//Ϊfatfs��ر��������ڴ�				 
 	res=f_mount(fs[1],"1:",1); 										//����FLASH.	
	if(res==0X0D)																	//FLASH����,FAT�ļ�ϵͳ����,���¸�ʽ��FLASH
	{
		printf("Flash Disk Formatting...\r\n");			//��ʽ��FLASH
		StrDis(0,1,"Flash Disk��ʽ����");	
		delay_xms(500);				
		res=f_mkfs("1:",1,4096);										//��ʽ��FLASH,1,�̷�;1,����Ҫ������,8������Ϊ1����
		if(res==0)
		{
			f_setlabel((const TCHAR *)"1:sela");			//����Flash���̵�����Ϊ��sela
			printf("Flash Disk Format Finish\r\n");		//��ʽ�����	
			StrDis(0,1,"Flash Disk��ʽ���ɹ���");	
			delay_xms(500);						
		}
		else
		{
			printf("Flash Disk Format Error \r\n");	//��ʽ��ʧ��
			StrDis(0,1,"Flash Disk��ʽ��ʧ�ܣ�");			
		}			
		delay_ms(500);
	}	
	else
	{
		printf("FLASH Fatfs���ش��룺%d\r\n",res);	//FLASH���ش���
		StrDis(0,1,"FATFS��ʼ���ɹ���");	
		delay_xms(500);				
	}
	W25QXX_Read(SysParaArr,FLASH_SIZE-200,200);
	if(SysParaArr[0] != 2) //�ǵ�һ��ʹ��
	{
		printf("Flash ������ʼ����\r\n");
		DefautSet();
		SysParaToArr(SysParaArr);
		W25QXX_Write(SysParaArr,FLASH_SIZE-200,200);
	}
	else
	{
		printf("Flash ������ȡ��\r\n");
		SysParaToStruct(SysParaArr);
		if(Sys_ParaMeter.UartDebugFlag == 1)//�����ӡ������Ϣ		
		{
			for(i=0;i<140;i++)
				printf("Para%d:%d\r\n",i,SysParaArr[i]);		
		}
	}
	f_mount(fs[2],"2:",1); 												//����U��	
	bsp_InitAD7606();															//����AD7606���õ�GPIO
	AD7606_SetOS(AD_OS_X4);												//4������������
	AD7606_StartConvst();													//����1��ADת��
	printf("\r\nAD7606����FIFO����ģʽ (50KHz 8ͨ��ͬ���ɼ�)...\r\n");
	AD7606_StartRecord(10000);										//����10kHz��������
	AD7606DmaRead(SRAM_BANK,(u32)GetBuf,BUFFER_SIZE);	//����һ��DMA����
  USBH_Init(&USB_OTG_Core,USB_OTG_FS_CORE_ID,&USB_Host,&USBH_MSC_cb,&USR_Callbacks);  	//��ʼ��USB����
	for(i=0;i<6;i++)
	{
		LED0=!LED0;
		BEEP=!BEEP;	
		delay_ms(200);
	}
	StrDis(0,1,"��������...");	
	delay_xms(500);			
	
	//������ʼ����
	xTaskCreate((TaskFunction_t )start_task,            //������
							(const char*    )"start_task",          //��������
							(uint16_t       )START_STK_SIZE,        //�����ջ��С
							(void*          )NULL,                  //���ݸ��������Ĳ���
							(UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
							(TaskHandle_t*  )&STARTTask_Handler);   //������              
	vTaskStartScheduler();          //�����������
}
//��ʼ����������
void start_task(void *pvParameters)
{
		u8 res;
	
    taskENTER_CRITICAL();           //�����ٽ���
		//�������ݴ�������
    xTaskCreate((TaskFunction_t )DataPro_task,     	
                (const char*    )"DataPro_task",   	
                (uint16_t       )DATAPRO_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )DATAPRO_TASK_PRIO,	
                (TaskHandle_t*  )&DATAPROTask_Handler);   
    //������ʾ����
    xTaskCreate((TaskFunction_t )DisPlay_task,     
                (const char*    )"DisPlay_task",   
                (uint16_t       )DISPLAY_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )DISPLAY_TASK_PRIO,
                (TaskHandle_t*  )&DISPLAYTask_Handler);        
    //����������Ϣ����
    xTaskCreate((TaskFunction_t )DataInfo_task,     
                (const char*    )"DataInfo_task",   
                (uint16_t       )DATAINFO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )DATAINFO_TASK_PRIO,
                (TaskHandle_t*  )&DATAINFOTask_Handler);  
		//������������						
		xTaskCreate((TaskFunction_t )Other_task,     
                (const char*    )"Other_task",   
                (uint16_t       )OTHER_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )OTHER_TASK_PRIO,
                (TaskHandle_t*  )&OTHERTask_Handler); 
		//����ϵͳ��Ϣ����						
		xTaskCreate((TaskFunction_t )SysInfo_task,     
                (const char*    )"SysInfo_task",   
                (uint16_t       )SYSINFO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )SYSINFO_TASK_PRIO,
                (TaskHandle_t*  )&SYSINFOTask_Handler); 
		//����ϵͳ��Ϣ����						
		xTaskCreate((TaskFunction_t )HeapSize_task,     
                (const char*    )"HeapSize_task",   
                (uint16_t       )HEAPSIZE_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )HEAPSIZE_TASK_PRIO,
                (TaskHandle_t*  )&HEAPSIZETask_Handler); 
		//Ŀǰ�ú����ᴴ���������̣����а���LWIP�߳��Լ����ݽ����߳�
		//��Ϊ��FreeRots��LWIP��ⲻ��͸����Ե�ʣ�������ж��н��н������ݴ��������һЩ������Ȼ��Ӱ��ʹ�á�
		if(Sys_ParaMeter.NETFlag == 1)
		{
			StrDis(0,1,"���Եȣ�LWIP��ʼ��...");	
			delay_ms(200);		
			res=lwip_comm_init();						//��ʼ��������ETH����
			if(res) 		//lwip��ʼ��
			{
				printf("LWIP Init Falied! %d\r\n",res);
				StrDis(0,1,"LWIP��ʼ��ʧ��...");	
				delay_ms(200);				
				if(res == 3)
				{
					printf("Ҫʹ��NET�����������������ϵͳ!\r\n");				
				}
			}
			else				//��������������ݳ�ʼ���ɹ�֮����ܿ�������߳�
			{

				if((Sys_ParaMeter.DHCPFlag==1)&&(LWIP_DHCP))			//����ڿ�ʼ����֮ǰ�������̣߳���Ϊ���ȼ������⣬���ܻᵼ��һЩ����
				{
					printf("DHCP_Config.....\r\n");
					MyDhcp_Creat();				
				}
				else
				{
					printf("��̬IP��TCP������.....\r\n");
					MyTCPS_Creat();					
				}
			}			
		}
		else
		{
			StrDis(0,1,"����LWIP��ʼ��...");	
			delay_ms(200);		
		}
	
		printf("ϵͳ��ʼ����ɣ�\r\n");
    vTaskDelete(STARTTask_Handler); //ɾ����ʼ����				
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

//���ݴ������� 
void DataPro_task(void *pvParameters)
{
	u16 i,j;
	u8 StartNum=0;
	u8 EndFlag1=0,EndFlag2=0;
	u8 CatchFlag[2] = {0};				//���������رձ��
	u8 SteaDataNum=0;							//�������ݼ���			
	u8 ChannelStatus[4] = {0};		//ͨ��״̬��̬����̬�жϡ�ÿ����Ϊһ�飬��0��1����ͨ��1��̬�����̬ͳ������
	u32 NotifyValue=0;						//����֪ͨ����ֵ
	s16 GetSteaBuf[2][32];				//ÿ��ͨ�����32����������
	s16 MaxValueBuf[2] = {0};			//���������е����ֵ
	s16 MinValueBuf[2] = {0};			//���������е���Сֵ
	u16 MaxCatchnum[2] = {0};			//���������е����ֵλ��
	u16 MinCatchnum[2] = {0};			//���������е���Сֵλ��
	u16 CatchNum[2] = {0};				//�������ݸ���
	u16 CatchChannelFlag = 0;			//�������ݱ�ʶ����������ʹ��
	u16 FWaitSendFlag = 0;
	u16 HWaitSendFlag = 0;
	while(1)
	{
		NotifyValue=ulTaskNotifyTake(pdTRUE,1000);	//��ȡ����֪ͨ
		if(StartNum < 64)
			StartNum++;
		//32λ�����˲�
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
		//ȥ��λ�����ݻ���
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
		//��ȡÿһ�൱ǰ״̬

		CatchMenXian = Sys_ParaMeter.ReviseValue[6]; //��ʱʹ�ò�������6���޸Ĳ�������
		if((CatchMenXian < 50)||(CatchMenXian > 500))
		{
			CatchMenXian = 100;	
			Sys_ParaMeter.ReviseValue[6] = 100;			
		}
		WenDingMenXian = Sys_ParaMeter.ReviseValue[5]; //��ʱʹ�ò�������6���޸Ĳ�������
		if((WenDingMenXian < 50)||(WenDingMenXian > 500))
		{
			WenDingMenXian = 100;	
			Sys_ParaMeter.ReviseValue[5] = 100;			
		}
		
		if(StartNum >= 64)  //����64��ſ�ʼ���빤��
		{
			if(SteaDataNum < 32) //���浱ǰ���ݵ�ǰ32����,δ�˲�����
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
			//��̬ͳ�� A���բ 1 ChannelStatus[0]����̬������ChannelStatus[1]��̬������
			//ͻ���жϣ���ԭʼ�����������32�����˲�ֵ300kg��Ϊ����������ԭʼ���ݴ����˲�������+300������ԭʼ����С���˲�������-300
			if((PDataBuf[0] > (DataBuf[0] + CatchMenXian))||(PDataBuf[0] < (DataBuf[0] - CatchMenXian))) //ͻ������
			{
				ChannelStatus[0]++; 
				if(ChannelStatus[0] >= 10) //�仯��������10�Σ�������̬
				{
					ChannelStatus[1] = 0;
					if(ChannelStatus[0] == 10) //�����ֵ��Сֵ��ֵ
					{
						MaxValueBuf[0] = DataBuf[0];
						MinValueBuf[0] = DataBuf[0];
					}
					CatchFlag[0] = 1;
				}
				if(ChannelStatus[0] > 200)
					ChannelStatus[0] = 50;				
			}	
			//else //ǰ�汾ֱ��ʹ��else���жϷ�Χ����ͬ�ġ����˴�����ٴ������ʵ�ͬʱҲͬʱ�������ȵļ��ʡ���
			//�޶�---�ڴ�������������ʱ�������������ȣ��ȶ�����Ϊ���50kg����
			else if((PDataBuf[0] < (DataBuf[0]+WenDingMenXian))&&(PDataBuf[0] > (DataBuf[0]-WenDingMenXian)))
			{
				ChannelStatus[1]++;				
				if(ChannelStatus[1] >= 100)//����100�Σ��ָ���̬
				{
					if(CatchNum[0] >= 1000)  //һ�δ������ٲɼ�500�����ݲ�����ֹ
						ChannelStatus[0] = 0;			
				}
				if(ChannelStatus[1] > 200)
					ChannelStatus[1] = 100;					
			}
			//��̬ͳ�� A���բ 2		----��ʱ���ε���բ���ɲ���	
//			if((PDataBuf[1] > (DataBuf[1] + CatchMenXian))||(PDataBuf[1] < (DataBuf[1] - CatchMenXian)))
//			{
//				ChannelStatus[2]++;
//				if(ChannelStatus[2] >= 10) //�仯��������10�Σ�������̬
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
//				if(ChannelStatus[3] >= 50)//����50�Σ��ָ���̬
//				{
//					if(CatchNum[1] >= 500)  //һ�δ������ٲɼ�500�����ݲ�����ֹ
//						ChannelStatus[2] = 0;			
//				}
//				if(ChannelStatus[3] > 200)
//					ChannelStatus[3] = 50;					
//			}
			//��̬��ȡ�豸״̬������̬����ͻ������ A���բ������ ------------------------A1
			if(ChannelStatus[0] >= 10) //����ͻ������
			{
				if(FWaitSendFlag > 0) //�����ݵ����������ݻ�δ���ͣ�ֱ�ӷ���
				{
					CatchNum[0] = 0;
					CatchFlag[0] = 0;	
					FWaitSendFlag = 0;
				}
				if(CatchNum[0]<32) //����ǰ32����������
				{
					for(i=0;i<32;i++)
					{
						FCatchBuf[0][CatchNum[0]] = GetSteaBuf[0][i];
						FCatchBuf[1][CatchNum[0]] = GetSteaBuf[1][i];
						CatchNum[0]++;
					}
				}
				else if(CatchNum[0]<1980) //ֻ�ܻ���1980������
				{
					FCatchBuf[0][CatchNum[0]] = PDataBuf[0];
					FCatchBuf[1][CatchNum[0]] = PDataBuf[1];
					CatchNum[0]++;					
				}		
			}
			else if(ChannelStatus[1] >= 100) //��̬��ֻ����̬�ж��豸״̬
			{
				EndFlag1 = 0;
				EndFlag2 = 0;
				for(i=0;i<CatchNum[0];i++) //Ѱ�ҹؼ����ݵ�λ��
				{
					//Ѱ������ͨ���������ݵ����ֵ
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
					//ͨ��1С��Ԥ���բֵ֮���ٴ���
					if((FCatchBuf[0][i] < Sys_ParaMeter.DefaultData[0])&&(EndFlag1 == 0))
					{
						EndFlag1 = 1;	
					}

					//ͨ��2С��Ԥ��δ����ֵ֮���ٴ���
					if((FCatchBuf[1][i] < Sys_ParaMeter.DefaultData[2])&&(EndFlag2 == 0))
					{
						MinValueBuf[1] = FCatchBuf[1][i];
						MinCatchnum[1] = i;
						EndFlag2 = 1;
					}
					//������Сֵ
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
				if((CatchFlag[0] == 1)&&(CatchNum[0] > 50))//���粶����Ϊ1 �� �����������50����Ϊ��Ч
				{
					if(FCatchBuf[0][CatchNum[0]-1] > (FCatchBuf[0][0]+200))				//����ͻ��
					{
						SpeedBuf[0] = (MinCatchnum[1])/10; //10k�ٶȣ����ݵ�����ȡʱ��
						printf("Speed1:%d \r\n",SpeedBuf[0]);
						CatchChannelFlag = 0xA1A1;	//A���բ����ͻ���ʶ
					}
					else if((FCatchBuf[0][CatchNum[0]-1]+200) < FCatchBuf[0][0]) //�½�ͻ��
					{
						SpeedBuf[1] = (MinCatchnum[0]-MaxCatchnum[0])/10;
						printf("Speed2:%d \r\n",SpeedBuf[1]);
						CatchChannelFlag = 0xA2A2;	//A���բ�½�ͻ���ʶ
					}
					else
					{
						SpeedBuf[0] = MinCatchnum[0]/10;
						printf("Speed3:%d \r\n",SpeedBuf[0]);
						CatchChannelFlag = 0xA3A3;	//A���բ����ͻ���ʶ
					}
					//����Ч�����������������´洢��������
					if(FWaitSendList < 3) //�б����п�λʱ��������䣬������
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
												
						//�������ʹ�õı���
						CatchNum[0] = 0;
						CatchFlag[0] = 0;	
						FWaitSendFlag = 0;
					}
					else  	//�����Ͷ�������
					{
						//printf("Wait:%d \r\n",FWaitSendFlag);
						FWaitSendFlag++;
						if(FWaitSendFlag >= 10000) //���ȴ�1S,��ʱ���������
						{
							//�������ʹ�õı���
							CatchNum[0] = 0;
							CatchFlag[0] = 0;
							FWaitSendFlag = 0;
						}
					}
				}
				else //��Ч���������ز���
				{
					//�������ʹ�õı���
					CatchNum[0] = 0;
					CatchFlag[0] = 0;
					FWaitSendFlag = 0;						
				}
				MinCatchnum[0] = 0;
				MaxCatchnum[0] = 0;
				MaxValueBuf[0] = 0;
				MinValueBuf[0] = 0;	

				//�豸״̬�ж�
				if((DataBuf[0] > (Sys_ParaMeter.DefaultData[1]-Sys_ParaMeter.SafeThreshold[0]))&&
					(DataBuf[0] < (Sys_ParaMeter.DefaultData[1]+Sys_ParaMeter.SafeThreshold[0])))//�ں�բ��ȫ������		
					DeviceStatus[0] = 1;
				else if((DataBuf[0] > (Sys_ParaMeter.DefaultData[0]-Sys_ParaMeter.SafeThreshold[0]))&&
					(DataBuf[0] < (Sys_ParaMeter.DefaultData[0]+Sys_ParaMeter.SafeThreshold[0])))//�ڷ�բ��ȫ������
					DeviceStatus[0] = 2;
				else //Σ��״̬
					DeviceStatus[0] = 3;

				if((DataBuf[1] > (Sys_ParaMeter.DefaultData[3]-Sys_ParaMeter.SafeThreshold[1]))&&
					(DataBuf[1] < (Sys_ParaMeter.DefaultData[3]+Sys_ParaMeter.SafeThreshold[1])))//��ȫ����������				
					DeviceStatus[1] = 1;
				else if((DataBuf[1] > (Sys_ParaMeter.DefaultData[2]-Sys_ParaMeter.SafeThreshold[1]))&&
					(DataBuf[1] < (Sys_ParaMeter.DefaultData[2]+Sys_ParaMeter.SafeThreshold[1])))//��ȫδ����������
					DeviceStatus[1] = 2;
				else										//����״̬���߹���
					DeviceStatus[1] = 3;
					
					
			}
			//��̬��ȡ�豸״̬������̬����ͻ������ A���բ������ ------------A2
			//��ʱ���ε���բ
//			if(ChannelStatus[2] >= 10)
//			{
//				if(HWaitSendFlag > 0)//������δ���ͣ������Ϣ
//				{
//					CatchNum[1] = 0;
//					CatchFlag[1] = 0;
//					HWaitSendFlag = 0;							
//				}
//				if(CatchNum[1]<20) //����ǰ20����������
//				{
//					for(i=0;i<20;i++)
//					{
//						HCatchBuf[0][CatchNum[1]] = GetSteaBuf[0][i];		
//						HCatchBuf[1][CatchNum[1]] = GetSteaBuf[1][i];	
//						CatchNum[1]++;							
//					}
//				}
//				else if(CatchNum[1]<1980) //ֻ�ܻ���1980������
//				{
//					HCatchBuf[0][CatchNum[1]] = PDataBuf[0];	
//					HCatchBuf[1][CatchNum[1]] = PDataBuf[1];	

//					//��ȡ���ֵ����λ��
//					if(MaxValueBuf[1] < HCatchBuf[0][CatchNum[1]])
//					{
//						MaxValueBuf[1] = HCatchBuf[0][CatchNum[1]];
//						MaxCatchnum[1] = CatchNum[1];
//					}
//					//��ȡ��Сֵ����λ��,�����㸺ֵ
//					if((MinValueBuf[1] > HCatchBuf[0][CatchNum[1]])&&(HCatchBuf[0][CatchNum[1]] >= 0))
//					{
//						MinValueBuf[1] = HCatchBuf[0][CatchNum[1]];
//						MinCatchnum[1] = CatchNum[1];
//					}	
//					CatchNum[1]++;					
//				}			
//			}
//			else if(ChannelStatus[3] >= 20) //��̬��ֻ����̬�ж��豸״̬
//			{
//				if((CatchFlag[1] == 1)&&(CatchNum[1] > 50))//���粶����Ϊ1 �� �����������50����Ϊ��Ч
//				{
//					if(HCatchBuf[0][CatchNum[1]-1] > (HCatchBuf[0][0]+200))				//����ͻ��
//					{
//						SpeedBuf[1] = MaxCatchnum[1]/10; //10k�ٶȣ����ݵ�����ȡʱ��
//						CatchChannelFlag = 0xA4A4;	//A���բ����ͻ���ʶ
//					}
//					else if((HCatchBuf[0][CatchNum[1]-1]+200) < HCatchBuf[0][0]) //�½�ͻ��
//					{
//						SpeedBuf[1] = MinCatchnum[1]/10;
//						CatchChannelFlag = 0xA5A5;	//A���բ�½�ͻ���ʶ
//					}
//					else
//					{
//						SpeedBuf[1] = CatchNum[1]/10;
//						CatchChannelFlag = 0xA6A6;	//A���բ����ͻ���ʶ
//					}	
//					if(HWaitSendList == 0) //��բ�����Ͷ���Ϊ��
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
//						//�������ʹ�õı���
//						CatchNum[1] = 0;
//						CatchFlag[1] = 0;
//						HWaitSendFlag = 0;							
//					}
//					else
//					{
//						HWaitSendFlag++;
//						if(HWaitSendFlag >= 10000) //�ȴ�ʱ�䳬��1S���������
//						{
//							//�������ʹ�õı���
//							CatchNum[1] = 0;
//							CatchFlag[1] = 0;
//							HWaitSendFlag = 0;							
//						}
//					}
//				}
//				else //��Ч�������
//				{
//					//�������ʹ�õı���
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
//��ʾ����
void DisPlay_task(void *pvParameters)
{		
		s32 CheckNum = 0;
		delay_ms(500);		
		StrDis(0,1,"��ɣ�����ϵͳ...");	
		delay_ms(500);		
		ChangePic(1);										//�л�����ҳ
		PICNO = 1;
		delay_ms(500);			
		while(1)
		{	
			CheckNum++;
			switch(PICNO)
			{
				case 1:		//��ҳ1
					MainDis1();
					break;
				case 2:		//��ҳ2
					MainDis2();
					break;
				case 4:		//ͨ��ҳ��
					CommDis();
					break;
				case 5:		//�궨ҳ��
					DemaDis();
					break;
				case 6:		//����ҳ��
					ParaDis();
					break;
				case 7:		//ʱ��ҳ��
					TimeDis();
					break;
				case 9:		//�洢ҳ��
					SaveDis();
					break;
				case 10:	//����ҳ��
					HelpDis();
					break;
				default:
					break;
			}
			delay_ms(5);	
			if(CheckNum%20 == 0) //ʮ���ȡһ�ε�ǰҳ��
				GetPicNo();
			vTaskDelay(500); 
		}
}
//������Ϣ����
void DataInfo_task(void *pvParameters)
{
		u8 i,j;
		u8 PsNo=0;
		u8 PWDCheck=0,PWDCheck1=0,PWDCheck2=0;
		u8 PWDBuf1[7],PWDBuf2[7];//���뻺��
		char DisInfo[100];
		u8 DGetBuf[40];
		u8 DGetBufNum=0;
		while(1)
		{	
			if(DISCmdLen>0)//���յ�������ָ��
			{
				for(i=8;i<DISCmdLen-5;i++) //��ȡ���յ��������ݴӵڰ�λ��ʼ,DISCmdLen-5����
				{
					DGetBuf[DGetBufNum++] = U3RX_BUF[i];
				}
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x02))//��ҳ2--------------2
				{
					if(U3RX_BUF[6] == 0x03)  //�����г�
					{
						if(Sys_ParaMeter.RunLenth < 5000)
							Sys_ParaMeter.RunLenth++;					
					}
					if(U3RX_BUF[6] == 0x04)  //�����г�
					{
						if(Sys_ParaMeter.RunLenth>10)
							Sys_ParaMeter.RunLenth--;							
					}			
				}
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x03))//����ҳ-------------3
				{
					if((U3RX_BUF[6] == 0x08)&&(U3RX_BUF[8] == 0x00))//ȡ������
					{
						ChangePic(1);
						PICNO = 1;
						delay_ms(5);
					}
					if((U3RX_BUF[6] == 0x08)&&(U3RX_BUF[8] == 0x01))//�������
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
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x04))//ͨ��ҳ------------4
				{
					switch(U3RX_BUF[6])
					{
						case 6:	//DHCP����
							if(U3RX_BUF[8] == 0)
								Sys_ParaMeter.DHCPFlag = 1;
							else
								Sys_ParaMeter.DHCPFlag = 0;
							break;
						case 7: //�������ڿ���
							if(U3RX_BUF[8] == 0)
								Sys_ParaMeter.UartDebugFlag = 1;
							else
								Sys_ParaMeter.UartDebugFlag = 0;
							break;
						case 10://��Լת������
							if(U3RX_BUF[8] == 0)
								Sys_ParaMeter.ProtocolChange	= 1;
							else
								Sys_ParaMeter.ProtocolChange = 0;
							break;
						case 13://��·���ܿ���
							if(U3RX_BUF[8] == 0)
								Sys_ParaMeter.NETFlag	= 1;
							else
								Sys_ParaMeter.NETFlag = 0;
							break;
						case 17://����˿�
							Sys_ParaMeter.NetPort = 0;
							for(i=0;i<DGetBufNum;i++)
							{
								Sys_ParaMeter.NetPort = Sys_ParaMeter.NetPort*10+(GetBuf[i]-0x30);								
							}
							if(Sys_ParaMeter.UartDebugFlag == 1)
								printf("NetPort:%d\r\n",Sys_ParaMeter.NetPort);
							break;
						case 14://IP��ַ
							j=0;
							Sys_ParaMeter.IPAddress[j] = 0;
							for(i=0;i<DGetBufNum;i++)
							{
								if(DGetBuf[i] == '.')//�ָ��
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
						case 15://��������
							j=0;
							Sys_ParaMeter.NetMask[j] = 0;
							for(i=0;i<DGetBufNum;i++)
							{
								if(DGetBuf[i] == '.')//�ָ��
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
						case 16://���ص�ַ
							j=0;
							Sys_ParaMeter.GateWay[j] = 0;
							for(i=0;i<DGetBufNum;i++)
							{
								if(DGetBuf[i] == '.')//�ָ��
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
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x05))//�궨ҳ------------5
				{
					if(U3RX_BUF[6] == 2) //Ŀ��ֵ
					{
						DemaData = 0;
						for(i=0;i<DGetBufNum;i++)
						{
							DemaData = DemaData*10+(DGetBuf[i]-0x30);								
						}					
					}
					if(U3RX_BUF[6] == 15) //�������
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
					if(U3RX_BUF[6] == 16) //�궨
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
					if((U3RX_BUF[6]>2)&&(U3RX_BUF[6]<5))//�궨ϵ������
					{
						Sys_ParaMeter.DemaPara[U3RX_BUF[6]-3] = 0;
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.DemaPara[U3RX_BUF[6]-3] = Sys_ParaMeter.DemaPara[U3RX_BUF[6]-3]*10+(DGetBuf[i]-0x30);								
						}							
					}
					if((U3RX_BUF[6]>18)&&(U3RX_BUF[6]<21))//ͨ��ѡ��
					{
						ChoseChannel = U3RX_BUF[6] - 18;
					}
					//�洢��ʼֵ
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
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x06))//����ҳ-------------6
				{				
					if(U3RX_BUF[6] == 25) //��λ����
					{
							Sys_ParaMeter.ChannelConfig = U3RX_BUF[8];
					}
					if(U3RX_BUF[6] == 3) //�¶Ȳ�������
					{
						Sys_ParaMeter.ReviseValue[4]= 0; //��������4��Ϊ����Kֵ1
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.ReviseValue[4] = Sys_ParaMeter.ReviseValue[4]*10+(DGetBuf[i]-0x30);								
						}	
					}						
					if(U3RX_BUF[6] == 4) //����⿪��
					{
						Sys_ParaMeter.ReviseValue[3]= 0; //��������3��Ϊ����Kֵ2
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.ReviseValue[3] = Sys_ParaMeter.ReviseValue[3]*10+(DGetBuf[i]-0x30);								
						}	
					}	
					
					if(U3RX_BUF[6] == 6) //��������
					{
						Sys_ParaMeter.ReviseValue[6]= 0; //��������6��Ϊ��������
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.ReviseValue[6] = Sys_ParaMeter.ReviseValue[6]*10+(DGetBuf[i]-0x30);								
						}						
					}
					if(U3RX_BUF[6] == 7) //�ȶ�����
					{
						Sys_ParaMeter.ReviseValue[5]= 0; //��������5��Ϊ�ȶ�����
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.ReviseValue[5] = Sys_ParaMeter.ReviseValue[5]*10+(DGetBuf[i]-0x30);								
						}						
					}					
					
					if(U3RX_BUF[6] == 8) //�ٶȲ���
					{
						Sys_ParaMeter.ReviseValue[7]= 0; //��������7��Ϊ�ٶȲ���
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.ReviseValue[7] = Sys_ParaMeter.ReviseValue[7]*10+(DGetBuf[i]-0x30);								
						}						
					}
					if(U3RX_BUF[6] == 16)//����ģʽ
						Sys_ParaMeter.WorkMode = U3RX_BUF[8];
					if(U3RX_BUF[6] == 22)//���߿���
					{
						if(U3RX_BUF[8] == 0)
							Sys_ParaMeter.WireLessFlag = 1;
						else
							Sys_ParaMeter.WireLessFlag = 0;
					}
					if(U3RX_BUF[6] == 20)//�����ٶ�
						Sys_ParaMeter.UartSpeed = U3RX_BUF[8];
					if(U3RX_BUF[6] == 21)//�¶Ȳ���
					{
						if(U3RX_BUF[8] == 0)
							Sys_ParaMeter.TempComp = 1;
						else
							Sys_ParaMeter.TempComp = 0;
					}					
					if(U3RX_BUF[6] == 13) //������ֵ safe0
					{
						Sys_ParaMeter.SafeThreshold[0] = 0; 
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.SafeThreshold[0] = Sys_ParaMeter.SafeThreshold[0]*10+(DGetBuf[i]-0x30);								
						}						
					}
					if(U3RX_BUF[6] == 14) //ѹ����ֵ safe1
					{
						Sys_ParaMeter.SafeThreshold[1] = 0; 
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.SafeThreshold[1] = Sys_ParaMeter.SafeThreshold[1]*10+(DGetBuf[i]-0x30);								
						}						
					}	
					if(U3RX_BUF[6] == 23) //�����г�
					{
						Sys_ParaMeter.RunLenth= 0; 
						for(i=0;i<DGetBufNum;i++)
						{
							Sys_ParaMeter.RunLenth = Sys_ParaMeter.RunLenth*10+(DGetBuf[i]-0x30);								
						}						
					}
          if(U3RX_BUF[6] == 24)	//�ֶ�ֵ
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
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x07))//ʱ��ҳ------------7
				{
					if(U3RX_BUF[6] == 3) //ʱ������	
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
					if(U3RX_BUF[6] == 4) //����ͬ��
					{
						;
					}
					if(U3RX_BUF[6] == 5) //Զ��ͬ��
					{
						;
					}						
					if(U3RX_BUF[6] == 6) //��ʱͬ��	
					{
						;
					}						
				}			
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x08))//����ҳ-----------8
				{
					if(U3RX_BUF[6] == 2) //������
					{
						for(i=0;i<DGetBufNum;i++)
						{
								if((DGetBuf[i] == (Sys_ParaMeter.PassWord[i]+0x30))||(DGetBuf[i] == SPassWord[i]))
									PsNo++;
						}
						if((DGetBufNum < 7)||(PsNo<7))
						{
							StrDis(8,7,"�������");
							PWDCheck1 = 0;
						}
						else if(PsNo == 7)
						{
							StrDis(8,7,"������ȷ��");
							PWDCheck1 = 1;
						}
						PsNo = 0;
					}
					if(U3RX_BUF[6] == 3) //������1
					{
						for(i=0;i<DGetBufNum;i++)
						{
							PWDBuf1[i] = DGetBuf[i];
						}	
						if(DGetBufNum != 7)
						{
							StrDis(8,7,"���볤�ȴ���");
							PWDCheck2=0;
						}
						else
							PWDCheck2=1;
					}
					if(U3RX_BUF[6] == 4) //������2
					{
						for(i=0;i<DGetBufNum;i++)
						{
							PWDBuf2[i] = DGetBuf[i];
						}	
						if(DGetBufNum != 7)
						{
							StrDis(8,7,"���볤�ȴ���");
							PWDCheck2=0;							
						}
						else
							PWDCheck2=1;
					}	
					if(U3RX_BUF[6] == 5) //��������
					{
						sprintf(DisInfo,"�������룺%x-%x",MCUID[1],MCUID[2]);
						StrDis(8,7,DisInfo);						
					}
					if(U3RX_BUF[6] == 6) //ȷ���޸�
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
								StrDis(8,7,"�����޸���ɣ�");	
							}
							else
								StrDis(8,7,"�������벻һ�£�");	
						}
						else
						{
							if(PWDCheck1 == 0)
								StrDis(8,7,"ԭ�������");	
						}
					}	
					delay_ms(5);					
				}			
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x09))//�洢ҳ--------------9
				{
					if(U3RX_BUF[6] == 3) //������Ϣ
					{
						DataUpdataOver = 1;
						StrDis(9,7,"��ʼ��������...");	
					}
					if(U3RX_BUF[6] == 6) //���񿪹�
					{
						if(U3RX_BUF[8] == 0)
							Sys_ParaMeter.CatchFlag = 1;
						else
							Sys_ParaMeter.CatchFlag = 0;
					}
					if(U3RX_BUF[6] == 9) //�ָ���������
					{
						if(U3RX_BUF[8] == 0)
							Sys_ParaMeter.FirstFlag = 1;
						else
							Sys_ParaMeter.FirstFlag = 2; //Ϊ2ʱ���ָ�����
					}					
					delay_ms(5);
				}						
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[4] == 0x0B))//����У��ҳ----------11
				{
					if(U3RX_BUF[6] == 0x04) //��������
					{
						for(i=0;i<7;i++)
						{
							if((U3RX_BUF[8+i] == (Sys_ParaMeter.PassWord[i]+0x30))||(U3RX_BUF[8+i] == SPassWord[i]))//�˶�����
								PsNo++;
							if(Sys_ParaMeter.UartDebugFlag == 1)//�����ӡ������Ϣ
								printf("ps%d:%d\r\n",i,U3RX_BUF[8+i]);
						}
						if(PsNo == 7)//������ȷ
						{
							if(Sys_ParaMeter.UartDebugFlag == 1)//�����ӡ������Ϣ
								printf("������ȷ��\r\n");
							StrDis(11,5,"������ȷ...");
							PWDCheck = 1;		
							delay_ms(5);							
						}
						else
						{
							if(Sys_ParaMeter.UartDebugFlag == 1)//�����ӡ������Ϣ
								printf("�������\r\n");
							StrDis(11,5,"�������...");
							PWDCheck = 0;							
							delay_ms(5);
						}
						PsNo=0;	
					}			
					if(U3RX_BUF[6] == 0x02) //ȷ��
					{
						if(PWDCheck == 1) //������ȷ
						{
							delay_ms(5);		
							ChangePic(3);	
							delay_ms(5);							
						}
						PWDCheck = 0;
					}						
					if(U3RX_BUF[6] == 0x03) //��������
					{
						sprintf(DisInfo,"�������룺%x-%x",MCUID[1],MCUID[2]);
						StrDis(11,5,DisInfo);
						delay_ms(5);
					}
				}
				if((U3RX_BUF[1] == 0xB1)&&(U3RX_BUF[2] == 0x01)&&(DISCmdLen == 9))
				{
					PICNO = U3RX_BUF[3]<<8|U3RX_BUF[4];
					if(Sys_ParaMeter.UartDebugFlag == 1)//�����ӡ������Ϣ
						printf("��ǰҳ�棺%d\r\n",PICNO);
				}
				DISCmdLen = 0;
				DGetBufNum = 0; //�����ȡ���ݸ���
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
		p=mymalloc(SRAMIN,1024);//����2K�ֽ�	
		while(1)
		{
			if(p!=NULL)sprintf((char*)p,"Memory Malloc Test%03d",i);//��pд��һЩ����		
			test = p;
			USBH_Process(&USB_OTG_Core, &USB_Host);
			delay_ms(1);
			i++;
			vTaskDelay(200); 
		}
}
//ϵͳ��Ϣ����
void SysInfo_task(void *pvParameters)
{
	u32 SysRunNum = 0;
	while(1)
	{
		if(DataUpdataOver == 1)
			DataUpdataOver = 2;
		if(FWaitSendList > 0)	//��բ�����Ͷ��в�Ϊ��
		{
			FCatchDataSend();		//��բ�������ݷ���		
		}
		else if(HWaitSendList >0)	//��բ�����Ͷ��в�Ϊ��
		{
			HCatchDataSend();		//��բ�������ݷ���		
		}
		if((FWaitSendList == 0)&&(HWaitSendList == 0)) //û�д����͵Ĳ�������
		{
			if((SysRunNum%5) == 0)
				ADDataSend();				 //����һ����ֵ����	
		}
		if((SysRunNum%50) == 0) //5��ִ��һ��		
		{
			taskENTER_CRITICAL();           //�����ٽ���
			Temp=DS18B20_Get_Temp();	
			taskEXIT_CRITICAL();            //�˳��ٽ���				
			if(Sys_ParaMeter.UartDebugFlag == 1)//�����ӡ������Ϣ
				printf("ADת������:%d/��--�������ޣ�%d -- �ȶ����ޣ�%d\r\n",GetADNum/5,CatchMenXian,WenDingMenXian);
			GetADNum = 0;			
		}
		if((SysRunNum%10) == 0)
			LED0=!LED0;//��˸LED,��ʾϵͳ��������.			
		SysRunNum++;
		if(SysRunNum > 200)
			SysRunNum = 1;
		vTaskDelay(100);   //ѭ��ʱ��Ϊ100ms��
	}
}
//**HeapSize_task			��ջ�������
void HeapSize_task(void *pvParameters)
{
	u8 tbuf[40];
	while(1)
	{
			tcp_server_flag |= LWIP_SEND_DATA; //���LWIP������Ҫ����
			RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);			
			sprintf((char*)tbuf,"Time:%02d:%02d:%02d",RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds); 
//			printf("%s\r\n",tbuf);
			RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);			
			sprintf((char*)tbuf,"Date:20%02d-%02d-%02d",RTC_DateStruct.RTC_Year,RTC_DateStruct.RTC_Month,RTC_DateStruct.RTC_Date); 
//			printf("%s\r\n",tbuf);

//			vTaskList(InfoBuffer);								//��ȡ�����������Ϣ
//			printf("������\t����״̬\t���ȼ�\tʣ���ջ\t�������\r\n");
//			printf("%s\r\n",InfoBuffer);					//ͨ�����ڴ�ӡ�����������Ϣ
			
     vTaskDelay(1000);                     //��ʱ1s��Ҳ����1000��ʱ�ӽ���	
	}
}  
///////////////////////////////////////////
//������ʾ��ʽ��
//PicNo:������
//CtrNo���ؼ����
//data����ʾ����
//Dbit: С����λ�� 
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
	if(data<0)//�������Ӹ���
	{
		DisValArr[bitnum++]='-';	
		data = 0-data;
	}
	if(data>9999999)//����9999999�����ݲ���ʾ
		DisValArr[bitnum++]='F';
	else
	{
		if(Dbit > 5)//���֧��5λС��
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
		if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//�ȴ�DMA1_Steam3�������
		{
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//���DMA1_Steam3������ɱ�־		
			break;
		}		
	}
	MYDMA_Enable(DMA1_Stream3,bitnum); 
//	if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//�ȴ�DMA1_Steam4�������
//		DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);//���DMA1_Steam4������ɱ�־	
//	MYDMA_Enable(DMA1_Stream4,bitnum); 
//	for(i=0;i<bitnum;i++)
//	{
//			USART_SendData(USART3, DisValArr[i]);         					//�򴮿�3��������
//			while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);	//�ȴ����ͽ���		
//	}
}

///////////////////////////////////////////
//�ַ�����ʾ
//PicNo:������
//CtrNo���ؼ����
//str����ʾ�ַ���
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
		if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//�ȴ�DMA1_Steam3�������
		{
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//���DMA1_Steam3������ɱ�־		
			break;
		}		
	}	
	MYDMA_Enable(DMA1_Stream3,bitnum);	
}
///////////////////////////////////////////
//����汾����ʾ��ʽ��
//PicNo:������
//CtrNo���ؼ����
//data����ʾ����
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
		if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//�ȴ�DMA1_Steam3�������
		{
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//���DMA1_Steam3������ɱ�־		
			break;
		}		
	}	
	MYDMA_Enable(DMA1_Stream3,bitnum); 		
}
//��ȡͼƬID
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
		if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//�ȴ�DMA1_Steam3�������
		{
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//���DMA1_Steam3������ɱ�־		
			break;
		}		
	}	
	MYDMA_Enable(DMA1_Stream3,bitnum); 			
}
//�ı�ͼƬID
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
		if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//�ȴ�DMA1_Steam3�������
		{
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//���DMA1_Steam3������ɱ�־		
			break;
		}		
	}	
	MYDMA_Enable(DMA1_Stream3,bitnum); 		
}
//��ҳ1��ʾ����
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
			str1 = "�Ѵ���";		
			break;
		case 2:
			str1 = "δ����";	
			break;
		case 3:
			str1 = "δ֪";		
			break;	
		default:
			str1 = "δ֪";	
			break;
	}
	if(DeviceStatus[0] == 3) //δ֪״̬����100S
	{

		if(ErrFlagF >= 200)
			str1 = "����";
		else
			ErrFlagF++;
	}
	else
		ErrFlagF=0;

	switch(DeviceStatus[1])
	{
		case 1:
			str2 = "�Ѵ���";		
			break;
		case 2:
			str2 = "δ����";	
			break;
		case 3:
			str2 = "δ֪";		
			break;	
		default:
			str2 = "δ֪";	
			break;
	}
	if(DeviceStatus[1] == 3) //δ֪״̬����100S
	{

		if(ErrFlagH >= 200)
			str2 = "����";
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
			StrDis(1,18,"A��");		
			break;			
		case 1:
			StrDis(1,18,"B��");		
			break;					
		case 2:
			StrDis(1,18,"C��");		
			break;					
		default:
			StrDis(1,18,"����");		
			break;					
	}
	delay_ms(2);	
	DisValue(1,15,Temp,1);	
	delay_ms(2);	
	if((ErrFlagH>=200)||(ErrFlagF>=200))
		StrDis(1,19,"����Ԥ�跶Χ��");
	else
		StrDis(1,19,"��");
	delay_ms(2);	
}
//��ҳ2��ʾ����
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
			str1 = "�Ѵ���";		
			break;
		case 2:
			str1 = "δ����";	
			break;
		case 3:
			str1 = "δ֪";		
			break;	
		default:
			str1 = "δ֪";	
			break;
	}
	if(DeviceStatus[0] == 3) //δ֪״̬����100S
	{

		if(ErrFlagF >= 200)
			str1 = "����";
		else
			ErrFlagF++;
	}
	else
		ErrFlagF=0;

	switch(DeviceStatus[1])
	{
		case 1:
			str2 = "�Ѵ���";		
			break;
		case 2:
			str2 = "δ����";	
			break;
		case 3:
			str2 = "δ֪";		
			break;	
		default:
			str2 = "δ֪";	
			break;
	}
	if(DeviceStatus[1] == 3) //δ֪״̬����100S
	{

		if(ErrFlagH >= 200)
			str2 = "����";
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
		StrDis(2,21,"����Ԥ�跶Χ��");
	else
		StrDis(2,21,"��");
	delay_ms(2);	
}
//�궨������ʾ����
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
			StrDis(5,18,"��բ");
			delay_ms(2);
			DisValue(5,5,GetBuf[0],0);	
			delay_ms(2);
		}
		else if(ChoseChannel == 2)
		{
			StrDis(5,18,"��բ");	
			delay_ms(2);
			DisValue(5,5,GetBuf[1],0);	
			delay_ms(2);			
		}
		else
		{
			StrDis(5,18,"δѡ��");	
			delay_ms(2);
			DisValue(5,5,0,0);	
			delay_ms(2);			
		}
		
}
//����������ʾ
void ParaDis(void)
{
		char * DisStr;
		DisValue(6,8,Sys_ParaMeter.ReviseValue[7],0); //��������7Ϊ�ٶȲ���
		delay_ms(2);
		switch(Sys_ParaMeter.WorkMode)
		{
			case 0 :
				DisStr ="����";
				break;
			case 1 :
				DisStr ="����";
				break;		
			case 2 :
				DisStr ="����";
				break;
			case 3 :
				DisStr ="����";
				break;
			default:
				Sys_ParaMeter.WorkMode = 0;
				DisStr ="����";
				break;				
		}
		StrDis(6,9,DisStr);	
		delay_ms(2);	
		if(Sys_ParaMeter.WireLessFlag == 1)
			StrDis(6,10,"��");
		else
			StrDis(6,10,"�ر�");			
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
			StrDis(6,12,"��");
		else
			StrDis(6,12,"�ر�");			
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
				StrDis(6,5,"A��");		
				break;			
			case 1:
				StrDis(6,5,"B��");		
				break;					
			case 2:
				StrDis(6,5,"C��");		
				break;					
			default:
				StrDis(6,5,"����");		
				break;					
		}		
		delay_ms(2);	
		DisValue(6,6,Sys_ParaMeter.ReviseValue[6],0);//ͻ������
		delay_ms(2);	
		DisValue(6,7,Sys_ParaMeter.ReviseValue[5],0);//�ȶ�����
		delay_ms(2);	
		DisValue(6,3,Sys_ParaMeter.ReviseValue[4],0);//����Kֵ1
		delay_ms(2);	
		DisValue(6,4,Sys_ParaMeter.ReviseValue[3],0);//����Kֵ2
		delay_ms(2);		
		delay_ms(2);		
}
//ʱ��ҳ����ʾ
void TimeDis(void)
{
	char DisStr[100];
	sprintf(DisStr,"20%d-%d-%d %d:%d:%d\r\n",RTC_DateStruct.RTC_Year,RTC_DateStruct.RTC_Month,RTC_DateStruct.RTC_Date,RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
	StrDis(7,2,DisStr);
	delay_ms(2);
}
//�洢ҳ����ʾ
void SaveDis(void)
{
	char * DisStr;
	if(Sys_ParaMeter.CatchFlag == 1)
		StrDis(9,5,"��");
	else
		StrDis(9,5,"�ر�");
	delay_ms(2);
	
	if(FindUSBDeviceFlag == 1)
		StrDis(9,2,"������");
	else
		StrDis(9,2,"δ����");
	delay_ms(2);

	if(DataUpdataOver == 2)
	{
		StrDis(9,7,"���ݵ�����ɣ�");	
		DataUpdataOver = 0;
	}
	delay_ms(2);
}
//ͨ��ҳ����ʾ
void CommDis(void)
{
	char DisStr[20];
	if(Sys_ParaMeter.UartDebugFlag == 1)
		StrDis(4,3,"��");
	else
		StrDis(4,3,"�ر�");
	delay_ms(2);
	if(Sys_ParaMeter.DHCPFlag == 1)
		StrDis(4,5,"��");
	else
		StrDis(4,5,"�ر�");
	delay_ms(2);
	if(Sys_ParaMeter.ProtocolChange == 1)
		StrDis(4,9,"��");
	else
		StrDis(4,9,"�ر�");
	delay_ms(2);	
	if(Sys_ParaMeter.NETFlag == 1)
		StrDis(4,12,"��");
	else
		StrDis(4,12,"�ر�");
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
//����ҳ����ʾ
void HelpDis(void)
{
	StrDis(10,3,TelPhone);
	delay_ms(2);
	SoftVerDis(10,2);
	delay_ms(2);
}
//�û�����������
//����ֵ:0,����
//       1,������
u8 USH_User_App(void)
{ 
	u32 total,free;
	u8 res=0;
	printf("�豸���ӳɹ�!\r\n");	 
	res=exf_getfree((u8 *)"2:",&total,&free);
	if(res==0)
	{ 
		printf("FATFS OK!\r\n");	
		printf("U Disk Total Size:%d MB\r\n",total);	 
		printf("U Disk  Free Size:%d MB\r\n",free); 	    
	
	} 
 
	while(HCD_IsDeviceConnected(&USB_OTG_Core))//�豸���ӳɹ�
	{	
		LED1=!LED1;
		delay_ms(200);
	} 
	printf("�豸������...\r\n");
	return res;
}
//����Ĭ�ϲ���
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
//ϵͳ����ת�Ƶ�������
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
//ϵͳ����ת�Ƶ�������
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
//��ȡMCUID
void cpuidGetId(void)
{
    MCUID[0] = *(__IO u32*)(0x1FFF7A10);
    MCUID[1] = *(__IO u32*)(0x1FFF7A14);
    MCUID[2] = *(__IO u32*)(0x1FFF7A18);
}
//����ʵʱ��ֵ
void ADDataSend(void)
{
	u8 bitnum=0,i=0;
	u8 SendData[32];
	
	SendData[bitnum++] = 0xAA;
	SendData[bitnum++] = 0xBB;
	for(i=0;i<4;i++) //ʵʱ4ͨ������
	{
		SendData[bitnum++] = DataBuf[i] >> 8;	
		SendData[bitnum++] = DataBuf[i];
	}
	for(i=0;i<4;i++) //ǰ����ͨ�������Ĭ��ֵ
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
		if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������
		{
				DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־	
				break;
		}	
	}
	MYDMA_Enable(DMA2_Stream7,bitnum); 
	
}
//���ͷ�բ��������
void FCatchDataSend(void)
{
	u16 bitnum=0,i=0,j=0,Sendnum = 0;
		
	if(SendData_List[2].DataNum > 0) //ʼ�ն�ȡĩβ��������
	{
		taskENTER_CRITICAL();           //�����ٽ���
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
		FWaitSendList --; //�������ݼ�һ
		taskEXIT_CRITICAL();            //�˳��ٽ���			
		while(1)
		{
			if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������
			{
					DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־	
					break;
			}	
		}
		MYDMA_Enable(DMA2_Stream7,bitnum); 
		delay_ms((bitnum/100)*7);//��ֹ���ݶ�����������ʱ��115200 ʱ ��ϵΪ7%
		if(Sys_ParaMeter.UartDebugFlag == 1)
			printf("Send:%x - %x : %d -- %d\r\n",U6TX_BUF[1],U6TX_BUF[3],bitnum,FWaitSendList+1);
	}
}
//���ͺ�բ��������
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
			if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������
			{
					DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־	
					break;
			}	
		}
		MYDMA_Enable(DMA2_Stream7,bitnum); 
		delay_ms((bitnum/100)*7);//��ֹ���ݶ�����������ʱ��115200 ʱ ��ϵΪ7%
		if(Sys_ParaMeter.UartDebugFlag == 1)		
			printf("Send:%x - %x : %d -- %d\r\n",U6TX_BUF[1],U6TX_BUF[3],bitnum,HWaitSendList);
		HWaitSendNum = 0;	
		HWaitSendList = 0;		
	}
}
	