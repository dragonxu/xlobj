#include "lwip_comm.h" 
#include "ethernetif.h" 
#include "malloc.h"
#include "lwip/netif.h"
#include "lwip/ip.h"
#include "lwip/tcp.h"
#include "lwip/init.h"
#include "lwip/dhcp.h"
#include "netif/etharp.h"
#include "lwip/timers.h"
#include "lwip/tcp_impl.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/tcpip.h"
#include "lwip/ip_frag.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "sys.h"
#include "lwip/lwip_sys.h"
#include "delay.h"
#include "string.h"

__lwip_dev lwipdev;							//lwip���ƽṹ�� 
struct netif lwip_netif;				//����һ��ȫ�ֵ�����ӿ�

extern u32 memp_get_memorysize(void);	//��memp.c���涨��
extern u8_t *memp_memory;				//��memp.c���涨��.
extern u8_t *ram_heap;					//��mem.c���涨��.
u8 ENTInt_OK = 0;
u32 TCPTimer=0;				//TCP��ѯ��ʱ��
u32 ARPTimer=0;				//ARP��ѯ��ʱ��
volatile u32 lwip_localtime;		//lwip����ʱ�������,��λ:ms

u8 tcp_server_recvbuf[TCP_SERVER_RX_BUFSIZE];	//TCP�ͻ��˽������ݻ�����
u8 *tcp_server_sendbuf=(u8 *)"�����н�̩�Ƽ����޹�˾SPP��Ŀ TCP SERVER TEST��\r\n";	
u8 tcp_server_flag;								//TCP���������ݷ��ͱ�־λ

//�������ȼ�
#define DHCP_TASK_PRIO		14
//�����ջ��С	
#define DHCP_STK_SIZE 		128  
//������
TaskHandle_t DHCPTask_Handler;
//������
void MyDhcpCreat_task(void *pvParameters);

//�������ȼ�
#define TCPS_TASK_PRIO		13
//�����ջ��С	
#define TCPS_STK_SIZE 		300  
//������
TaskHandle_t TCPSTask_Handler;
//������
void MyTCPServer_task(void *pvParameters);

#if LWIP_DHCP
u32 DHCPfineTimer=0;		//DHCP��ϸ�����ʱ��
u32 DHCPcoarseTimer=0;	//DHCP�ֲڴ����ʱ��
#endif	
//lwip��mem��memp���ڴ�����
//����ֵ:0,�ɹ�;
//    ����,ʧ��
u8 lwip_comm_mem_malloc(void)
{
	u32 mempsize;
	u32 ramheapsize; 
	mempsize=memp_get_memorysize();			//�õ�memp_memory�����С
	memp_memory=mymalloc(SRAMIN,mempsize);	//Ϊmemp_memory�����ڴ�
	ramheapsize=LWIP_MEM_ALIGN_SIZE(MEM_SIZE)+2*LWIP_MEM_ALIGN_SIZE(4*3)+MEM_ALIGNMENT;//�õ�ram heap��С
	ram_heap=mymalloc(SRAMIN,ramheapsize);	//Ϊram_heap�����ڴ� 
	if(!memp_memory||!ram_heap)//������ʧ�ܵ�
	{
		lwip_comm_mem_free();
		return 1;
	}
	return 0;	
}
//lwip��mem��memp�ڴ��ͷ�
void lwip_comm_mem_free(void)
{ 	
	myfree(SRAMIN,memp_memory);
	myfree(SRAMIN,ram_heap);
}
/*-----------------------------------------------------------------------------------*/
//lwip Ĭ��IP����
//lwipx:lwip���ƽṹ��ָ��
void lwip_comm_default_ip_set(__lwip_dev *lwipx)
{
	u32 sn0;
	sn0=*(vu32*)(0x1FFF7A10);//��ȡSTM32��ΨһID��ǰ24λ��ΪMAC��ַ�����ֽ�
	//Ĭ��Զ��IPΪ:192.168.1.100
	lwipx->remoteip[0]=192;	
	lwipx->remoteip[1]=168;
	lwipx->remoteip[2]=1;
	lwipx->remoteip[3]=22;
	//MAC��ַ����(�����ֽڹ̶�Ϊ:2.0.0,�����ֽ���STM32ΨһID)
	lwipx->mac[0]=2;//�����ֽ�(IEEE��֮Ϊ��֯ΨһID,OUI)��ַ�̶�Ϊ:2.0.0
	lwipx->mac[1]=0;
	lwipx->mac[2]=0;
	lwipx->mac[3]=(sn0>>16)&0XFF;//�����ֽ���STM32��ΨһID
	lwipx->mac[4]=(sn0>>8)&0XFFF;;
	lwipx->mac[5]=sn0&0XFF; 	
	//Ĭ�ϱ���IPΪ:192.168.1.30
	lwipx->ip[0]=Sys_ParaMeter.IPAddress[0];	
	lwipx->ip[1]=Sys_ParaMeter.IPAddress[1];
	lwipx->ip[2]=Sys_ParaMeter.IPAddress[2];
	lwipx->ip[3]=Sys_ParaMeter.IPAddress[3];
	//Ĭ����������:255.255.255.0
	lwipx->netmask[0]=Sys_ParaMeter.NetMask[0];	
	lwipx->netmask[1]=Sys_ParaMeter.NetMask[1];
	lwipx->netmask[2]=Sys_ParaMeter.NetMask[2];
	lwipx->netmask[3]=Sys_ParaMeter.NetMask[3];
	//Ĭ������:192.168.1.1
	lwipx->gateway[0]=Sys_ParaMeter.GateWay[0];	
	lwipx->gateway[1]=Sys_ParaMeter.GateWay[1];
	lwipx->gateway[2]=Sys_ParaMeter.GateWay[2];
	lwipx->gateway[3]=Sys_ParaMeter.GateWay[3];	

	lwipx->dhcpstatus=0;//û��DHCP	
}

//������̫���жϵ���
//void lwip_pkt_handle(void)
//{
//	ethernetif_input(&lwip_netif);
//}
u8 lwip_comm_init(void)
{
	struct netif *Netif_Init_Flag;			//����netif_add()����ʱ�ķ���ֵ,�����ж������ʼ���Ƿ�ɹ�
	struct ip_addr ipaddr;  						//ip��ַ
	struct ip_addr netmask; 						//��������
	struct ip_addr gw;      						//Ĭ������ 
	if(ETH_Mem_Malloc())return 1;				//�ڴ�����ʧ��
	if(lwip_comm_mem_malloc())return 2;	//�ڴ�����ʧ��	
	if(LAN8720_Init())return 3;					//��ʼ��LAN8720ʧ�� 
	tcpip_init(NULL,NULL);							//��ʼ��tcp ip�ں�,�ú�������ᴴ��tcpip_thread�ں�����
	lwip_comm_default_ip_set(&lwipdev);	//����Ĭ��IP����Ϣ

	if((Sys_ParaMeter.DHCPFlag == 1)&&(LWIP_DHCP)) //��̬��ȡIP
	{
		ipaddr.addr = 0;
		netmask.addr = 0;
		gw.addr = 0;	
	}
	else			//ʹ�þ�̬IP
	{
		IP4_ADDR(&ipaddr,lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
		IP4_ADDR(&netmask,lwipdev.netmask[0],lwipdev.netmask[1] ,lwipdev.netmask[2],lwipdev.netmask[3]);
		IP4_ADDR(&gw,lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
		printf("����en��MAC��ַΪ:................%d.%d.%d.%d.%d.%d\r\n",lwipdev.mac[0],lwipdev.mac[1],lwipdev.mac[2],lwipdev.mac[3],lwipdev.mac[4],lwipdev.mac[5]);
		printf("��̬IP��ַ........................%d.%d.%d.%d\r\n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
		printf("��������..........................%d.%d.%d.%d\r\n",lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
		printf("Ĭ������..........................%d.%d.%d.%d\r\n",lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);	
	}
  taskENTER_CRITICAL();           //�����ٽ���
	Netif_Init_Flag=netif_add(&lwip_netif,&ipaddr,&netmask,&gw,NULL,&ethernetif_init,&ethernet_input);//�������б������һ������
	taskEXIT_CRITICAL();            //�˳��ٽ���
	if((Sys_ParaMeter.DHCPFlag == 1)&&(LWIP_DHCP)) //��̬��ȡIP
	{
		lwipdev.dhcpstatus=0;	//DHCP���Ϊ0
		dhcp_start(&lwip_netif);	//����DHCP����
	}
	if(Netif_Init_Flag==NULL)return 3;//�������ʧ�� 
	else//������ӳɹ���,����netifΪĬ��ֵ,���Ҵ�netif����
	{
		netif_set_default(&lwip_netif); //����netifΪĬ������
		netif_set_up(&lwip_netif);		//��netif����
	}
	return 0;//����OK.
} 

//���ʹ����DHCP
#if LWIP_DHCP

void MyDhcp_Creat(void)
{
    taskENTER_CRITICAL();           //�����ٽ���
    //�������ݴ�������
    xTaskCreate((TaskFunction_t )MyDhcpCreat_task,     	
                (const char*    )"dhcp_task",   	
                (uint16_t       )DHCP_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )DHCP_TASK_PRIO,	
                (TaskHandle_t*  )&DHCPTask_Handler);   		
    taskEXIT_CRITICAL();            //�˳��ٽ���
}
//ɾ��DHCP����
void MyDhcp_delete(void)
{
	dhcp_stop(&lwip_netif); 		//�ر�DHCP
	vTaskDelete(DHCPTask_Handler); //ɾ������	
}
//DHCP��������
void MyDhcpCreat_task(void *pvParameters)
{
	u32 ip=0,netmask=0,gw=0;
	dhcp_start(&lwip_netif);//����DHCP 
	lwipdev.dhcpstatus=0;	//����DHCP
	printf("���ڲ���DHCP������,���Ե�...........\r\n");   
	while(1)
	{ 
		printf("���ڻ�ȡ��ַ...\r\n");
		ip=lwip_netif.ip_addr.addr;		//��ȡ��IP��ַ
		netmask=lwip_netif.netmask.addr;//��ȡ��������
		gw=lwip_netif.gw.addr;			//��ȡĬ������ 
		if(ip!=0)   					//����ȷ��ȡ��IP��ַ��ʱ��
		{
			lwipdev.dhcpstatus=2;	//DHCP�ɹ�
 			printf("����en��MAC��ַΪ:................%d.%d.%d.%d.%d.%d\r\n",lwipdev.mac[0],lwipdev.mac[1],lwipdev.mac[2],lwipdev.mac[3],lwipdev.mac[4],lwipdev.mac[5]);
			//������ͨ��DHCP��ȡ����IP��ַ
			lwipdev.ip[3]=(uint8_t)(ip>>24); 
			lwipdev.ip[2]=(uint8_t)(ip>>16);
			lwipdev.ip[1]=(uint8_t)(ip>>8);
			lwipdev.ip[0]=(uint8_t)(ip);
			printf("ͨ��DHCP��ȡ��IP��ַ..............%d.%d.%d.%d\r\n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
			//����ͨ��DHCP��ȡ�������������ַ
			lwipdev.netmask[3]=(uint8_t)(netmask>>24);
			lwipdev.netmask[2]=(uint8_t)(netmask>>16);
			lwipdev.netmask[1]=(uint8_t)(netmask>>8);
			lwipdev.netmask[0]=(uint8_t)(netmask);
			printf("ͨ��DHCP��ȡ����������............%d.%d.%d.%d\r\n",lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
			//������ͨ��DHCP��ȡ����Ĭ������
			lwipdev.gateway[3]=(uint8_t)(gw>>24);
			lwipdev.gateway[2]=(uint8_t)(gw>>16);
			lwipdev.gateway[1]=(uint8_t)(gw>>8);
			lwipdev.gateway[0]=(uint8_t)(gw);
			printf("ͨ��DHCP��ȡ����Ĭ������..........%d.%d.%d.%d\r\n",lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
			ENTInt_OK = 1;
			MyTCPS_Creat();					
			break;
		}
		else if(lwip_netif.dhcp->tries > LWIP_MAX_DHCP_TRIES) //ͨ��DHCP�����ȡIP��ַʧ��,�ҳ�������Դ���
		{  
			lwipdev.dhcpstatus=0XFF;//DHCPʧ��.
			//ʹ�þ�̬IP��ַ
			IP4_ADDR(&(lwip_netif.ip_addr),lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
			IP4_ADDR(&(lwip_netif.netmask),lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
			IP4_ADDR(&(lwip_netif.gw),lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
			printf("DHCP����ʱ,ʹ�þ�̬IP��ַ!\r\n");
			printf("����en��MAC��ַΪ:................%d.%d.%d.%d.%d.%d\r\n",lwipdev.mac[0],lwipdev.mac[1],lwipdev.mac[2],lwipdev.mac[3],lwipdev.mac[4],lwipdev.mac[5]);
			printf("��̬IP��ַ........................%d.%d.%d.%d\r\n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
			printf("��������..........................%d.%d.%d.%d\r\n",lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
			printf("Ĭ������..........................%d.%d.%d.%d\r\n",lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
			MyTCPS_Creat();					
			break;
		}
		delay_ms(500); //��ʱ500ms		
	}
	MyDhcp_delete();//ɾ��DHCP���� 
}
#endif 
void MyTCPS_delete(void)
{
	vTaskDelete(TCPSTask_Handler); //ɾ������	
}
void MyTCPS_Creat(void)
{
    taskENTER_CRITICAL();           //�����ٽ���
    //�������ݴ�������
    xTaskCreate((TaskFunction_t )MyTCPServer_task,     	
                (const char*    )"TCPS_task",   	
                (uint16_t       )TCPS_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )TCPS_TASK_PRIO,	
                (TaskHandle_t*  )&TCPSTask_Handler);   		
    taskEXIT_CRITICAL();            //�˳��ٽ���
}
void MyTCPServer_task(void *pvParameters)
{
//	u16 tt;
	u32 data_len = 0;
	struct pbuf *q;
	err_t err,recv_err;
	u8 remot_addr[4];
	struct netconn *conn, *newconn;
	static ip_addr_t ipaddr;
	static u16_t 			port;
	
	LWIP_UNUSED_ARG(pvParameters);

	conn = netconn_new(NETCONN_TCP);  //����һ��TCP����
	netconn_bind(conn,IP_ADDR_ANY,TCP_SERVER_PORT);  //�󶨶˿� 8�Ŷ˿�
	netconn_listen(conn);  			//�������ģʽ
	conn->recv_timeout = 10;  	//��ֹ�����߳� �ȴ�10ms
	while (1) 
	{
//		tt++;
		err = netconn_accept(conn,&newconn);  //������������
//		if(tt%200 == 0)
//			printf("Err code: %d\r\n",err);
		if(err==ERR_OK)newconn->recv_timeout = 10;

		if (err == ERR_OK)    //���������ӵ�����
		{ 
			struct netbuf *recvbuf;

			netconn_getaddr(newconn,&ipaddr,&port,0); //��ȡԶ��IP��ַ�Ͷ˿ں�
			
			remot_addr[3] = (uint8_t)(ipaddr.addr >> 24); 
			remot_addr[2] = (uint8_t)(ipaddr.addr>> 16);
			remot_addr[1] = (uint8_t)(ipaddr.addr >> 8);
			remot_addr[0] = (uint8_t)(ipaddr.addr);
			printf("����%d.%d.%d.%d�����Ϸ�����,�����˿ں�Ϊ:%d\r\n",remot_addr[0], remot_addr[1],remot_addr[2],remot_addr[3],port);
			
			while(1)
			{
				if((tcp_server_flag & LWIP_SEND_DATA) == LWIP_SEND_DATA) //������Ҫ����
				{
					err = netconn_write(newconn ,tcp_server_sendbuf,strlen((char*)tcp_server_sendbuf),NETCONN_COPY); //����tcp_server_sendbuf�е�����
					if(err != ERR_OK)
					{
						printf("����ʧ��\r\n");
					}
					tcp_server_flag &= ~LWIP_SEND_DATA;
				}
				recv_err = netconn_recv(newconn,&recvbuf);				
				if(recv_err == ERR_OK)  	//���յ�����
				{	
					taskENTER_CRITICAL();           //�����ٽ���
					memset(tcp_server_recvbuf,0,TCP_SERVER_RX_BUFSIZE);  //���ݽ��ջ���������
					for(q=recvbuf->p;q!=NULL;q=q->next)  //����������pbuf����
					{
						//�ж�Ҫ������TCP_SERVER_RX_BUFSIZE�е������Ƿ����TCP_SERVER_RX_BUFSIZE��ʣ��ռ䣬�������
						//�Ļ���ֻ����TCP_SERVER_RX_BUFSIZE��ʣ�೤�ȵ����ݣ�����Ļ��Ϳ������е�����
						if(q->len > (TCP_SERVER_RX_BUFSIZE-data_len)) memcpy(tcp_server_recvbuf+data_len,q->payload,(TCP_SERVER_RX_BUFSIZE-data_len));//��������
						else memcpy(tcp_server_recvbuf+data_len,q->payload,q->len);
						data_len += q->len;  	
						if(data_len > TCP_SERVER_RX_BUFSIZE) break; //����TCP�ͻ��˽�������,����	
					}
					taskEXIT_CRITICAL();            //�˳��ٽ���
					data_len=0;  //������ɺ�data_lenҪ���㡣	
					printf("%s\r\n",tcp_server_recvbuf);  //ͨ�����ڷ��ͽ��յ�������
					netbuf_delete(recvbuf);
				}
				else if(recv_err == ERR_CLSD)  //�ر�����
				{
					netconn_close(newconn);
					netconn_delete(newconn);
					printf("����:%d.%d.%d.%d�Ͽ��������������\r\n",remot_addr[0], remot_addr[1],remot_addr[2],remot_addr[3]);
					break;
				}
			}
		}
	}
}
