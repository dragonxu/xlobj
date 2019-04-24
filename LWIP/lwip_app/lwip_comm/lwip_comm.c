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

__lwip_dev lwipdev;							//lwip控制结构体 
struct netif lwip_netif;				//定义一个全局的网络接口

extern u32 memp_get_memorysize(void);	//在memp.c里面定义
extern u8_t *memp_memory;				//在memp.c里面定义.
extern u8_t *ram_heap;					//在mem.c里面定义.
u8 ENTInt_OK = 0;
u32 TCPTimer=0;				//TCP查询计时器
u32 ARPTimer=0;				//ARP查询计时器
volatile u32 lwip_localtime;		//lwip本地时间计数器,单位:ms

u8 tcp_server_recvbuf[TCP_SERVER_RX_BUFSIZE];	//TCP客户端接收数据缓冲区
u8 *tcp_server_sendbuf=(u8 *)"西安市杰泰科技有限公司SPP项目 TCP SERVER TEST！\r\n";	
u8 tcp_server_flag;								//TCP服务器数据发送标志位

//任务优先级
#define DHCP_TASK_PRIO		14
//任务堆栈大小	
#define DHCP_STK_SIZE 		128  
//任务句柄
TaskHandle_t DHCPTask_Handler;
//任务函数
void MyDhcpCreat_task(void *pvParameters);

//任务优先级
#define TCPS_TASK_PRIO		13
//任务堆栈大小	
#define TCPS_STK_SIZE 		300  
//任务句柄
TaskHandle_t TCPSTask_Handler;
//任务函数
void MyTCPServer_task(void *pvParameters);

#if LWIP_DHCP
u32 DHCPfineTimer=0;		//DHCP精细处理计时器
u32 DHCPcoarseTimer=0;	//DHCP粗糙处理计时器
#endif	
//lwip中mem和memp的内存申请
//返回值:0,成功;
//    其他,失败
u8 lwip_comm_mem_malloc(void)
{
	u32 mempsize;
	u32 ramheapsize; 
	mempsize=memp_get_memorysize();			//得到memp_memory数组大小
	memp_memory=mymalloc(SRAMIN,mempsize);	//为memp_memory申请内存
	ramheapsize=LWIP_MEM_ALIGN_SIZE(MEM_SIZE)+2*LWIP_MEM_ALIGN_SIZE(4*3)+MEM_ALIGNMENT;//得到ram heap大小
	ram_heap=mymalloc(SRAMIN,ramheapsize);	//为ram_heap申请内存 
	if(!memp_memory||!ram_heap)//有申请失败的
	{
		lwip_comm_mem_free();
		return 1;
	}
	return 0;	
}
//lwip中mem和memp内存释放
void lwip_comm_mem_free(void)
{ 	
	myfree(SRAMIN,memp_memory);
	myfree(SRAMIN,ram_heap);
}
/*-----------------------------------------------------------------------------------*/
//lwip 默认IP设置
//lwipx:lwip控制结构体指针
void lwip_comm_default_ip_set(__lwip_dev *lwipx)
{
	u32 sn0;
	sn0=*(vu32*)(0x1FFF7A10);//获取STM32的唯一ID的前24位作为MAC地址后三字节
	//默认远端IP为:192.168.1.100
	lwipx->remoteip[0]=192;	
	lwipx->remoteip[1]=168;
	lwipx->remoteip[2]=1;
	lwipx->remoteip[3]=22;
	//MAC地址设置(高三字节固定为:2.0.0,低三字节用STM32唯一ID)
	lwipx->mac[0]=2;//高三字节(IEEE称之为组织唯一ID,OUI)地址固定为:2.0.0
	lwipx->mac[1]=0;
	lwipx->mac[2]=0;
	lwipx->mac[3]=(sn0>>16)&0XFF;//低三字节用STM32的唯一ID
	lwipx->mac[4]=(sn0>>8)&0XFFF;;
	lwipx->mac[5]=sn0&0XFF; 	
	//默认本地IP为:192.168.1.30
	lwipx->ip[0]=Sys_ParaMeter.IPAddress[0];	
	lwipx->ip[1]=Sys_ParaMeter.IPAddress[1];
	lwipx->ip[2]=Sys_ParaMeter.IPAddress[2];
	lwipx->ip[3]=Sys_ParaMeter.IPAddress[3];
	//默认子网掩码:255.255.255.0
	lwipx->netmask[0]=Sys_ParaMeter.NetMask[0];	
	lwipx->netmask[1]=Sys_ParaMeter.NetMask[1];
	lwipx->netmask[2]=Sys_ParaMeter.NetMask[2];
	lwipx->netmask[3]=Sys_ParaMeter.NetMask[3];
	//默认网关:192.168.1.1
	lwipx->gateway[0]=Sys_ParaMeter.GateWay[0];	
	lwipx->gateway[1]=Sys_ParaMeter.GateWay[1];
	lwipx->gateway[2]=Sys_ParaMeter.GateWay[2];
	lwipx->gateway[3]=Sys_ParaMeter.GateWay[3];	

	lwipx->dhcpstatus=0;//没有DHCP	
}

//用于以太网中断调用
//void lwip_pkt_handle(void)
//{
//	ethernetif_input(&lwip_netif);
//}
u8 lwip_comm_init(void)
{
	struct netif *Netif_Init_Flag;			//调用netif_add()函数时的返回值,用于判断网络初始化是否成功
	struct ip_addr ipaddr;  						//ip地址
	struct ip_addr netmask; 						//子网掩码
	struct ip_addr gw;      						//默认网关 
	if(ETH_Mem_Malloc())return 1;				//内存申请失败
	if(lwip_comm_mem_malloc())return 2;	//内存申请失败	
	if(LAN8720_Init())return 3;					//初始化LAN8720失败 
	tcpip_init(NULL,NULL);							//初始化tcp ip内核,该函数里面会创建tcpip_thread内核任务
	lwip_comm_default_ip_set(&lwipdev);	//设置默认IP等信息

	if((Sys_ParaMeter.DHCPFlag == 1)&&(LWIP_DHCP)) //动态获取IP
	{
		ipaddr.addr = 0;
		netmask.addr = 0;
		gw.addr = 0;	
	}
	else			//使用静态IP
	{
		IP4_ADDR(&ipaddr,lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
		IP4_ADDR(&netmask,lwipdev.netmask[0],lwipdev.netmask[1] ,lwipdev.netmask[2],lwipdev.netmask[3]);
		IP4_ADDR(&gw,lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
		printf("网卡en的MAC地址为:................%d.%d.%d.%d.%d.%d\r\n",lwipdev.mac[0],lwipdev.mac[1],lwipdev.mac[2],lwipdev.mac[3],lwipdev.mac[4],lwipdev.mac[5]);
		printf("静态IP地址........................%d.%d.%d.%d\r\n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
		printf("子网掩码..........................%d.%d.%d.%d\r\n",lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
		printf("默认网关..........................%d.%d.%d.%d\r\n",lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);	
	}
  taskENTER_CRITICAL();           //进入临界区
	Netif_Init_Flag=netif_add(&lwip_netif,&ipaddr,&netmask,&gw,NULL,&ethernetif_init,&ethernet_input);//向网卡列表中添加一个网口
	taskEXIT_CRITICAL();            //退出临界区
	if((Sys_ParaMeter.DHCPFlag == 1)&&(LWIP_DHCP)) //动态获取IP
	{
		lwipdev.dhcpstatus=0;	//DHCP标记为0
		dhcp_start(&lwip_netif);	//开启DHCP服务
	}
	if(Netif_Init_Flag==NULL)return 3;//网卡添加失败 
	else//网口添加成功后,设置netif为默认值,并且打开netif网口
	{
		netif_set_default(&lwip_netif); //设置netif为默认网口
		netif_set_up(&lwip_netif);		//打开netif网口
	}
	return 0;//操作OK.
} 

//如果使能了DHCP
#if LWIP_DHCP

void MyDhcp_Creat(void)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建数据处理任务
    xTaskCreate((TaskFunction_t )MyDhcpCreat_task,     	
                (const char*    )"dhcp_task",   	
                (uint16_t       )DHCP_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )DHCP_TASK_PRIO,	
                (TaskHandle_t*  )&DHCPTask_Handler);   		
    taskEXIT_CRITICAL();            //退出临界区
}
//删除DHCP任务
void MyDhcp_delete(void)
{
	dhcp_stop(&lwip_netif); 		//关闭DHCP
	vTaskDelete(DHCPTask_Handler); //删除任务	
}
//DHCP处理任务
void MyDhcpCreat_task(void *pvParameters)
{
	u32 ip=0,netmask=0,gw=0;
	dhcp_start(&lwip_netif);//开启DHCP 
	lwipdev.dhcpstatus=0;	//正在DHCP
	printf("正在查找DHCP服务器,请稍等...........\r\n");   
	while(1)
	{ 
		printf("正在获取地址...\r\n");
		ip=lwip_netif.ip_addr.addr;		//读取新IP地址
		netmask=lwip_netif.netmask.addr;//读取子网掩码
		gw=lwip_netif.gw.addr;			//读取默认网关 
		if(ip!=0)   					//当正确读取到IP地址的时候
		{
			lwipdev.dhcpstatus=2;	//DHCP成功
 			printf("网卡en的MAC地址为:................%d.%d.%d.%d.%d.%d\r\n",lwipdev.mac[0],lwipdev.mac[1],lwipdev.mac[2],lwipdev.mac[3],lwipdev.mac[4],lwipdev.mac[5]);
			//解析出通过DHCP获取到的IP地址
			lwipdev.ip[3]=(uint8_t)(ip>>24); 
			lwipdev.ip[2]=(uint8_t)(ip>>16);
			lwipdev.ip[1]=(uint8_t)(ip>>8);
			lwipdev.ip[0]=(uint8_t)(ip);
			printf("通过DHCP获取到IP地址..............%d.%d.%d.%d\r\n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
			//解析通过DHCP获取到的子网掩码地址
			lwipdev.netmask[3]=(uint8_t)(netmask>>24);
			lwipdev.netmask[2]=(uint8_t)(netmask>>16);
			lwipdev.netmask[1]=(uint8_t)(netmask>>8);
			lwipdev.netmask[0]=(uint8_t)(netmask);
			printf("通过DHCP获取到子网掩码............%d.%d.%d.%d\r\n",lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
			//解析出通过DHCP获取到的默认网关
			lwipdev.gateway[3]=(uint8_t)(gw>>24);
			lwipdev.gateway[2]=(uint8_t)(gw>>16);
			lwipdev.gateway[1]=(uint8_t)(gw>>8);
			lwipdev.gateway[0]=(uint8_t)(gw);
			printf("通过DHCP获取到的默认网关..........%d.%d.%d.%d\r\n",lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
			ENTInt_OK = 1;
			MyTCPS_Creat();					
			break;
		}
		else if(lwip_netif.dhcp->tries > LWIP_MAX_DHCP_TRIES) //通过DHCP服务获取IP地址失败,且超过最大尝试次数
		{  
			lwipdev.dhcpstatus=0XFF;//DHCP失败.
			//使用静态IP地址
			IP4_ADDR(&(lwip_netif.ip_addr),lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
			IP4_ADDR(&(lwip_netif.netmask),lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
			IP4_ADDR(&(lwip_netif.gw),lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
			printf("DHCP服务超时,使用静态IP地址!\r\n");
			printf("网卡en的MAC地址为:................%d.%d.%d.%d.%d.%d\r\n",lwipdev.mac[0],lwipdev.mac[1],lwipdev.mac[2],lwipdev.mac[3],lwipdev.mac[4],lwipdev.mac[5]);
			printf("静态IP地址........................%d.%d.%d.%d\r\n",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);
			printf("子网掩码..........................%d.%d.%d.%d\r\n",lwipdev.netmask[0],lwipdev.netmask[1],lwipdev.netmask[2],lwipdev.netmask[3]);
			printf("默认网关..........................%d.%d.%d.%d\r\n",lwipdev.gateway[0],lwipdev.gateway[1],lwipdev.gateway[2],lwipdev.gateway[3]);
			MyTCPS_Creat();					
			break;
		}
		delay_ms(500); //延时500ms		
	}
	MyDhcp_delete();//删除DHCP任务 
}
#endif 
void MyTCPS_delete(void)
{
	vTaskDelete(TCPSTask_Handler); //删除任务	
}
void MyTCPS_Creat(void)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建数据处理任务
    xTaskCreate((TaskFunction_t )MyTCPServer_task,     	
                (const char*    )"TCPS_task",   	
                (uint16_t       )TCPS_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )TCPS_TASK_PRIO,	
                (TaskHandle_t*  )&TCPSTask_Handler);   		
    taskEXIT_CRITICAL();            //退出临界区
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

	conn = netconn_new(NETCONN_TCP);  //创建一个TCP链接
	netconn_bind(conn,IP_ADDR_ANY,TCP_SERVER_PORT);  //绑定端口 8号端口
	netconn_listen(conn);  			//进入监听模式
	conn->recv_timeout = 10;  	//禁止阻塞线程 等待10ms
	while (1) 
	{
//		tt++;
		err = netconn_accept(conn,&newconn);  //接收连接请求
//		if(tt%200 == 0)
//			printf("Err code: %d\r\n",err);
		if(err==ERR_OK)newconn->recv_timeout = 10;

		if (err == ERR_OK)    //处理新连接的数据
		{ 
			struct netbuf *recvbuf;

			netconn_getaddr(newconn,&ipaddr,&port,0); //获取远端IP地址和端口号
			
			remot_addr[3] = (uint8_t)(ipaddr.addr >> 24); 
			remot_addr[2] = (uint8_t)(ipaddr.addr>> 16);
			remot_addr[1] = (uint8_t)(ipaddr.addr >> 8);
			remot_addr[0] = (uint8_t)(ipaddr.addr);
			printf("主机%d.%d.%d.%d连接上服务器,主机端口号为:%d\r\n",remot_addr[0], remot_addr[1],remot_addr[2],remot_addr[3],port);
			
			while(1)
			{
				if((tcp_server_flag & LWIP_SEND_DATA) == LWIP_SEND_DATA) //有数据要发送
				{
					err = netconn_write(newconn ,tcp_server_sendbuf,strlen((char*)tcp_server_sendbuf),NETCONN_COPY); //发送tcp_server_sendbuf中的数据
					if(err != ERR_OK)
					{
						printf("发送失败\r\n");
					}
					tcp_server_flag &= ~LWIP_SEND_DATA;
				}
				recv_err = netconn_recv(newconn,&recvbuf);				
				if(recv_err == ERR_OK)  	//接收到数据
				{	
					taskENTER_CRITICAL();           //进入临界区
					memset(tcp_server_recvbuf,0,TCP_SERVER_RX_BUFSIZE);  //数据接收缓冲区清零
					for(q=recvbuf->p;q!=NULL;q=q->next)  //遍历完整个pbuf链表
					{
						//判断要拷贝到TCP_SERVER_RX_BUFSIZE中的数据是否大于TCP_SERVER_RX_BUFSIZE的剩余空间，如果大于
						//的话就只拷贝TCP_SERVER_RX_BUFSIZE中剩余长度的数据，否则的话就拷贝所有的数据
						if(q->len > (TCP_SERVER_RX_BUFSIZE-data_len)) memcpy(tcp_server_recvbuf+data_len,q->payload,(TCP_SERVER_RX_BUFSIZE-data_len));//拷贝数据
						else memcpy(tcp_server_recvbuf+data_len,q->payload,q->len);
						data_len += q->len;  	
						if(data_len > TCP_SERVER_RX_BUFSIZE) break; //超出TCP客户端接收数组,跳出	
					}
					taskEXIT_CRITICAL();            //退出临界区
					data_len=0;  //复制完成后data_len要清零。	
					printf("%s\r\n",tcp_server_recvbuf);  //通过串口发送接收到的数据
					netbuf_delete(recvbuf);
				}
				else if(recv_err == ERR_CLSD)  //关闭连接
				{
					netconn_close(newconn);
					netconn_delete(newconn);
					printf("主机:%d.%d.%d.%d断开与服务器的连接\r\n",remot_addr[0], remot_addr[1],remot_addr[2],remot_addr[3]);
					break;
				}
			}
		}
	}
}
