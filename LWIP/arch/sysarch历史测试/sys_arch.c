/*
 * Copyright (c) 2001, Swedish Institute of Computer Science.
 * All rights reserved. 
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met: 
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 * 3. Neither the name of the Institute nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
 * SUCH DAMAGE. 
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 * $Id: sys_arch.c,v 1.1.1.1 2003/05/17 05:06:56 chenyu Exp $
 */
#if !NO_SYS
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/err.h"
#include "lwip/lwip_sys.h"
#include "lwip/mem.h"
#include "lwip/stats.h"
#include "sys_arch.h"
#include "lwip_sys.h"
//定义系统使用的超时链表首指针结构
//struct sys_timeouts global_timeouts;
//与系统任务新建函数相关的变量定义

#define SYS_THREAD_MAX 18
static u16_t s_nextthread = 0;
//系统初始化
void sys_init(void)
{
  //currently do nothing
  printf("[Sys_arch] init ok");
}
//创建一个信号量
//*sem：创建的信号量
//count：信号量值
//返回值： ERR_OK 成功，  ERR_MEM失败
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
  *sem = xSemaphoreCreateBinary();	
	
	if(count  == 0)
		xSemaphoreTake(*sem,0);
  if(*sem != NULL) {
		LWIP_ASSERT("[Sys_arch]Creat sem", *sem != NULL);
    return ERR_OK;
  }
	else
	{
		LWIP_ASSERT("[Sys_arch]Error creating sem", *sem == NULL);	
		*sem = SYS_SEM_NULL;
		return ERR_MEM;
	}
}
//释放一个信号量
//*sem：释放的信号量
void sys_sem_free(sys_sem_t *sem)
{
	vSemaphoreDelete(*sem);
	*sem = NULL;
}
//等待一个信号量
//*sem：信号量
//timeout：超时时间
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
  u8_t Err;
  u32_t start,end;
	BaseType_t xHigherPriTaskWoken;	
  LWIP_ASSERT("*sem != NULL", *sem != NULL);

  start  = xTaskGetTickCount();
	if(SCB_ICSR_REG&0xFF)
		Err = xSemaphoreTakeFromISR(*sem,&xHigherPriTaskWoken);
	else
		Err = xSemaphoreTake(*sem, 0);
  
  if (Err == pdTRUE)
	{
		end = xTaskGetTickCount();
		return (u32_t)(end - start);		//将等待时间设置为timeout/2	
	}
  else
		return SYS_ARCH_TIMEOUT;
	
//	if(timeout != 0)
//  {    
//    if(xSemaphoreTake (*sem, timeout) == pdTRUE)
//    {
//      return (xTaskGetTickCount() - start);
//    }
//    else
//    {
//      return SYS_ARCH_TIMEOUT;
//    } 
//  }
//  else
//  {
//    if(xSemaphoreTake (*sem, portMAX_DELAY) != pdTRUE);
//    return (xTaskGetTickCount() - start);
//  }
  
}
//发送一个信号量
//sem：信号量指针
void sys_sem_signal(sys_sem_t *sem)
{
  u8_t Err;
  BaseType_t xHigherPriTaskWoken;	
	if(SCB_ICSR_REG&0xFF) //中断执行
	{
		LWIP_ASSERT("sem != NULL", *sem != NULL);
		xSemaphoreGiveFromISR(*sem,&xHigherPriTaskWoken);
		portYIELD_FROM_ISR(xHigherPriTaskWoken);//如果需要进行一次任务切换		
	}
	else
	{
		Err = xSemaphoreGive(*sem);
		if(Err != pdPASS)
		{
			 printf("[Sys_arch]:signal sem fail\n");
		}	
	}
}

//创建一个消息邮箱
//*mbox：消息邮箱
//size：邮箱大小
//返回值： ERR_OK 创建成功   其他创建失败
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
	if(size > MAX_QUEUE_ENTRIES)
		size = MAX_QUEUE_ENTRIES;
	*mbox = xQueueCreate( size, sizeof( void * )); 
	
	if (*mbox == NULL)
		return ERR_MEM;
	else
		return ERR_OK;
}
//释放并删除一个消息邮箱
void sys_mbox_free(sys_mbox_t *mbox)
{
  vQueueDelete(*mbox);
	*mbox = NULL;
}

const uint32_t NullMessage;
//向消息邮箱中发送一条消息（必须发送成功）
//mbox：消息邮箱
//data：消息内容
void sys_mbox_post(sys_mbox_t *mbox, void *data)
{
	BaseType_t * xHigherPriTaskWorken ;
	if(data == NULL)
		data = (void *)&NullMessage;
	if(SCB_ICSR_REG & 0xFF)	//中断内使用
		while(xQueueSendToBackFromISR(*mbox,&data,xHigherPriTaskWorken)!=pdTRUE);
	else
		while(xQueueSendToBack(*mbox,&data,portMAX_DELAY)!=pdTRUE);		
}
//尝试向消息邮箱中发送一条消息
//mbox：消息邮箱
//data：消息内容
//返回值： ERR_OK 创建成功   其他创建失败
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
	err_t result;
	BaseType_t * xHigherPriTaskWorken ;
	
	if(msg==NULL)
		msg = (void *)&NullMessage;
	if(SCB_ICSR_REG & 0xFF)	//中断内使用
	{
		if (xQueueSendFromISR( *mbox, &msg, xHigherPriTaskWorken) == pdPASS )
		{
			result = ERR_OK;
			//portYIELD_FROM_ISR(xHigherPriTaskWorken); 		
		}
		else
		{
			result = ERR_MEM;
		}
	}	
	else
	{
		 if ( xQueueSend( *mbox, &msg, 0 ) == pdPASS )
			 result = ERR_OK;
		 else
			 result = ERR_MEM;
	}
   return result;
}
//等待邮箱中的消息
//mbox：消息邮箱
//msg：消息内容
//timeout：超时时间 ，如果timeout为0，就一直等待
//返回值：当timeout不为0时，如果成功就返回等待的时间
//				失败的话就返回超时SYS_ARCH_TimeOUT
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
	void *dummyptr;
	BaseType_t * sela;	
	portTickType StartTime, EndTime, Elapsed;

	StartTime = xTaskGetTickCount();

	if ( msg == NULL )
	{
		msg = &dummyptr;
	}
		
	if ( timeout != 0 )
	{
		if(SCB_ICSR_REG&0xFF)	//中断内使用
		{
			if (pdTRUE ==xQueueReceiveFromISR(*mbox, &(*msg), sela ))
			{
				EndTime = xTaskGetTickCount();
				Elapsed = (EndTime - StartTime) * portTICK_RATE_MS;
				return ( Elapsed );
			}
			else
				return SYS_ARCH_TIMEOUT;
		}
		else
		{
			if (pdTRUE == xQueueReceive(*mbox, &(*msg), timeout / portTICK_RATE_MS))
			{
				EndTime = xTaskGetTickCount();
				Elapsed = (EndTime - StartTime) * portTICK_RATE_MS;
				return ( Elapsed );
			}
			else // timed out blocking for message
			{
				*msg = NULL;
				return SYS_ARCH_TIMEOUT;
			}	
		}			
	}
	else // block forever for a message.
	{
		if(SCB_ICSR_REG&0xFF)	//中断内使用		
			while( pdTRUE != xQueueReceiveFromISR(*mbox, &(*msg),sela)); // time is arbitrary
		else
			while( pdTRUE != xQueueReceive( *mbox, &(*msg), portMAX_DELAY ) ); // time is arbitrary			
		EndTime = xTaskGetTickCount();
		Elapsed = (EndTime - StartTime) * portTICK_RATE_MS;
		
		return ( Elapsed ); // return time blocked TODO test	
	}
}
//类似上面的方法
u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
	void *dummyptr;
	BaseType_t * xHigherPriTaskWorken ;
	if ( msg == NULL )
	{
		msg = &dummyptr;
	}

	if(SCB_ICSR_REG & 0xFF)	//中断内使用
	{
		 if ( pdTRUE == xQueueReceiveFromISR( *mbox, &(*msg),xHigherPriTaskWorken ))
		 {
				return ERR_OK;
		 }
		 else
		 {
				return SYS_MBOX_EMPTY;
		 }	
	}
	else
	{
		 if ( pdTRUE == xQueueReceive( *mbox, &(*msg), 0 ) )
		 {
				return ERR_OK;
		 }
		 else
		 {
				return SYS_MBOX_EMPTY;
		 }	
	}
}
/*-----------------------------------------------------------------------------------*/
                                      /* Mutexes*/
/*-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------*/
#if LWIP_COMPAT_MUTEX == 0
/* Create a new mutex*/
err_t sys_mutex_new(sys_mutex_t *mutex) {

  *mutex = xSemaphoreCreateMutex();
	if(*mutex == NULL)
	{
		return ERR_MEM;
	}
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/* Deallocate a mutex*/
void sys_mutex_free(sys_mutex_t *mutex)
{
#if SYS_STATS
      --lwip_stats.sys.mutex.used;
#endif /* SYS_STATS */
			
	vQueueDelete(*mutex);
}
/*-----------------------------------------------------------------------------------*/
/* Lock a mutex*/
void sys_mutex_lock(sys_mutex_t *mutex)
{
	sys_arch_sem_wait(*mutex, 0);
}

/*-----------------------------------------------------------------------------------*/
/* Unlock a mutex*/
void sys_mutex_unlock(sys_mutex_t *mutex)
{
	xSemaphoreGive(*mutex);
}
#endif /*LWIP_COMPAT_MUTEX*/
//函数功能：新建一个进程，在整个系统中只会被调用一次
//sys_thread_t sys_thread_new(char *name, void (* thread)(void *arg), void *arg, int stacksize, int prio);
//prio 1~10, is kept for network 
//TCPIP_THREAD_PRIO    1   -> lwip thead prio
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread , void *arg, int stacksize, int prio)
{
		xTaskHandle CreatedTask;
		int result;

   if(s_nextthread < SYS_THREAD_MAX)
   {
		 taskENTER_CRITICAL();
     result = xTaskCreate( thread,  name, stacksize, arg, prio, &CreatedTask );
		 taskEXIT_CRITICAL();
	   if(result == pdPASS)
	   {
		   return CreatedTask;
	   }
	   else
	   {
		   return NULL;
	   }
   }
   else
   {
      return NULL;
   }
}
//进入临界区
sys_prot_t sys_arch_protect(void)
{
	if(SCB_ICSR_REG & 0xFF)	//中断内使用
	{
//		printf("1\r\n");
		return taskENTER_CRITICAL_FROM_ISR();		
	}
	else										//非中断内使用
	{
//		printf("2\r\n");
		taskENTER_CRITICAL();
		return 0;
	}
}
//退出临界区
void sys_arch_unprotect(sys_prot_t pval)
{
	if(SCB_ICSR_REG&0xFF)	//中断内使用
	{
//		printf("3\r\n");
		taskEXIT_CRITICAL_FROM_ISR(pval);
	}
	else									//非中断内使用
	{
//		printf("4\r\n");		
		taskEXIT_CRITICAL();
	}
}
//当前系统节拍数
u32_t sys_now(void)
{
	u32_t lwip_time;
	lwip_time = xTaskGetTickCount();
	return lwip_time;
}
#endif
