#ifndef __SYS_ARCH_H__
#define __SYS_ARCH_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "sys.h"
//�궨����̴���
typedef xTaskHandle sys_thread_t;	
typedef SemaphoreHandle_t sys_mutex_t;
#define MAX_QUEUES        		10	// ��Ϣ���������
#define MAX_QUEUE_ENTRIES 		20	// ÿ����Ϣ����Ĵ�С
//�궨���ź���������ͺͺ�
typedef SemaphoreHandle_t sys_sem_t;
#define SYS_SEM_NULL (xSemaphoreHandle)0
#define sys_sem_valid(sem) ((*sem)!=(xSemaphoreHandle)0)
#define sys_sem_set_invalid(sem) ((*sem)==(xSemaphoreHandle)0)
#define SYS_DEFAULT_THREAD_STACK_DEPTH configMINIMAL_STACK_SIZE

//ʹ��freertos���ж����������ĺ������
typedef QueueHandle_t sys_mbox_t;
#define SYS_MBOX_NULL (xQueueHandle)0
#define sys_mbox_valid(mbox) ((*mbox)!=(xQueueHandle)0)
#define sys_mbox_set_invalid(mbox) ((*mbox)==(xQueueHandle)0)
#define SYS_THREAD_NULL NULL
/* DWORD (thread id) is used for sys_thread_t but we won't include windows.h */
#define archMESG_QUEUE_LENGTH	(6)

#endif
