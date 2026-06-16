#ifndef __ACTION_CONFIG_H__
#define __ACTION_CONFIG_H__

#include "stdint.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define ACTION_MAX_PARALLEL   20    // 最大并行任务数
#define ACTION_STACK_SIZE     350  // 单个任务栈大小
#define ACTION_PRIORITY       4    // 动作执行优先级

typedef void (*ActionFunc_t)(void *param);

typedef struct{
	uint8_t in_use;//是否占用
	ActionFunc_t func;//动作函数指针
	void *param;//传给动作函数的参数
	
	//静态内存
	StaticTask_t tcb;
	StackType_t stack[ACTION_STACK_SIZE];
}ActionSlot_t;

typedef struct{
	ActionSlot_t slots[ACTION_MAX_PARALLEL];
}ActionManager_t;

void ActionManager_Init(ActionManager_t *manager);
BaseType_t ActionManager_Send(ActionManager_t *manager, ActionFunc_t func, void *param);
uint8_t ActionManager_GetCount(ActionManager_t  *manger);

#endif
