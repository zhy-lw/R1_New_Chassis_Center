#include "Action_Config.h"

static void ActionTaskEntry(void *pvParameters)
{
	ActionSlot_t *slot = (ActionSlot_t *)pvParameters;
	
	if(slot->func != NULL)
	{
		slot->func(slot->param);
	}
	
	slot->in_use = 0;
	vTaskDelete(NULL);
}

void ActionManager_Init(ActionManager_t *manager)
{
	if(manager == NULL)
		return;
	for(uint8_t i = 0; i < ACTION_MAX_PARALLEL; i++)
	{
		manager->slots[i].in_use = 0;
		manager->slots[i].func = NULL;
		manager->slots[i].param = NULL;
	}
}

BaseType_t ActionManager_Send(ActionManager_t *manager, ActionFunc_t func, void *param)
{
	if(manager == NULL || func == NULL)
		return pdFALSE;
	for(uint8_t i = 0; i < ACTION_MAX_PARALLEL; i++)
	{
		if(!manager->slots[i].in_use)
		{
			manager->slots[i].in_use = 1;
			manager->slots[i].func = func;
			manager->slots[i].param = param;
			
			xTaskCreateStatic(
					ActionTaskEntry,              //任务入口函数
					"Action",                     //任务名字（字符串）
					ACTION_STACK_SIZE,            //栈深度（字数）
					&manager->slots[i],           //传给任务的参数
					ACTION_PRIORITY,              //优先级
					manager->slots[i].stack,     //栈内存地址
					&manager->slots[i].tcb       //TCB内存地址
			);
			
			return pdTRUE;
		}
	}
	return pdFALSE;
}

uint8_t ActionManager_GetCount(ActionManager_t  *manger)
{
	if(manger == NULL)
		return 0;
	uint8_t count = 0;
	for(uint8_t i = 0; i < ACTION_MAX_PARALLEL; i++)
	{
		if(manger->slots[i].in_use)
		{
			count++;
		}
	}
	return count;
}
