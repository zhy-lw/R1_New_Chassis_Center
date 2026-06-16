#include "Action.h"
#include "Navigation.h"
#include "Task_Init.h"

extern ChassisMode Mode;
extern PurePursuitController Pure_Handle;
extern Remote_Handle_t Remote_Control;
ActionManager_t g_mgr;

void Auto_Rod_Retrieval_Action(void *param)
{
		PurePursuit_SetTarget(&Pure_Handle,1.0f, 2.655f, 0.0f);
}

TaskHandle_t Action_Handle;
void Action(void *param)
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	ActionManager_Init(&g_mgr);
	
	while(1)
	{
		if(Mode == AUTO)
		{
			if(Remote_Control.First.Left_Key_Up == 1 && Remote_Control.Second.Left_Key_Up == 0)
			{
				ActionManager_Send(&g_mgr, Auto_Rod_Retrieval_Action, NULL);
			}
		}
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(10));
	}
}
