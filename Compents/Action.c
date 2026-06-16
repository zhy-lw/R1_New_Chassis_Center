#include "Action.h"
#include "Navigation.h"
#include "Task_Init.h"

extern ChassisMode Mode;
extern PurePursuitController Pure_Handle;
extern Remote_Handle_t Remote_Control;
extern uint8_t Action_Sign;
extern uint8_t run_count;
extern uint8_t Two_Three_Sign;
extern uint8_t One_Four_Sign;

extern float cloud_pos_target;//云台
extern float exp_height_3508;//升降
extern float over_turn_pos;//翻转
extern float flexible_pos;//伸缩

ActionManager_t g_mgr;

void Auto_Rod_Retrieval_Action(void *param)
{
	TickType_t last_wake_time = xTaskGetTickCount();
	static uint32_t last_count = 0;
	
	while(1)
	{
		if(run_count == 0)
		{
			if(last_count != 1)
			{
				last_count = 1;
				PurePursuit_SetTarget(&Pure_Handle, 0.93f, 2.085f, 0.0f);//第一个点
				exp_height_3508 = 400;
				Action_Sign = 1;//爪1和爪2张开，后05倾斜
			}
		}
		
		if(run_count == 1)
		{
			if(last_count != 2)
			{
				last_count = 2;
				PurePursuit_SetTarget(&Pure_Handle, 0.617f, 2.085f, 0.0f);//第二个点
			}
		}
		
		if(run_count == 2)
		{
			if(last_count != 3)
			{
				last_count = 3;
				Action_Sign = 2;//闭合爪2,闭合完成后将后杆竖直
				vTaskDelay(2500);
				Pure_Handle.max_velocity = 0.6f;
				PurePursuit_SetTarget(&Pure_Handle, 1.1f, 2.085f, 0.0f);//第三个点
				Action_Sign = 3;//将05竖直放杆，然后闭合爪1
			}
		}
		
		if(run_count == 3)
		{
			if(last_count != 4)
			{
				last_count = 4;
				PurePursuit_SetTarget(&Pure_Handle, 1.1f, 2.085f, 1.55f);//第三个点
			}
		}
		
		if(run_count == 4)//0.9 3.68 1.55  0.41..
		{
			if(last_count != 5)
			{
				last_count = 5;
				PurePursuit_SetTarget(&Pure_Handle, 1.1f, 3.68f, 1.55f);//第四个点
			}
		}
		
		if(run_count == 5)//0.9 3.68 1.55  0.41..
		{
			if(last_count != 6)
			{
				last_count = 6;
				PurePursuit_SetTarget(&Pure_Handle, 0.47f, 3.68f, 1.55f);//第四个点
				break;
			}
		}
		
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
	}
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
