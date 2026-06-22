#include "Action.h"
#include "Navigation.h"
#include "Task_Init.h"
#include "dataFrame.h"

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
extern float exp_flexible_len;//伸缩
extern GPIO_PinState GPIO_Pin_State_AirPump;
extern GPIO_PinState GPIO_Pin_State_Valve;
extern PackControl_t recv_pack;
uint8_t flag_Pump = 0;
uint8_t flag_Value = 0;

ActionManager_t g_mgr;

//声明
uint8_t GetReturnValue(void);
uint8_t MerLin_State_flag = 0;
void Auto_Rod_Retrieval_Action(void *param)//一区取杆儿
{
	TickType_t last_wake_time = xTaskGetTickCount();
	static uint32_t last_count = 0;
	
	while(1)
	{
		if(run_count == 0)
		{
			if(last_count != 1)
			{
				PurePursuit_SetTarget(&Pure_Handle, 0.85f, 2.07f, 0.0f);//第一个点
				exp_height_3508 = 400;
				Action_Sign = 1;//爪1和爪2张开，后05倾斜
				last_count = 1;
			}
		}
		
		if(run_count == 1)
		{
			if(last_count != 2)
			{
				PurePursuit_SetTarget(&Pure_Handle, 0.615f, 2.07f, 0.0f);//第二个点
				last_count = 2;
			}
		}
		
		if(run_count == 2)
		{
			if(last_count != 3)
			{
				Pure_Handle.max_velocity = 0.65f;
				Action_Sign = 2;//闭合爪2,闭合完成后将后杆竖直
				vTaskDelay(3000);
				PurePursuit_SetTarget(&Pure_Handle, 1.2f, 2.07f, 0.0f);//第三个点
				Action_Sign = 3;//将05竖直放杆，然后闭合爪1
				last_count = 3;
			}
		}
		
		if(run_count == 3)
		{
			if(last_count != 4)
			{
				Pure_Handle.target_theta = 1.55f;
				vTaskDelay(1000);
				PurePursuit_SetTarget(&Pure_Handle, 0.85f, 3.68f, 1.55f);//第四个点
				last_count = 4;
			}
		}
		
		if(run_count == 4)
		{
			if(last_count != 5)
			{
				Pure_Handle.max_velocity = 1.0f;
				PurePursuit_SetTarget(&Pure_Handle, 0.39f, 3.68f, 1.55f);//第五个点
				last_count = 5;
				g_mgr.slots[0].in_use = 0;
				vTaskDelete(NULL);
			}
		}
		
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
	}
}

void Auto_KFS_Retrieval_Action(void *param)//二区取块儿
{
	TickType_t last_wake_time = xTaskGetTickCount();
	
	run_count = 0;
	static uint32_t last_count = 0;	
	while(1)
	{
		if(run_count == 0)
		{
			if(last_count != 1)
			{
				PurePursuit_SetTarget(&Pure_Handle, 1.5f, 3.68f, 1.55f);//第一个点
				exp_flexible_len = 350;
				One_Four_Sign = 1;//左张开舵机转，延时2000闭合左爪
				Action_Sign = 4;//前爪张开 ,后05，-1.6
				last_count = 1;
			}
		}
		
		if(run_count == 1)
		{
			if(last_count != 2)
			{
				PurePursuit_SetTarget(&Pure_Handle, 2.56f, 0.605f, 1.55f);//第二个点
				exp_flexible_len = 100;
				exp_height_3508 = 120;
				over_turn_pos = 1.5;
				last_count = 2;
			}
		}
		
		if(run_count == 2)
		{
			if(last_count != 3)
			{
				Pure_Handle.max_velocity = 0.8f;
				PurePursuit_SetTarget(&Pure_Handle, 3.89f, 0.605f, 1.55f);//第三个点
				last_count = 3;
			}
		}
		
		if(run_count == 3)
		{
			if(last_count != 4)
			{
				PurePursuit_SetTarget(&Pure_Handle, 3.89f, 0.81f, 1.55f);//第三个点
				GPIO_Pin_State_AirPump = 1;
				GPIO_Pin_State_Valve = 1;
				last_count = 4;
			}
		}
		
		if(run_count == 4)
		{
			if(last_count != 5)
			{
				exp_flexible_len = 400;
				vTaskDelay(300);
				exp_height_3508 = 350;
				vTaskDelay(300);
				exp_flexible_len = 100;
				PurePursuit_SetTarget(&Pure_Handle, 3.89f, 0.605f, 1.55f);//第四个点
				last_count = 5;
			}
		}
		
		if(run_count == 5)
		{
			if(last_count != 6)
			{
				PurePursuit_SetTarget(&Pure_Handle, 6.29f, 0.605f, 1.55f);//第五个点
				exp_height_3508 = 300;
				vTaskDelay(400);
				cloud_pos_target = -1.427;
				vTaskDelay(700);
				exp_flexible_len = 40;
				vTaskDelay(200);
				exp_height_3508 = 170;
				vTaskDelay(400);
				cloud_pos_target = -1.6;
				vTaskDelay(200);
				GPIO_Pin_State_Valve = 0;
				Two_Three_Sign = 1;
				vTaskDelay(200);
				exp_height_3508 = 500;
				vTaskDelay(500);
				cloud_pos_target = 0;
				last_count = 6;
			}
		}
		
		if(run_count == 6)
		{
			if(last_count != 7)
			{
				PurePursuit_SetTarget(&Pure_Handle, 6.29f, 0.81f, 1.55f);//第六个点
				exp_height_3508 = 120;
				GPIO_Pin_State_Valve = 1;
				last_count = 7;
			}
		}
		
		if(run_count == 7)
		{
			if(last_count != 8)
			{
				exp_flexible_len = 400;
				vTaskDelay(300);
				exp_height_3508 = 500;
				vTaskDelay(300);
				exp_flexible_len = 100;
				PurePursuit_SetTarget(&Pure_Handle, 6.29f, 0.65f, 1.55f);//第七个点
				last_count = 8;
			}
		}
		
		if(run_count == 8)
		{
			if(last_count != 9)
			{
				Pure_Handle.max_velocity = 0.7f;
				PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.65f, 1.55f);//第八个点
				last_count = 9;
			}
		}
		
		if(run_count == 9)
		{
			if(last_count != 10)
			{
				PurePursuit_SetTarget(&Pure_Handle, 11.3f, 0.85f, 1.55f);//第九个点
				last_count = 10;
				g_mgr.slots[0].in_use = 0;
				vTaskDelete(NULL);
			}
		}
		
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
	}
}

extern MerLin_Pack_t MerLin_Pack;
// [2]侧 2.759 4.146 0
// [1]侧 2.767 2.923 0   (h_3508 50)
// [0]侧 2.765 1.722 0   (h_3028 250)
// [3]   5.110 0.780 1.5 (h_3508 50)
// [6]   6.307 0.780 1.5 
// [9]   7.546 0.780 1.5
// [10]  8.445 3.102 -3.102
// [5]   4.935 5.260 -1.5
// [8]   6.145 5.250 -1.5
// [11]  7.352 5.247 -1.5

uint8_t index_no_fetch = 0;//那个没拿索引
uint8_t index_State_Two = 0;//2侧状态
uint8_t index_State_Three = 0;//1侧状态
void Auto_KFS_Action(void *param)
{
	TickType_t last_wake_time = xTaskGetTickCount();

	run_count = 0;
	static uint32_t last_count = 0;

	while (1)
	{
		if (run_count == 0)
		{
			if (last_count != 1)
			{
				PurePursuit_SetTarget(&Pure_Handle, 1.5f, 3.68f, 1.55f); // 第一个点
				exp_flexible_len = 350;
				One_Four_Sign = 1; // 左张开舵机转，延时2000闭合左爪
				Action_Sign = 4;   // 前爪张开 ,后05，-1.6
				MerLin_State_flag = GetReturnValue();
				last_count = 1;
			}
		}
		
		if (MerLin_State_flag == 1) // 0,1,2有两个
		{
			if (run_count == 1)
			{
				if (MerLin_Pack.MerLin[0] != 1)
				{
					// 1 和 2 为1，最大是 2,先去取第三个块
					if (last_count != 2)
					{
						Pure_Handle.max_velocity = 0.5f;
						Pure_Handle.target_theta = 0.0f;
						vTaskDelay(700);
						PurePursuit_SetTarget(&Pure_Handle, 2.759f, 4.15f, 0.0f); // 去取第三个格的块儿,取第二个格的块儿是2.767 2.923
						exp_flexible_len = 100;
						exp_height_3508 = 250; // 高度400
						over_turn_pos = 1.5;   // 升起并翻转05
						last_count = 2;

						index_no_fetch = 1;
					}
				}
				else if (MerLin_Pack.MerLin[1] != 1)
				{
					// 0 和 2 为1，最大是 2先去取2先去取第三个块
					if (last_count != 2)
					{
						Pure_Handle.max_velocity = 0.5f;
						Pure_Handle.target_theta = 0.0f;
						vTaskDelay(700);
						PurePursuit_SetTarget(&Pure_Handle, 2.759f, 4.15f, 0.0f); // 去取第三个格的块儿，取第一个格的块儿是2.765 1.722
						exp_flexible_len = 100;
						exp_height_3508 = 250; // 高度400
						over_turn_pos = 1.5;   // 升起并翻转05
						last_count = 2;

						index_no_fetch = 2;
					}
				}
				else if (MerLin_Pack.MerLin[2] != 1)
				{
					// 0 和 1 为1，最大是 1先去取1先去取第二个块
					if (last_count != 2)
					{
						Pure_Handle.max_velocity = 0.5f;
						Pure_Handle.target_theta = 0.0f;
						vTaskDelay(700);
						PurePursuit_SetTarget(&Pure_Handle, 2.767f, 2.90f, 0.0f); // 去取第二个格的块儿，取第一个格的块儿是2.765 1.722
						exp_flexible_len = 100;
						exp_height_3508 = 50; // 高度200
						over_turn_pos = 1.5;  // 升起并翻转05
						last_count = 2;

						index_no_fetch = 3;
					}
				}
			}

			if (run_count == 2)
			{
				if (index_no_fetch == 1)
				{
					if (last_count != 3)
					{
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 40;
						PurePursuit_SetTarget(&Pure_Handle, 2.767f, 2.90f, 0.0f); // 第二个块
						cloud_pos_target = -1.35;
						vTaskDelay(700);
						exp_height_3508 = 270;
						vTaskDelay(700);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 3;
					}
				}

				if (index_no_fetch == 2)
				{
					if (last_count != 3)
					{
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 40;
						PurePursuit_SetTarget(&Pure_Handle, 2.765f, 1.722f, 0.0f); // 取第一个块
						cloud_pos_target = -1.35;
						vTaskDelay(700);
						exp_height_3508 = 270;
						vTaskDelay(700);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 3;
					}
				}

				if (index_no_fetch == 3)
				{
					if (last_count != 3)
					{
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 40;
						PurePursuit_SetTarget(&Pure_Handle, 2.765f, 1.722f, 0.0f); // 取第一个块
						cloud_pos_target = -1.35;
						vTaskDelay(700);
						exp_height_3508 = 270;
						vTaskDelay(700);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 3;
					}
				}
			}

			if (run_count == 3)
			{
				if(index_no_fetch == 1)
				{
					if (last_count != 4)
					{
						vTaskDelay(300);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						exp_height_3508 = 50;
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 2.56f, 0.605f, 0.0f); // 准备进有斜坡的一边
						last_count = 4;
					}
				}
				
				if(index_no_fetch == 2)
				{
					if (last_count != 4)
					{
						vTaskDelay(300);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						exp_height_3508 = 250;
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 2.56f, 0.605f, 0.0f); // 准备进有斜坡的一边
						last_count = 4;
					}
				}

				if(index_no_fetch == 3)
				{
					if (last_count != 4)
					{
						vTaskDelay(300);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						exp_height_3508 = 250;
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(500);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 2.56f, 0.56f, 0.0f); // 准备进有斜坡的一边
						last_count = 4;
					}
				}
			}

			if (run_count == 4)
			{
				if (last_count != 5)
				{
					Pure_Handle.target_theta = 1.55f;
					vTaskDelay(1200);
					Pure_Handle.max_velocity = 0.6f;
					PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
					last_count = 5;
				}
			}

			if (run_count == 5)
			{
				if (last_count != 6)
				{
					Pure_Handle.max_velocity = 1.0f;
					PurePursuit_SetTarget(&Pure_Handle, 11.3f, 0.85f, 1.55f); //上斜坡
					last_count = 6;
					g_mgr.slots[0].in_use = 0;
					vTaskDelete(NULL);
				}
			}
		}
		
		if (MerLin_State_flag == 2) // 3,6,9有两个
		{
			if(run_count == 1)
			{
				if(last_count != 2)
				{
					PurePursuit_SetTarget(&Pure_Handle, 2.56f, 0.605f, 1.55f);//第二个点
					exp_flexible_len = 100;
					exp_height_3508 = 250;
					over_turn_pos = 1.5;
					last_count = 2;
				}
			}
			
			if(run_count == 2)
			{
				if(MerLin_Pack.MerLin[3] != 1)//吸6，9
				{
					//先去吸6，然后吸9
					if(last_count != 3)
					{
						Pure_Handle.max_velocity = 0.68f;
						PurePursuit_SetTarget(&Pure_Handle, 6.37f, 0.80f, 1.55f);//去吸6
						vTaskDelay(500);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						index_State_Two = 1;//吸9
						last_count = 3;
					}
				}
				
				if(MerLin_Pack.MerLin[6] != 1)//吸3，9
				{
					if(last_count != 3)
					{
						Pure_Handle.max_velocity = 0.7f;
						PurePursuit_SetTarget(&Pure_Handle, 5.11f, 0.80f, 1.55f);//去吸3
						vTaskDelay(500);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						index_State_Two = 2;//吸9
						last_count = 3;
					}
				}
				
				if(MerLin_Pack.MerLin[9] != 1)//吸3，6
				{
					if(last_count != 3)
					{
						Pure_Handle.max_velocity = 0.7f;
						PurePursuit_SetTarget(&Pure_Handle, 5.11f, 0.80f, 1.55f);//去吸3
						vTaskDelay(500);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						index_State_Two = 3;//吸6
						last_count = 3;
					}
				}
			}
			
			if(run_count == 3)
			{
				if(index_State_Two == 1)
				{
					if(last_count != 4)
					{
						exp_height_3508 = 250;//高度400
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 40;
						PurePursuit_SetTarget(&Pure_Handle, 7.546f, 0.80f, 1.55f);//吸9
						cloud_pos_target = -1.3;
						vTaskDelay(700);
						exp_height_3508 = 270;
						vTaskDelay(700);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 4;
					}
				}

				if(index_State_Two == 2)
				{
					if(last_count != 4)
					{
						exp_height_3508 = 50;//高度200
						vTaskDelay(300);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 40;
						PurePursuit_SetTarget(&Pure_Handle, 7.546f, 0.80f, 1.55f);//吸9
						cloud_pos_target = -1.3;
						vTaskDelay(700);
						exp_height_3508 = 270;
						vTaskDelay(700);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 4;
					}
				}

				if(index_State_Two == 3)
				{
					if(last_count != 4)
					{
						exp_height_3508 = 50;//高度200
						vTaskDelay(300);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 40;
						PurePursuit_SetTarget(&Pure_Handle, 6.37f, 0.80f, 1.55f);//吸6
						cloud_pos_target = -1.3;
						vTaskDelay(700);
						exp_height_3508 = 270;
						vTaskDelay(700);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 4;
					}
				}
			}

			if (run_count == 4)
			{
				if(index_State_Two == 1)
				{
					if (last_count != 5)
					{
						vTaskDelay(300);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						exp_height_3508 = 50;//吸9
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
						last_count = 5;
					}
				}
				if(index_State_Two == 2)
				{
					if (last_count != 5)
					{
						vTaskDelay(300);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						exp_height_3508 = 50;//吸9
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
						last_count = 5;
					}
				}

				if(index_State_Two == 3)
				{
					if (last_count != 5)
					{
						vTaskDelay(300);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						exp_height_3508 = 250;//吸6
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
						last_count = 5;
					}
				}
			}

			if (run_count == 5)
			{
				if (last_count != 6)
				{
					Pure_Handle.max_velocity = 1.0f;
					PurePursuit_SetTarget(&Pure_Handle, 11.3f, 0.85f, 1.55f); //上斜坡
					last_count = 6;
					g_mgr.slots[0].in_use = 0;
					vTaskDelete(NULL);
				}
			}
		}
		
		if (MerLin_State_flag == 3) // 5,8,11有两个
		{
			if(run_count == 1)
			{
				if(last_count != 2)
				{
					Pure_Handle.max_velocity = 0.45f;
					Pure_Handle.target_theta = 0.0f;
					vTaskDelay(1700);
					PurePursuit_SetTarget(&Pure_Handle, 2.56f, 5.35f, 0.0f);
					exp_flexible_len = 100;
					exp_height_3508 = 450;//高度600
					over_turn_pos = 1.5;
					last_count = 2;
				}
			}
			
			if(run_count == 2)
			{
				if(MerLin_Pack.MerLin[5] != 1)
				{
					if(last_count != 3)
					{
						Pure_Handle.max_velocity = 0.4f;
						Pure_Handle.target_theta = -1.55f;
						vTaskDelay(1700);
						PurePursuit_SetTarget(&Pure_Handle, 6.15f, 5.26f, -1.55f);//吸8
						vTaskDelay(500);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						index_State_Three = 1;
						last_count = 3;
					}
				}
				
				if(MerLin_Pack.MerLin[8] != 1)
				{
					if(last_count != 3)
					{
						Pure_Handle.max_velocity = 0.4f;
						Pure_Handle.target_theta = -1.55f;
						vTaskDelay(1700);
						PurePursuit_SetTarget(&Pure_Handle,  4.90f, 5.26f, -1.55f);//吸5
						vTaskDelay(500);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						index_State_Three = 2;
						last_count = 3;
					}
				}
				
				if(MerLin_Pack.MerLin[11] != 1)
				{
					if(last_count != 3)
					{
						Pure_Handle.max_velocity = 0.4f;
						Pure_Handle.target_theta = -1.55f;
						vTaskDelay(1700);
						PurePursuit_SetTarget(&Pure_Handle,  4.90f, 5.26f, -1.55f);//吸5
						vTaskDelay(500);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						index_State_Three = 3;
						last_count = 3;
					}
				}
			}

			if(run_count == 3)
			{
				if(index_State_Three == 1)
				{
					if(last_count != 4)
					{
						Pure_Handle.max_velocity = 0.7f;
						exp_height_3508 = 250;//高度400
						vTaskDelay(300);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 40;
						PurePursuit_SetTarget(&Pure_Handle, 6.15f, 5.58f, -1.55f);//后退
						last_count = 4;
					}
				}

				if(index_State_Three == 2)
				{
					if(last_count != 4)
					{
						Pure_Handle.max_velocity = 0.7f;
						exp_height_3508 = 450;//高度600
						vTaskDelay(300);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 40;
						PurePursuit_SetTarget(&Pure_Handle, 4.90f, 5.58f, -1.55f);//后退
						last_count = 4;
					}
				}

				if(index_State_Three == 3)
				{
					if(last_count != 4)
					{
						Pure_Handle.max_velocity = 0.7f;
						exp_height_3508 = 450;//高度600
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 40;
						PurePursuit_SetTarget(&Pure_Handle, 4.90f, 5.60f, -1.55f);//后退
						last_count = 4;
					}
				}
			}

			if(run_count == 4)
			{
				if(index_State_Three == 1)
				{
					if (last_count != 5)
					{
						cloud_pos_target = -1.35;
						vTaskDelay(700);
						exp_height_3508 = 270;
						vTaskDelay(200);
						PurePursuit_SetTarget(&Pure_Handle, 7.33f, 5.58f, -1.55f);//吸11
						vTaskDelay(500);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 5;
					}
				}

				if(index_State_Three == 2)
				{
					if (last_count != 5)
					{
						cloud_pos_target = -1.35;
						vTaskDelay(700);
						exp_height_3508 = 270;
						vTaskDelay(200);
						PurePursuit_SetTarget(&Pure_Handle, 7.33f, 5.58f, -1.55f);//吸11
						vTaskDelay(500);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 5;
					}
				}

				if(index_State_Three == 3)
				{
					if (last_count != 5)
					{
						cloud_pos_target = -1.35;
						vTaskDelay(700);
						exp_height_3508 = 270;
						vTaskDelay(200);
						PurePursuit_SetTarget(&Pure_Handle, 6.15f, 5.58f, -1.55f);//吸11
						vTaskDelay(500);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 5;
					}
				}
			}

			if(run_count == 5)
			{
				if(index_State_Three == 1)
				{
					if (last_count != 6)
					{
						PurePursuit_SetTarget(&Pure_Handle, 7.352f, 5.26f, -1.55f);//吸11
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						last_count = 6;
					}
				}

				if(index_State_Three == 2)
				{
					if (last_count != 6)
					{
						PurePursuit_SetTarget(&Pure_Handle, 7.33f, 5.26f, -1.55f);//吸11
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						last_count = 6;
					}
				}

				if(index_State_Three == 3)
				{
					if (last_count != 6)
					{
						PurePursuit_SetTarget(&Pure_Handle, 6.15f, 5.26f, -1.55f);//吸8
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						last_count = 6;
					}
				}
			}

			if(run_count == 6)
			{
				if(index_State_Three == 1)
				{
					if (last_count != 7)
					{
						exp_height_3508 = 50;//吸11
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.70f, 5.57f, -1.55f);//准备准备进斜坡
						last_count = 7;
					}
				}

				if(index_State_Three == 2)
				{
					if (last_count != 7)
					{
						exp_height_3508 = 50;//吸11
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.70f, 5.57f, -1.55f);//准备准备进斜坡
						last_count = 7;
					}
				}

				if(index_State_Three == 3)
				{
					if (last_count != 7)
					{
						exp_height_3508 = 250;//吸8
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.70f, 5.57f, -1.55f); //准备准备进斜坡
						last_count = 7;
					}
				}
			}

			if (run_count == 7)
			{
				if (last_count != 8)
				{
					Pure_Handle.max_velocity = 0.8f;
					PurePursuit_SetTarget(&Pure_Handle, 8.70f, 0.70f, -1.55f); // 准备上斜坡
					last_count = 8;
				}
			}

			if(run_count == 8)
			{
				if (last_count != 9)
				{
					Pure_Handle.target_theta = 0.0f;
					vTaskDelay(1500);
					PurePursuit_SetTarget(&Pure_Handle, 11.3f, 0.85f, 0.0f); //上斜坡
					last_count = 9;
				}
			}

			if(run_count == 9)
			{
				if (last_count != 10)
				{
					Pure_Handle.target_theta = 1.55f;
					last_count = 10;
					g_mgr.slots[0].in_use = 0;
					vTaskDelete(NULL);
				}
			}
		}

		if (MerLin_State_flag == 4) // 0,1,2有一个，3，6，9有一个
		{
			if(run_count == 1)
			{
				if(last_count != 2)
				{
					Pure_Handle.max_velocity = 0.4f;
					Pure_Handle.target_theta = 0.0f;
					vTaskDelay(1000);
					if(MerLin_Pack.MerLin[0] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 2.765f, 1.722f, 0.0f); // 去取第一个格的块儿
						exp_height_3508 = 250; // 高度400
					}
					else if(MerLin_Pack.MerLin[1] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 2.767f, 2.90f, 0.0f); // 去取第2个格的块
						exp_height_3508 = 50; // 高度200
					}
					else if(MerLin_Pack.MerLin[2] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 2.759f, 4.15f, 0.0f); // 去取第三个格的块儿
						exp_height_3508 = 250; // 高度400
					}
					exp_flexible_len = 100;
					over_turn_pos = 1.5;
					last_count = 2;
				}
			}

			if(run_count == 2)
			{
				if(last_count != 3)
				{
					GPIO_Pin_State_AirPump = 1;
					GPIO_Pin_State_Valve = 1;
					exp_flexible_len = 400;
					vTaskDelay(300);
					exp_height_3508 = 500;
					vTaskDelay(300);
					exp_flexible_len = 40;
					PurePursuit_SetTarget(&Pure_Handle, 2.56f, 0.605f, 0.0f); // 准备进有斜坡的一边
					cloud_pos_target = -1.35;
					vTaskDelay(700);
					exp_height_3508 = 270;
					vTaskDelay(700);
					cloud_pos_target = -1.6;
					vTaskDelay(200);
					GPIO_Pin_State_Valve = 0;
					Two_Three_Sign = 1;
					vTaskDelay(200);
					exp_height_3508 = 500;
					vTaskDelay(500);
					cloud_pos_target = 0;
					last_count = 3;
				}
			}

			if(run_count == 3)
			{
				if(last_count != 4)
				{
					Pure_Handle.target_theta = 1.55f;
					vTaskDelay(1700);
					Pure_Handle.max_velocity = 0.67f;
					if(MerLin_Pack.MerLin[3] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 5.11f, 0.80f, 1.55f);//吸3
						GPIO_Pin_State_Valve = 1;
					}
					else if(MerLin_Pack.MerLin[6] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 6.37f, 0.80f, 1.55f); //吸6
						GPIO_Pin_State_Valve = 1;
					}
					else if(MerLin_Pack.MerLin[9] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 7.546f, 0.80f, 1.55f);//吸9
						GPIO_Pin_State_Valve = 1;
					}
					exp_height_3508 = 250;//高度400
					last_count = 4;
				}
			}

			if(run_count == 4)
			{
				if (last_count != 5)
				{
					if(MerLin_Pack.MerLin[3] == 1)
					{
						exp_height_3508 = 50;//吸3
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
					}
					else if(MerLin_Pack.MerLin[6] == 1)
					{
						exp_height_3508 = 250;//吸6
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
					}
					else if(MerLin_Pack.MerLin[9] == 1)
					{
						exp_height_3508 = 50;//吸9
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
					}
					last_count = 5;
				}
			}

			if(run_count == 5)
			{
				if (last_count != 6)
				{
					Pure_Handle.max_velocity = 1.0f;
					PurePursuit_SetTarget(&Pure_Handle, 11.3f, 0.85f, 1.55f); //上斜坡
					last_count = 6;
					g_mgr.slots[0].in_use = 0;
					vTaskDelete(NULL);
				}
			}
		}

		if (MerLin_State_flag == 5) // 0,1,2有一个，5，8，11有一个
		{
			if (MerLin_Pack.MerLin[5] != 1)
			{
				if (run_count == 1)
				{
					if (last_count != 2)
					{
						Pure_Handle.max_velocity = 0.4f;
						Pure_Handle.target_theta = 0.0f;
						vTaskDelay(1000);
						if (MerLin_Pack.MerLin[0] == 1)
						{
							PurePursuit_SetTarget(&Pure_Handle, 2.765f, 1.722f, 0.0f); // 去取第一个格的块儿
							exp_height_3508 = 250;									   // 高度400
						}
						else if (MerLin_Pack.MerLin[1] == 1)
						{
							PurePursuit_SetTarget(&Pure_Handle, 2.767f, 2.90f, 0.0f); // 去取第2个格的块
							exp_height_3508 = 50;									  // 高度200
						}
						else if (MerLin_Pack.MerLin[2] == 1)
						{
							PurePursuit_SetTarget(&Pure_Handle, 2.759f, 4.15f, 0.0f); // 去取第三个格的块儿
							exp_height_3508 = 250;									  // 高度400
						}
						exp_flexible_len = 100;
						over_turn_pos = 1.5;
						last_count = 2;
					}
				}

				if (run_count == 2)
				{
					if (last_count != 3)
					{
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 40;
						PurePursuit_SetTarget(&Pure_Handle, 2.56f, 5.35f, 0.0f); // 无斜坡
						cloud_pos_target = -1.35;
						vTaskDelay(700);
						exp_height_3508 = 270;
						vTaskDelay(700);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 3;
					}
				}

				if (run_count == 3)
				{
					if (last_count != 4)
					{
						Pure_Handle.target_theta = -1.55f;
						vTaskDelay(1700);
						if (MerLin_Pack.MerLin[8] == 1)
						{
							PurePursuit_SetTarget(&Pure_Handle, 6.15f, 5.35f, -1.55f); // 吸8
							GPIO_Pin_State_Valve = 1;
						}
						else if (MerLin_Pack.MerLin[11] == 1)
						{
							PurePursuit_SetTarget(&Pure_Handle, 7.33f, 5.35f, -1.55f); // 吸11
							GPIO_Pin_State_Valve = 1;
						}
					}
				}

				if (run_count == 4)
				{
					if (MerLin_Pack.MerLin[8] == 1)
					{
						if (last_count != 5)
						{
							exp_height_3508 = 250; // 吸8
							vTaskDelay(500);
							exp_flexible_len = 400;
							vTaskDelay(300);
							exp_flexible_len = 100;
							exp_height_3508 = 540;
							vTaskDelay(300);
							over_turn_pos = 2.7;
							PurePursuit_SetTarget(&Pure_Handle, 8.70f, 5.57f, -1.55f); // 准备准备进斜坡
							last_count = 5;
						}
					}

					if (MerLin_Pack.MerLin[11] == 1)
					{
						if (last_count != 5)
						{
							exp_height_3508 = 50; // 吸11
							vTaskDelay(500);
							exp_flexible_len = 400;
							vTaskDelay(300);
							exp_flexible_len = 100;
							exp_height_3508 = 540;
							vTaskDelay(300);
							over_turn_pos = 2.7;
							PurePursuit_SetTarget(&Pure_Handle, 8.70f, 5.57f, -1.55f); // 准备准备进斜坡
							last_count = 5;
						}
					}
				}
				if (run_count == 5)
				{
					if (last_count != 6)
					{
						Pure_Handle.max_velocity = 0.8f;
						PurePursuit_SetTarget(&Pure_Handle, 8.70f, 0.70f, -1.55f); // 准备上斜坡
						last_count = 6;
					}
				}

				if (run_count == 6)
				{
					if (last_count != 7)
					{
						Pure_Handle.target_theta = 0.0f;
						vTaskDelay(1500);
						PurePursuit_SetTarget(&Pure_Handle, 11.3f, 0.85f, 0.0f); // 上斜坡
						last_count = 7;
					}
				}

				if (run_count == 7)
				{
					if (last_count != 8)
					{
						Pure_Handle.target_theta = 1.55f;
						last_count = 8;
						g_mgr.slots[0].in_use = 0;
						vTaskDelete(NULL);
					}
				}
			}
			else if (MerLin_Pack.MerLin[5] == 1)
			{
				if (run_count == 1)
				{
					Pure_Handle.target_theta = 0.0f;
					vTaskDelay(1000);
					PurePursuit_SetTarget(&Pure_Handle, 2.56f, 5.40f, 0.0f); // 无斜坡
					exp_flexible_len = 100;
					exp_height_3508 = 450; // 高度600
					over_turn_pos = 1.5;
					last_count = 2;
				}

				if (run_count == 2)
				{
					if (last_count != 3)
					{
						Pure_Handle.target_theta = -1.55f;
						vTaskDelay(1700);
						PurePursuit_SetTarget(&Pure_Handle, 4.90f, 5.40f, -1.55f); // 吸5
						GPIO_Pin_State_Valve = 1;
						GPIO_Pin_State_AirPump = 1;
						last_count = 3;
					}
				}

				if (run_count == 3)
				{
					if (last_count != 4)
					{
						PurePursuit_SetTarget(&Pure_Handle, 4.90f, 5.260f, -1.55f);
						last_count = 4;
					}
				}

				if (run_count == 4)
				{
					if (last_count != 5)
					{
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 40;
						PurePursuit_SetTarget(&Pure_Handle, 4.90f, 5.58f, -1.55f); // 后退
						last_count = 5;
					}
				}

				if (run_count == 5)
				{
					if (last_count != 6)
					{
						cloud_pos_target = -1.35;
						vTaskDelay(700);
						exp_height_3508 = 270;
						vTaskDelay(200);
						PurePursuit_SetTarget(&Pure_Handle, 2.56f, 5.40f, 0.0f); // 无斜坡
						vTaskDelay(500);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 6;
					}
				}

				if (run_count == 6)
				{
					if (last_count != 7)
					{
						Pure_Handle.target_theta = 0.0f;
						vTaskDelay(1500);
						if (MerLin_Pack.MerLin[0] == 1)
						{
							PurePursuit_SetTarget(&Pure_Handle, 2.56f, 1.722f, 0.0f); // 去取第一个格的块儿
							exp_height_3508 = 250;									  // 高度400
						}
						else if (MerLin_Pack.MerLin[1] == 1)
						{
							PurePursuit_SetTarget(&Pure_Handle, 2.56f, 2.90f, 0.0f); // 去取第2个格的块
							exp_height_3508 = 50;									 // 高度200
						}
						else if (MerLin_Pack.MerLin[2] == 1)
						{
							PurePursuit_SetTarget(&Pure_Handle, 2.56f, 4.15f, 0.0f); // 去取第三个格的块儿
							exp_height_3508 = 250;									 // 高度400
						}
						last_count = 7;
					}
				}

				if (run_count == 7)
				{
					if (MerLin_Pack.MerLin[0] == 1)
					{
						if (last_count != 8)
						{
							GPIO_Pin_State_AirPump = 1;
							GPIO_Pin_State_Valve = 1;
							PurePursuit_SetTarget(&Pure_Handle, 2.725f, 1.722f, 0.0f); // 去取第一个格的块儿
							last_count = 8;
						}
					}
					else if (MerLin_Pack.MerLin[1] == 1)
					{
						if (last_count != 8)
						{
							GPIO_Pin_State_AirPump = 1;
							GPIO_Pin_State_Valve = 1;
							PurePursuit_SetTarget(&Pure_Handle, 2.725f, 2.90f, 0.0f); // 去取第二个格的块儿
							last_count = 8;
						}
					}
					else if (MerLin_Pack.MerLin[2] == 1)
					{
						if (last_count != 8)
						{
							GPIO_Pin_State_AirPump = 1;
							GPIO_Pin_State_Valve = 1;
							PurePursuit_SetTarget(&Pure_Handle, 2.725f, 4.15f, 0.0f); // 去取第三个格的块儿
							last_count = 8;
						}
					}
				}

				if (run_count == 8)
				{
					if (last_count != 9)
					{
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_height_3508 = 500;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 2.56f, 0.605f, 0.0f); // 准备进有斜坡的一边
						last_count = 9;
					}
				}

				if (run_count == 9)
				{
					if (last_count != 10)
					{
						Pure_Handle.target_theta = 1.55f;
						vTaskDelay(1200);
						Pure_Handle.max_velocity = 0.6f;
						PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
						last_count = 10;
					}
				}

				if (run_count == 10)
				{
					if (last_count != 11)
					{
						Pure_Handle.max_velocity = 1.0f;
						PurePursuit_SetTarget(&Pure_Handle, 11.3f, 0.85f, 1.55f); // 上斜坡
						last_count = 11;
						g_mgr.slots[0].in_use = 0;
						vTaskDelete(NULL);
					}
				}
			}
		}
		
		if (MerLin_State_flag == 6) // 0,1,2有一个，10有一个
		{
			if(run_count == 1)
			{
				if(last_count != 2)
				{
					Pure_Handle.max_velocity = 0.4f;
					Pure_Handle.target_theta = 0.0f;
					vTaskDelay(1000);
					if(MerLin_Pack.MerLin[0] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 2.765f, 1.722f, 0.0f); // 去取第一个格的块儿
						exp_height_3508 = 250; // 高度400
					}
					else if(MerLin_Pack.MerLin[1] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 2.767f, 2.90f, 0.0f); // 去取第2个格的块
						exp_height_3508 = 50; // 高度200
					}
					else if(MerLin_Pack.MerLin[2] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 2.759f, 4.15f, 0.0f); // 去取第三个格的块儿
						exp_height_3508 = 250; // 高度400
					}
					exp_flexible_len = 100;
					over_turn_pos = 1.5;
					last_count = 2;
				}
			}

			if(run_count == 2)
			{
				if(last_count != 3)
				{
					GPIO_Pin_State_AirPump = 1;
					GPIO_Pin_State_Valve = 1;
					exp_flexible_len = 400;
					vTaskDelay(300);
					exp_height_3508 = 500;
					vTaskDelay(300);
					exp_flexible_len = 40;
					PurePursuit_SetTarget(&Pure_Handle, 2.56f, 0.605f, 0.0f); // 准备进有斜坡的一边
					cloud_pos_target = -1.35;
					vTaskDelay(700);
					exp_height_3508 = 270;
					vTaskDelay(700);
					cloud_pos_target = -1.6;
					vTaskDelay(200);
					GPIO_Pin_State_Valve = 0;
					Two_Three_Sign = 1;
					vTaskDelay(200);
					exp_height_3508 = 500;
					vTaskDelay(500);
					cloud_pos_target = 0;
					last_count = 3;
				}
			}

			if(run_count == 3)
			{
				if(last_count != 4)
				{
					Pure_Handle.target_theta = 1.55f;
					vTaskDelay(1700);
					Pure_Handle.max_velocity = 0.67f;
					PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
					exp_height_3508 = 250;//高度400
					last_count = 4;
				}
			}

			if(run_count == 4)
			{
				if(last_count != 5)
				{
					Pure_Handle.target_theta = 3.14f;
					vTaskDelay(1700);
					Pure_Handle.max_velocity = 0.8f;
					PurePursuit_SetTarget(&Pure_Handle, 8.63f, 3.102f, 3.14f); //吸10
					last_count = 5;
				}
			}

			if(run_count == 5)
			{
				if(last_count != 6)
				{
					PurePursuit_SetTarget(&Pure_Handle, 8.455f, 3.102f,3.14f);
					GPIO_Pin_State_Valve = 1;
					GPIO_Pin_State_AirPump = 1;
					last_count = 6;
				}
			}

			if(run_count == 6)
			{
				if(last_count != 7)
				{
					exp_height_3508 = 250;//吸10
					vTaskDelay(500);
					exp_flexible_len = 400;
					vTaskDelay(300);
					exp_flexible_len = 100;
					exp_height_3508 = 540;
					vTaskDelay(300);
					over_turn_pos = 2.7;
					PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 3.14f);
					last_count = 7;
				}
			}

			if(run_count == 7)
			{
				if(last_count != 8)
				{
					Pure_Handle.max_velocity = 1.0f;
					PurePursuit_SetTarget(&Pure_Handle, 11.3f, 0.85f, 3.14f);
					last_count = 8;
					g_mgr.slots[0].in_use = 0;
					vTaskDelete(NULL);
				}
			}

			if(run_count == 8)
			{
				if(last_count != 9)
				{
					Pure_Handle.target_theta = 1.55f;
					last_count = 9;
					g_mgr.slots[0].in_use = 0;
					vTaskDelete(NULL);
				}
			}
		}

		if (MerLin_State_flag == 7) // 3，6，9有一个，5，8，11有一个
		{
			if(run_count == 1)
			{
				if(last_count != 2)
				{
					Pure_Handle.max_velocity = 0.45f;
					Pure_Handle.target_theta = 0.0f;
					vTaskDelay(1700);
					PurePursuit_SetTarget(&Pure_Handle, 2.56f, 5.35f, 0.0f);
					exp_flexible_len = 100;
					exp_height_3508 = 450;//高度600
					over_turn_pos = 1.5;
					last_count = 2;
				}
			}

			if(run_count == 2)
			{
				if(MerLin_Pack.MerLin[5] == 1)
				{
					if(last_count == 3)
					{
						Pure_Handle.target_theta = -1.55f;
						vTaskDelay(1700);
						PurePursuit_SetTarget(&Pure_Handle,  4.90f, 5.35f, -1.55f);//吸5
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						last_count = 3;
					}
				}else if(MerLin_Pack.MerLin[8] == 1)
				{
					if(last_count == 3)
					{
						Pure_Handle.target_theta = -1.55f;
						vTaskDelay(1700);
						PurePursuit_SetTarget(&Pure_Handle,  6.15f, 5.35f, -1.55f);//吸8
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						last_count = 3;
					}
				}else if(MerLin_Pack.MerLin[11] == 1)
				{
					if(last_count == 3)
					{
						Pure_Handle.target_theta = -1.55f;
						vTaskDelay(1700);
						PurePursuit_SetTarget(&Pure_Handle, 7.33f, 5.35f, -1.55f);//吸8
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						last_count = 3;
					}
				}
			}

			if(run_count == 3)
			{
				if(MerLin_Pack.MerLin[5] == 1)
				{
					if(last_count == 4)
					{
						PurePursuit_SetTarget(&Pure_Handle,  4.90f, 5.260f, -1.55f);//吸5
						last_count = 4;
					}
				}

				if(MerLin_Pack.MerLin[8] == 1)
				{
					if(last_count == 4)
					{
						PurePursuit_SetTarget(&Pure_Handle, 6.15f, 5.260f, -1.55f);//吸5
						last_count = 4;
					}
				}

				if(MerLin_Pack.MerLin[11] == 1)
				{
					if(last_count == 4)
					{
						PurePursuit_SetTarget(&Pure_Handle, 7.33f, 5.260f, -1.55f);//吸5
						last_count = 4;
					}
				}
			}

			if(run_count == 4)
			{
				if(MerLin_Pack.MerLin[5] == 1)
				{
					exp_height_3508 = 450;//高度600
					vTaskDelay(300);
					exp_flexible_len = 400;
					vTaskDelay(300);
					exp_height_3508 = 500;
					vTaskDelay(300);
					exp_flexible_len = 40;
					PurePursuit_SetTarget(&Pure_Handle, 4.90f, 5.58f, -1.55f);//后退
				}

				if(MerLin_Pack.MerLin[8] == 1)
				{
					exp_height_3508 = 250;//高度400
					vTaskDelay(300);
					exp_flexible_len = 400;
					vTaskDelay(300);
					exp_height_3508 = 500;
					vTaskDelay(300);
					exp_flexible_len = 40;
					PurePursuit_SetTarget(&Pure_Handle, 6.15f, 5.58f, -1.55f);//后退
				}

				if(MerLin_Pack.MerLin[11] == 1)
				{
					exp_height_3508 = 50;//高度200
					vTaskDelay(300);
					exp_flexible_len = 400;
					vTaskDelay(300);
					exp_height_3508 = 500;
					vTaskDelay(300);
					exp_flexible_len = 40;
					PurePursuit_SetTarget(&Pure_Handle, 7.33f, 5.58f, -1.55f);//后退
				}
			}

			if(run_count == 5)
			{
				cloud_pos_target = -1.35;
				vTaskDelay(700);
				exp_height_3508 = 270;
				vTaskDelay(200);
				PurePursuit_SetTarget(&Pure_Handle, 8.70f, 5.57f, -1.55f);//准备准备进斜坡
				vTaskDelay(500);
				cloud_pos_target = -1.6;
				vTaskDelay(200);
				GPIO_Pin_State_Valve = 0;
				Two_Three_Sign = 1;
				vTaskDelay(200);
				exp_height_3508 = 500;
				vTaskDelay(500);
				cloud_pos_target = 0;
			}

			if (run_count == 6)
			{
				if (last_count != 7)
				{
					Pure_Handle.max_velocity = 0.8f;
					PurePursuit_SetTarget(&Pure_Handle, 8.70f, 0.70f, -1.55f); // 准备上斜坡
					last_count = 7;
				}
			}

			if(run_count == 7)
			{
				if(last_count != 8)
				{
					Pure_Handle.target_theta = 1.55f;
					vTaskDelay(2000);
					if(MerLin_Pack.MerLin[3] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 2.765f, 1.722f, 0.0f); // 去取第一个格的块儿
						GPIO_Pin_State_Valve = 1;
						GPIO_Pin_State_AirPump = 1;
					}
					if(MerLin_Pack.MerLin[6] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 2.767f, 2.90f, 0.0f); // 去取第2个格的块
						GPIO_Pin_State_Valve = 1;
						GPIO_Pin_State_AirPump = 1;
					}
					if(MerLin_Pack.MerLin[9] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 7.546f, 0.80f, 1.55f);//吸9
						GPIO_Pin_State_Valve = 1;
						GPIO_Pin_State_AirPump = 1;
					}
					last_count = 8;
				}
			}

			if(run_count == 8)
			{
				if(last_count != 9)
				{
					if (MerLin_Pack.MerLin[3] == 1)
					{
						exp_height_3508 = 50;//吸3
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
					}
					if (MerLin_Pack.MerLin[6] == 1)
					{
						exp_height_3508 = 250;//吸6
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
					}
					if (MerLin_Pack.MerLin[9] == 1)
					{
						exp_height_3508 = 50;//吸9
						vTaskDelay(500);
						exp_flexible_len = 400;
						vTaskDelay(300);
						exp_flexible_len = 100;
						exp_height_3508 = 540;
						vTaskDelay(300);
						over_turn_pos = 2.7;
						PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
					}
					last_count = 9;
				}
			}

			if(run_count == 9)
			{
				if (last_count != 10)
				{
					Pure_Handle.max_velocity = 1.0f;
					PurePursuit_SetTarget(&Pure_Handle, 11.3f, 0.85f, 1.55f); //上斜坡
					last_count = 10;
					g_mgr.slots[0].in_use = 0;
					vTaskDelete(NULL);
				}
			}
		}

		if (MerLin_State_flag == 8) // 3，6，9有一个，10有一个
		{
			if(run_count == 1)
			{
				if(last_count != 2)
				{
					PurePursuit_SetTarget(&Pure_Handle, 2.56f, 0.605f, 1.55f);//第二个点
					exp_flexible_len = 100;
					exp_height_3508 = 250;
					over_turn_pos = 1.5;
					last_count = 2;
				}
			}

			if(run_count == 2)
			{
				if(last_count != 3)
				{
					Pure_Handle.max_velocity = 0.67f;
					if(MerLin_Pack.MerLin[3] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 5.11f, 0.80f, 1.55f);//吸3
						GPIO_Pin_State_Valve = 1;
					}
					else if(MerLin_Pack.MerLin[6] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 6.37f, 0.80f, 1.55f); //吸6
						GPIO_Pin_State_Valve = 1;
					}
					else if(MerLin_Pack.MerLin[9] == 1)
					{
						PurePursuit_SetTarget(&Pure_Handle, 7.546f, 0.80f, 1.55f);//吸9
						GPIO_Pin_State_Valve = 1;
					}
					exp_height_3508 = 250;//高度400
					last_count = 3;
				}
			}

			if (run_count == 3)
			{
				if (MerLin_Pack.MerLin[3] == 1)
				{
					exp_height_3508 = 50; // 吸3
					vTaskDelay(500);
					exp_flexible_len = 400;
					vTaskDelay(300);
					exp_flexible_len = 100;
					exp_height_3508 = 540;
					vTaskDelay(300);
					over_turn_pos = 2.7;
					PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
				}
				else if (MerLin_Pack.MerLin[6] == 1)
				{
					exp_height_3508 = 250; // 吸6
					vTaskDelay(500);
					exp_flexible_len = 400;
					vTaskDelay(300);
					exp_flexible_len = 100;
					exp_height_3508 = 540;
					vTaskDelay(300);
					over_turn_pos = 2.7;
					PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
				}
				else if (MerLin_Pack.MerLin[9] == 1)
				{
					exp_height_3508 = 50; // 吸9
					vTaskDelay(500);
					exp_flexible_len = 400;
					vTaskDelay(300);
					exp_flexible_len = 100;
					exp_height_3508 = 540;
					vTaskDelay(300);
					over_turn_pos = 2.7;
					PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 1.55f); // 准备上斜坡
				}
			}

			if(run_count == 4)
			{
				if (last_count != 5)
				{
					Pure_Handle.target_theta = 3.14f;
					vTaskDelay(1700);
					Pure_Handle.max_velocity = 0.8f;
					PurePursuit_SetTarget(&Pure_Handle, 8.63f, 3.102f, 3.14f); // 吸10
					last_count = 5;
				}
			}

			if(run_count == 5)
			{
				if (last_count != 6)
				{
					PurePursuit_SetTarget(&Pure_Handle, 8.455f, 3.102f,3.14f);
					GPIO_Pin_State_Valve = 1;
					GPIO_Pin_State_AirPump = 1;
					last_count = 6;
				}
			}

			if(run_count == 6)
			{
				if (last_count != 7)
				{
					exp_height_3508 = 250;//吸10
					vTaskDelay(500);
					exp_flexible_len = 400;
					vTaskDelay(300);
					exp_flexible_len = 100;
					exp_height_3508 = 540;
					vTaskDelay(300);
					over_turn_pos = 2.7;
					PurePursuit_SetTarget(&Pure_Handle, 8.63f, 0.56f, 3.14f);
					last_count = 7;
				}
			}

			if(run_count == 7)
			{
				if (last_count != 8)
				{
					Pure_Handle.max_velocity = 1.0f;
					PurePursuit_SetTarget(&Pure_Handle, 11.3f, 0.85f, 3.14f);
					last_count = 8;
					g_mgr.slots[0].in_use = 0;
					vTaskDelete(NULL);
				}
			}

			if (run_count == 8)
			{
				if (last_count != 9)
				{
					Pure_Handle.target_theta = 1.55f;
					last_count = 9;
					g_mgr.slots[0].in_use = 0;
					vTaskDelete(NULL);
				}
			}
		}

		if (MerLin_State_flag == 9) // 5，8，11有一个，10有一个
		{
			if (run_count == 1)
			{
				if (last_count != 2)
				{
					Pure_Handle.max_velocity = 0.46f;
					Pure_Handle.target_theta = 0.0f;
					vTaskDelay(1680);
					PurePursuit_SetTarget(&Pure_Handle, 2.56f, 5.35f, 0.0f); // 5811区入口点
					exp_height_3508 = 450.0f;
					exp_flexible_len = 66.0f;
					over_turn_pos = 1.5f;
					vTaskDelay(500);
					last_count = 2;
				}
			}
			if (run_count == 2)
			{
				if (last_count != 3)
				{
					Pure_Handle.target_theta = -1.55f; // 旋转准备进入5811区
					vTaskDelay(2888);
					PurePursuit_SetTarget(&Pure_Handle, 2.56f, 5.35f, -1.55f);

					last_count = 3;
				}
			}
			/**First:取块[5]**/
			if (MerLin_Pack.MerLin[5] == 1)
			{
				if (run_count == 3)
				{
					if (last_count != 4)
					{
						last_count = 4;
					}
				}
			}
			/**First取块[8]**/
			else if (MerLin_Pack.MerLin[8] == 1)
			{

				if (run_count == 3) // 到达1.1
				{
					if (last_count != 4)
					{
						Pure_Handle.max_velocity = 0.76f;
						PurePursuit_SetTarget(&Pure_Handle, 6.145f, 5.35f, -1.55f);

						last_count = 4;
					}
				}
				if (run_count == 4) // 到达1.2
				{
					if (last_count != 5)
					{
						PurePursuit_SetTarget(&Pure_Handle, 6.145f, 5.26f, -1.55f);
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;

						last_count = 5;
					}
				}

				if (run_count == 5) // 吸+存+准备Second1.1
				{
					if (last_count != 6)
					{
						exp_height_3508 = 250.0f;
						vTaskDelay(300);
						exp_flexible_len = 400.0f;
						vTaskDelay(200);
						PurePursuit_SetTarget(&Pure_Handle, 6.145f, 5.47f, -1.55f);
						exp_height_3508 = 540.0f;
						vTaskDelay(300);
						exp_flexible_len = 40.0f;
						last_count = 6;
					}
				}
				if (run_count == 6) // 吸+存+准备Second1.2
				{
					if (last_count != 7)
					{
						PurePursuit_SetTarget(&Pure_Handle, 8.63f, 5.47f, -1.55f); // 左上角旋转处
						cloud_pos_target = -1.35f;
						vTaskDelay(700);
						exp_height_3508 = 270.0f;
						vTaskDelay(700);
						cloud_pos_target = -1.6;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 7;
					}
				}
			}
			/**First取块[11]**/
			else if (MerLin_Pack.MerLin[11] == 1)
			{
				if (run_count == 3) // 到达
				{
					if (last_count != 4)
					{
						Pure_Handle.max_velocity = 0.6f;
						PurePursuit_SetTarget(&Pure_Handle, 7.352f, 5.35f, -1.55f); // 到达
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						last_count = 4;
					}
				}
				if (run_count == 4) // 到达
				{
					if (last_count != 5)
					{
						Pure_Handle.max_velocity = 0.6f;
						PurePursuit_SetTarget(&Pure_Handle, 7.352f, 5.247f, -1.55f); // 到达
						GPIO_Pin_State_AirPump = 1;
						GPIO_Pin_State_Valve = 1;
						last_count = 5;
					}
				}
				if (run_count == 5) // 吸+存+准备Second
				{
					if (last_count != 6)
					{
						exp_height_3508 = 250.0f;
						vTaskDelay(300);
						exp_flexible_len = 88.0f;
						vTaskDelay(200);
						exp_height_3508 = 540.0f;
						vTaskDelay(300);
						exp_flexible_len = 40.0f;
						PurePursuit_SetTarget(&Pure_Handle, 8.653f, 5.47f, -1.55f); // 左上角旋转处1.1
						cloud_pos_target = -1.35f;
						vTaskDelay(700);
						exp_height_3508 = 270.0f;
						vTaskDelay(700);
						cloud_pos_target = -1.6f;
						vTaskDelay(200);
						GPIO_Pin_State_Valve = 0;
						Two_Three_Sign = 1;
						vTaskDelay(200);
						exp_height_3508 = 500.0f;
						vTaskDelay(500);
						cloud_pos_target = 0;
						last_count = 6;
					}
				}
			}
			/**左上角旋转区**/
			if (run_count == 7)
			{
				if (last_count != 8)
				{
					Pure_Handle.max_velocity = 0.45f;
					Pure_Handle.target_theta = -3.1f;
					vTaskDelay(700);
					PurePursuit_SetTarget(&Pure_Handle, 8.653f, 5.47f, -3.1f); // 左上角旋转处1.2
					last_count = 8;
				}
			}
			/**Second:取块[10]**/

			if (run_count == 8)
			{
				if (last_count != 9)
				{
					Pure_Handle.max_velocity = 0.66f;
					exp_height_3508 = 250.0f;
					PurePursuit_SetTarget(&Pure_Handle, 8.653f, 3.138f, -3.1f);
					last_count = 9;
				}
			}
			if (run_count == 9)
			{
				if (last_count != 10)
				{
					Pure_Handle.max_velocity = 0.66f;
					exp_height_3508 = 250.0f;
					PurePursuit_SetTarget(&Pure_Handle, 8.455f, 3.138f, -3.1f);
					GPIO_Pin_State_Valve = 1;
					last_count = 10;
				}
			}
			if (run_count == 10)
			{
				if (last_count != 11)
				{
					exp_flexible_len = 400.0f;
					vTaskDelay(300);
					exp_height_3508 = 500.0f;
					PurePursuit_SetTarget(&Pure_Handle, 8.653f, 3.138f, -3.1f);
					last_count = 11;
				}
			}
			if (run_count == 11)
			{
				if (last_count != 12)
				{
					PurePursuit_SetTarget(&Pure_Handle, 8.635f, 0.596f, -3.1f); // 斜坡前旋转区1.1
					exp_flexible_len = 58.0f;
					over_turn_pos = 2.7f;
					last_count = 12;
				}
			}
			if (run_count == 12)
			{
				if (last_count != 13)
				{
					Pure_Handle.max_velocity = 0.45f;
					Pure_Handle.target_theta = 1.55f;
					vTaskDelay(700);
					PurePursuit_SetTarget(&Pure_Handle, 8.635f, 0.586f, 1.55f); // 斜坡前旋转区1.2
					last_count = 13;
				}
			}
			if (run_count == 13)
			{
				if (last_count != 14)
				{
					Pure_Handle.max_velocity = 0.88f;
					PurePursuit_SetTarget(&Pure_Handle, 11.3f, 0.85f, 1.55f); // 上斜坡
					last_count = 14;
					g_mgr.slots[0].in_use = 0;
					vTaskDelete(NULL);
				}
			}
		}
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
	}
}

uint8_t GetReturnValue(void)
{
	
	uint8_t count_012 = (MerLin_Pack.MerLin[0] == 1) + (MerLin_Pack.MerLin[1] == 1) + (MerLin_Pack.MerLin[2] == 1);
	uint8_t count_369 = (MerLin_Pack.MerLin[3] == 1) + (MerLin_Pack.MerLin[6] == 1) + (MerLin_Pack.MerLin[9] == 1);
	uint8_t count_5811 = (MerLin_Pack.MerLin[5] == 1) + (MerLin_Pack.MerLin[8] == 1) + (MerLin_Pack.MerLin[11] == 1);
	uint8_t val_10 = (MerLin_Pack.MerLin[10] == 1);

	if (count_012 == 2)
	{
		return 1;
	}
	if (count_369 == 2)
	{
		return 2;
	}

	if (count_5811 == 2)
	{
		return 3;
	}

	if (count_012 == 1 && count_369 == 1)
	{
		return 4;
	}

	if (count_012 == 1 && count_5811 == 1)
	{
		return 5;
	}

	if (count_012 == 1 && val_10 == 1)
	{
		return 6;
	}

	if (count_369 == 1 && count_5811 == 1)
	{
		return 7;
	}

	if (count_369 == 1 && val_10 == 1)
	{
		return 8;
	}

	if (count_5811 == 1 && val_10 == 1)
	{
		return 9;
	}
	else{
		return 0;
	}
}

void Auto_Place_Block_Action(void *param)//三区放块儿
{
	TickType_t last_wake_time = xTaskGetTickCount();
	
	run_count = 0;
	static uint32_t last_count = 0;	
	while(1)
	{
		if(run_count == 0)
		{
			if(last_count != 1)
			{
				Pure_Handle.max_velocity = 1.0f;
				PurePursuit_SetTarget(&Pure_Handle, 11.3f, 1.9f, 1.55f);
				over_turn_pos = 1.8;
				exp_height_3508 = 300;
				last_count = 1;
			}
		}
		
		if(run_count == 1)
		{
			if(last_count != 2)
			{
				PurePursuit_SetTarget(&Pure_Handle, 10.74f, 5.14f, 1.55f);
				last_count = 2;
			}
		}
		
		if(run_count == 2)
		{
			if(last_count != 3)
			{
				Two_Three_Sign = 2;//前推后并关吸盘
				GPIO_Pin_State_Valve = 0;
				last_count = 3;
				g_mgr.slots[0].in_use = 0;
				vTaskDelete(NULL);
			}
		}
		
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
	}
}

TaskHandle_t Action_Handle;
uint8_t Auto_Rod_Retrieval_Action_Create = 0;
uint8_t Auto_KFS_Retrieval_Action_Create = 0;
uint8_t Auto_Place_Block_Create = 0;
uint8_t Auto_KFS_Action_Create = 0;

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
				if(Auto_Rod_Retrieval_Action_Create == 0)
				{
					ActionManager_Send(&g_mgr, Auto_Rod_Retrieval_Action, NULL);
					Auto_Rod_Retrieval_Action_Create =1;
				}
			}
			
			if(Remote_Control.First.Left_Key_Left == 1 && Remote_Control.Second.Left_Key_Left == 0)
			{
				if(Auto_Place_Block_Create == 0)
				{
					ActionManager_Send(&g_mgr, Auto_Place_Block_Action, NULL);
					Auto_Place_Block_Create = 1;
				}
			}
			
			if(Remote_Control.First.Left_Key_Right == 1 && Remote_Control.Second.Left_Key_Right == 0)
			{
				if(Auto_KFS_Action_Create == 0)
				{
					ActionManager_Send(&g_mgr, Auto_KFS_Action, NULL);
					Auto_KFS_Action_Create = 1;
				}
			}
		}
		
		if (Remote_Control.First.Right_Switch_Up == 0 && Remote_Control.Second.Right_Switch_Down == 0)
		{
			if (Mode == REMOTE)
			{
					if (Remote_Control.First.Left_Key_Left == 1 && Remote_Control.Second.Left_Key_Left == 0)
					{
							vTaskDelay(50);
							if (flag_Pump == 0)
							{
									GPIO_Pin_State_AirPump = 1;
									flag_Pump = 1;
							}
							else
							{
									GPIO_Pin_State_AirPump = 0;
									flag_Pump = 0;
							}
					}
					
					if (Remote_Control.First.Left_Key_Up == 1 && Remote_Control.Second.Left_Key_Up == 0)
					{
							vTaskDelay(50);
							if (flag_Value == 0)
							{
									GPIO_Pin_State_Valve = 1;
									flag_Value = 1;
							}
							else
							{
									GPIO_Pin_State_Valve = 0;
									flag_Value = 0;
							}
					}
					
			}
		}
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(10));
	}
}
