#include "Task_Init.h"
#include "VL53_100.h"
#include "encoder.h"
#include "usart.h"

#include "comm_stm32_hal_middle.h"
#include "dataFrame.h"
#include "comm.h"

#include "AutoPilot.h"
#include "Pilot_callback.h"
#include "Action_Config.h"
#include "Action.h"

#include "crc_ccitt.h"

#include "Navigation.h"
#include "math.h"

#include "Servo.h"

//底盘
SteeringWheel steeringWheelArray[4];
Wheel_t wheelArray[4];
Chassis_t chassis;

Joint_t Joint;//机械臂

//变量
float cloud_pos_target = 0.0f;//云台
float exp_height_3508 = 0.0f;//升降
float over_turn_pos = 0.0f;//翻转
float flexible_pos = 0.0f;//伸缩
volatile  float Kp_flexible = 0.0f;
volatile float Kd_flexible = 0.0f;
volatile  float Kp_CloudPlatform = 0.0f;
volatile float Kd_CloudPlatform = 0.0f;

int16_t can1_3508_Tx_Data[4]={0};//3508
uint8_t Joint_Enabled = 1;//使能

//气泵
GPIO_PinState GPIO_Pin_State_AirPump = 0;
GPIO_PinState GPIO_Pin_State_Valve = 0;

//句柄
TaskHandle_t Arm_Task_Handle;
TaskHandle_t Arm_3508_Task_Handle;
TaskHandle_t Remote_Analysis_Handle;
TaskHandle_t Uart_Tx_Handle;
TaskHandle_t Auto_Navigatoin_Handle;
extern TaskHandle_t task_handle;
extern TaskHandle_t Radar_Handle;
extern TaskHandle_t Action_Handle;

//串口数据
uint8_t usart3_dma_buff[64];
uint8_t usart5_dma_buff[60];
Remote_Handle_t Remote_Control;

//信号量
extern SemaphoreHandle_t Remote_semaphore;
extern SemaphoreHandle_t Radar_semaphore;

//雷达
uint8_t Data_Package[64] = {0};
struct Pose3DPacket pos_world;
PurePursuitController Pure_Handle;

//距离传感器
LASER_SEND_Typedef DT_35_Len;

//任务
void Radar_Analysis_Task(void *pvParameters);
void Remote_Analysis_Task(void *pvParameters);
void Uart_Tx(void *pvParameters);
void Auto_Navigatoin(void *para);
void Arm_Task(void *param);
void Arm_3508_Task(void *param);
//声明
void Motor_init();

//模式
ChassisMode Mode = REMOTE;

//伸展PID
PID2 One_Four_PID, Two_Three_PID;
float expect_len = 0.0f;

//底盘动作变量
uint8_t Two_Three_Sign = 0;
uint8_t One_Four_Sign = 0;
extern int32_t Ramp_Value_Servo[4];
// 角度标准化到 [-π, π]
float NormalizeAngle_Rad(float angle) {
    int guard = 0;
    while (angle > PI && guard < 10) { angle -= 2 * PI; guard++; }
    guard = 0;
    while (angle < -PI && guard < 10) { angle += 2 * PI; guard++; }
    if (angle > PI) angle = PI;
    if (angle < -PI) angle = -PI;
    return angle;
}

float GetShortArcDiff(float current, float target) {
    float diff = target - current;
    return NormalizeAngle_Rad(diff);
}

PID2 PID_Theta;
void Task_Init(void)
{
		//遥控器
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff, sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
		
		//雷达
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart3, usart3_dma_buff, sizeof(usart3_dma_buff));
	
    wheelArray[0].pos.x =  0.305f;
    wheelArray[0].pos.y =  0.280f; 
    wheelArray[0].pos.z =  PI / 4.0f;
    wheelArray[1].pos.x =  0.305f;
    wheelArray[1].pos.y =  -0.325f;
    wheelArray[1].pos.z =  PI / 4.0f;
    wheelArray[2].pos.x =  -0.325f;
    wheelArray[2].pos.y =  -0.325f;
    wheelArray[2].pos.z =  PI / 4.0f;
    wheelArray[3].pos.x =  -0.325f;
    wheelArray[3].pos.y =  0.280f;
    wheelArray[3].pos.z =  PI / 4.0f;
		
    for(int i = 0; i < 4; i++)
    {
        wheelArray[i].user_data = &steeringWheelArray[i];
        wheelArray[i].set_target_cb = SetWheelTarget_Callback;
        chassis.wheel[i] = &wheelArray[i];
    }
		
    Vector2D barycenter = {0, 0};
    ChassisInit(&chassis, wheelArray, 4, barycenter, 25.2f, 1.25f, 0.0005f, 2, 300, 4);
		
		vTaskDelay(700);
		Motor_init();

		xTaskCreate(Arm_Task,"Arm_Task", 256, NULL, 5, &Arm_Task_Handle);
		xTaskCreate(Arm_3508_Task,"Arm_3508_Task",256,NULL, 5,&Arm_3508_Task_Handle);
		xTaskCreate(Remote_Analysis_Task, "Remote_Analysis_Task", 180, NULL, 4, &Remote_Analysis_Handle);
		xTaskCreate(Uart_Tx, "Uart_Tx", 256, NULL, 4, &Uart_Tx_Handle);
		xTaskCreate(Auto_Navigatoin,"Auto_Navigatoin",256,NULL,4,&Auto_Navigatoin_Handle);
		xTaskCreate(Action, "Action", 256, NULL, 4 , &Action_Handle);
		xTaskCreate(Radar_Analysis_Task,"Radar_Analysis_Task",180,NULL,4,&Radar_Handle);
}

void Motor_init()
{
	Joint.RM3508_motor.ID = 0x202;
	Joint.RM3508_motor.hcan = &hcan1;//升降
	
	vTaskDelay(5000);
	RobStrideInit(&Joint.Rs_motor[0], &hcan2, 0x01, RobStride_EL05);//云台
	vTaskDelay(100);
	RobStrideInit(&Joint.Rs_motor[1], &hcan2, 0x04, RobStride_EL05);//翻转
	vTaskDelay(100);
	RobStrideInit(&Joint.Rs_motor[2], &hcan1, 0x03, RobStride_EL05);//伸缩
	vTaskDelay(100);
	RobStrideDisable(&Joint.Rs_motor[0], 1);  // clear_error=1
	vTaskDelay(100);
	RobStrideDisable(&Joint.Rs_motor[1], 1);
	vTaskDelay(100);
	RobStrideDisable(&Joint.Rs_motor[2], 1);
	vTaskDelay(100);
	RobStrideSetMode(&Joint.Rs_motor[0], RobStride_MotionControl);
	vTaskDelay(100);
	RobStrideSetMode(&Joint.Rs_motor[1], RobStride_MotionControl);
	vTaskDelay(100);
	RobStrideSetMode(&Joint.Rs_motor[2], RobStride_MotionControl);
	vTaskDelay(100);
  RobStrideEnable(&Joint.Rs_motor[0]);
  vTaskDelay(100);
  RobStrideEnable(&Joint.Rs_motor[1]);
	vTaskDelay(100);
	RobStrideEnable(&Joint.Rs_motor[2]);
	
	Joint.RM3508_motor.pos_pid.Kp = 0.06f;
	Joint.RM3508_motor.pos_pid.Ki = 0.0f;
	Joint.RM3508_motor.pos_pid.Kd = 0.0f;
	Joint.RM3508_motor.pos_pid.limit = 10000.0f;
	Joint.RM3508_motor.pos_pid.output_limit = 9000.0f;
	
	Joint.RM3508_motor.vel_pid.Kp = 1.7f;
	Joint.RM3508_motor.vel_pid.Ki = 0.05f;
	Joint.RM3508_motor.vel_pid.Kd = 0.0f;
	Joint.RM3508_motor.vel_pid.limit = 10000.0f;
	Joint.RM3508_motor.vel_pid.output_limit = 16384.0f;//升降
	
	Joint.Rs_pos_pid[0].Kp = 6.0f;
	Joint.Rs_pos_pid[0].Ki = 0.0f;
	Joint.Rs_pos_pid[0].Kd = 0.0f;
	Joint.Rs_pos_pid[0].limit = 10.0f;
	Joint.Rs_pos_pid[0].output_limit = 50.0f;
	
	Joint.Rs_vel_pid[0].Kp = 1.5f;
	Joint.Rs_vel_pid[0].Ki = 0.05f;
	Joint.Rs_vel_pid[0].Kd = 0.0f;
	Joint.Rs_vel_pid[0].limit = 1000.0f;
	Joint.Rs_vel_pid[0].output_limit = 6.0f;//云台
	
	Kp_flexible = 100.0f;
	Kd_flexible = 2.0f;
	
	Kp_CloudPlatform = 15.0f;
	Kd_CloudPlatform = 0.7f;
	Ramp_Value_Servo[0] = 500;
	Ramp_Value_Servo[1] = 500;
	Ramp_Value_Servo[2] = 500;
	Ramp_Value_Servo[3] = 500;
	expect_len = Initial_Laser_Distance;
	vTaskDelay(1000);
}
float exp_scale_3508_t = 0.0f;//升降
float exp_sclae_Rs_t = 0.0f;//云台
float exp_flexible_len = 64.0f;//伸缩

float Length_To_Scale(float target_len) //
{
	if(target_len>410.0f || target_len==410.0f)	
	target_len = 405.0f;
	
	float alpha= acosf((arm1*arm1+target_len*target_len-arm2*arm2)/(2*arm1*target_len));
  float target_rad= (M_PI/2-alpha)*5/3;
	if(target_rad>2.34f)
		target_rad = 2.34f;
	 return target_rad;
}

float Kp_t = 100.0f;
float Kd_t = 2.0f;
float yuntai_Ramp = 0.06f;

void Arm_Task(void *param)
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	
	Joint.pos_offset[0] = 3.3859f;
	Joint.pos_offset[1] = 5.16852283f;
	Joint.pos_offset[2] = 2.9348f;
	for (;;)
	{
		//云台
		float target_scale_Rs = cloud_pos_target * 3.5f;
		RampToTarget(&exp_sclae_Rs_t,target_scale_Rs, yuntai_Ramp);
		RobStrideMotionControl(&Joint.Rs_motor[0], 0x01, 0, Joint.pos_offset[0] + exp_sclae_Rs_t, 0, Kp_CloudPlatform, Kd_CloudPlatform);
		
		//翻转
		RobStrideMotionControl(&Joint.Rs_motor[1], 0x04, 0, Joint.pos_offset[1] + (over_turn_pos * 2.0f), 0, Kp_t, Kd_t);
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_Pin_State_AirPump);//气泵
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_Pin_State_Valve);//电磁阀
		
		if(Mode == AUTO)
		{
			float short_arc_diff = GetShortArcDiff(Pure_Handle.current_theta, Pure_Handle.target_theta);
			PID_Control2(0.0f, short_arc_diff, &PID_Theta);
			if(Pure_Handle.is_running == true)
			{
				chassis.exp_vel.x = Pure_Handle.vx;
				chassis.exp_vel.y = Pure_Handle.vy;
			}
		}
		
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
	}
}
TaskHandle_t Arm_3508_Task_Handle;
void Arm_3508_Task(void *param)
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	while(1)
	{
		//升降 最大565
		float exp_scale_3508 = exp_height_3508/(2*M_PI*14.46)*19*8192;
		RampToTarget(&exp_scale_3508_t, exp_scale_3508, 10000.0f);
		PID_Control2(Joint.RM3508_motor.actual_pos,exp_scale_3508_t,&Joint.RM3508_motor.pos_pid);
		PID_Control2(Joint.RM3508_motor.motor.Speed,Joint.RM3508_motor.pos_pid.pid_out,&Joint.RM3508_motor.vel_pid);
    can1_3508_Tx_Data[1] = Joint.RM3508_motor.vel_pid.pid_out;
		
		//伸缩
		flexible_pos = Length_To_Scale(exp_flexible_len);
		RobStrideMotionControl(&Joint.Rs_motor[2], 0x03, 0, Joint.pos_offset[2] + flexible_pos, 0, Kp_flexible, Kd_flexible);
		
		MotorSend(&hcan1,0x200,can1_3508_Tx_Data);
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
	}
}
//遥控器摇杆结构体
PackControl_t recv_pack;
uint8_t recv_buff[20] = {0};
//遥控器梅林结构体
MerLin_Pack_t MerLin_Pack;
uint8_t MerLin_buff[20] = {0};

static void Key_Parse(uint32_t key, hw_key_t *out)
{
    out->Right_Switch_Up     = (key & KEY_Right_Switch_Up)     ? 1 : 0;
    out->Right_Switch_Down   = (key & KEY_Right_Switch_Down)   ? 1 : 0;

    out->Right_Key_Up        = (key & KEY_Right_Key_Up)        ? 1 : 0;
    out->Right_Key_Down      = (key & KEY_Right_Key_Down)      ? 1 : 0;
    out->Right_Key_Left      = (key & KEY_Right_Key_Left)      ? 1 : 0;
    out->Right_Key_Right     = (key & KEY_Right_Key_Right)     ? 1 : 0;

    out->Right_Broadside_Key = (key & KEY_Right_Broadside_Key) ? 1 : 0;

    out->Left_Switch_Up      = (key & KEY_Left_Switch_Up)      ? 1 : 0;
    out->Left_Switch_Down    = (key & KEY_Left_Switch_Down)    ? 1 : 0;

    out->Left_Key_Up         = (key & KEY_Left_Key_Up)         ? 1 : 0;
    out->Left_Key_Down       = (key & KEY_Left_Key_Down)       ? 1 : 0;
    out->Left_Key_Left       = (key & KEY_Left_Key_Left)       ? 1 : 0;
    out->Left_Key_Right      = (key & KEY_Left_Key_Right)      ? 1 : 0;

    out->Left_Broadside_Key  = (key & KEY_Left_Broadside_Key)  ? 1 : 0;
}

float apply_s_curve(float normalized_value)	//s曲线
{
    float t = fabsf(normalized_value);
    t = t > 1.0f ? 1.0f : t;
    
    float s = t * t * t * (6.0f * t * t - 15.0f * t + 10.0f);
    
    return (normalized_value > 0.0f) ? s : -s;
}

void Remote_Analysis()
{
    if(xSemaphoreTake(Remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        Remote_Control.Second = Remote_Control.First;
        Key_Parse(recv_pack.Key, &Remote_Control.First);
        
        if(Mode == REMOTE)
        {
            float scale = 1547.0f;
            
            float y_normalized = (float)recv_pack.rocker[1] / scale;
            float x_normalized = (float)recv_pack.rocker[0] / scale;
            float w_normalized = (float)recv_pack.rocker[2] / scale;
            
            Remote_Control.Ex = -apply_s_curve(y_normalized) * MAX_ROBOT_VEL;
            Remote_Control.Ey = apply_s_curve(x_normalized) * MAX_ROBOT_VEL;
            Remote_Control.Eomega = apply_s_curve(w_normalized) * MAX_ROBOT_OMEGA;
				}
		}else 
		{
				Remote_Control.Ex = 0;
				Remote_Control.Ey = 0;
				Remote_Control.Eomega = 0;
		}
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
  memcpy(&recv_buff, src, size);
  memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
  xSemaphoreGive(Remote_semaphore);
}
CommPackRecv_Cb  recv_cb = MyRecvCallback;

void MyMerLinCallback(uint8_t *src, uint16_t size, void *user_data)
{
	memcpy(&MerLin_buff, src, size);
  memcpy(&MerLin_Pack, MerLin_buff, sizeof(MerLin_Pack));
}
CommPackRecv_Cb  Recv_Cb_MerLin = MyMerLinCallback;

void Remote_Analysis_Task(void *pvParameters)
{
	g_comm_handle = Comm_Init(&huart5);
	RemoteCommInit(NULL);
	register_comm_recv_cb(recv_cb, PACK_CONTROL_CMD, &recv_pack);
	register_comm_recv_cb(Recv_Cb_MerLin, PACK_MerLin_CMD, &MerLin_Pack);
	while(1)
	{
		Remote_Analysis();
	}
}

//雷达数据解析
TaskHandle_t Radar_Handle;
void Radar_Analysis_Task(void *pvParameters)
{
	while(1)
	{
		if(xSemaphoreTake(Radar_semaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
		{
			memcpy(&pos_world,usart3_dma_buff,sizeof(struct Pose3DPacket));
			if(Mode == AUTO)
			{
				chassis.exp_vel.z = - PID_Theta.pid_out;
			}
		}
		else
		{
			Pure_Handle.vx = 0.0f;
			Pure_Handle.vy = 0.0f;
			chassis.exp_vel.z = 0.0f;
		}
	}
}

void Auto_Navigatoin(void *para)
{
	TickType_t last_wake_time = xTaskGetTickCount();
	
	PurePursuit_Init(&Pure_Handle);
	Pure_Handle.target_x = 1.0f;
	Pure_Handle.target_y = 2.65f;
	Pure_Handle.target_theta = 0.0f;
	
	One_Four_PID.Kp = 0.0015f;
	One_Four_PID.Ki = 0.000012f;
	One_Four_PID.Kd = 0.0f;
	One_Four_PID.limit = 10.0f;
	One_Four_PID.output_limit = 0.7f;
	
	Two_Three_PID.Kp = 0.0015f;
	Two_Three_PID.Ki = 0.000012f;
	Two_Three_PID.Kd = 0.0f;
	Two_Three_PID.limit = 10.0f;
	Two_Three_PID.output_limit = 0.7f;
	
	PID_Theta.Kp = 1.1f;
	PID_Theta.Ki = 0.0f;
	PID_Theta.Kd = 3.5f;
	PID_Theta.limit = 1000.0f;
	PID_Theta.output_limit = 2.0f;
	
	while(1)
	{
		Servo_control();
		if(Remote_Control.First.Left_Switch_Up == 1)
		{
			Mode = AUTO;
		}
		
		if((Remote_Control.First.Left_Switch_Up == 0  && Remote_Control.Second.Left_Switch_Up == 1) || (Remote_Control.First.Left_Switch_Down == 0  && Remote_Control.Second.Left_Switch_Down == 1))//遥控模式(左上 按键)
		{
			Mode = REMOTE;
			vTaskResume(task_handle);
		}
		
		if(Remote_Control.First.Left_Switch_Down == 1)
		{
			Mode = STRETCH;
		}
		
		if(Mode == REMOTE)
		{
			chassis.exp_vel.x = Remote_Control.Ex;
			chassis.exp_vel.y = Remote_Control.Ey;
			chassis.exp_vel.z = Remote_Control.Eomega;
		}
		
		if(Mode == STRETCH)
		{
			if(Remote_Control.First.Right_Switch_Down == 1)
			{
				if(Remote_Control.First.Right_Key_Up == 1 && Remote_Control.Second.Right_Key_Up == 0)//伸展变型(右上 按键)
				{
					vTaskSuspend(task_handle);
					steeringWheelArray[0].expectDirection = -135.0f;
					steeringWheelArray[3].expectDirection = -135.0f;//向左
					
					steeringWheelArray[1].expectDirection = 45.0f;
					steeringWheelArray[2].expectDirection = 45.0f;//向右
					expect_len = 500;
				}
				
				if(Remote_Control.First.Right_Key_Down == 1 && Remote_Control.Second.Right_Key_Down == 0)
				{
					vTaskSuspend(task_handle);
					steeringWheelArray[0].expectDirection = -135.0f;
					steeringWheelArray[3].expectDirection = -135.0f;//向左
					
					steeringWheelArray[1].expectDirection = 45.0f;
					steeringWheelArray[2].expectDirection = 45.0f;//向右
					expect_len = 150;
				}
			}
			
			PID_Control2(DT_35_Len.spi2, expect_len, &One_Four_PID);
			PID_Control2(DT_35_Len.spi2, expect_len, &Two_Three_PID);
			steeringWheelArray[0].expextVelocity = One_Four_PID.pid_out;
			steeringWheelArray[3].expextVelocity = One_Four_PID.pid_out;
			
			steeringWheelArray[1].expextVelocity = Two_Three_PID.pid_out;
			steeringWheelArray[2].expextVelocity = Two_Three_PID.pid_out;
			
		}
		
		PurePursuit_SetCurrent(&Pure_Handle, pos_world.x, pos_world.y, pos_world.yaw);
		PurePursuit_Update(&Pure_Handle);
		
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
	}
}	

//伸展
PID2 One_Four_PID, Two_Three_PID;
//遥控数据包
Pack_TransRemote_t pack_t[2];
//抓杆数据包
Arm_TransRemote_t Arm_Remote_Data;
uint8_t Action_Sign;

void Uart_Tx(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();
	pack_t[0].head = 0xAB;
	pack_t[0].tail = 0xBA;
	
	pack_t[1].head = 0xAB;
	pack_t[1].tail = 0xBA;
	
	Arm_Remote_Data.head = 0xA5;
	Arm_Remote_Data.tail = 0x5A;
	
  while(1)
  {
		//遥控数据包
		pack_t[0].expectDirection[0] = steeringWheelArray[0].expectDirection;//1和4
		pack_t[0].expectDirection[1] = steeringWheelArray[3].expectDirection;
		pack_t[0].expextVelocity[0] = steeringWheelArray[0].expextVelocity;
		pack_t[0].expextVelocity[1] = steeringWheelArray[3].expextVelocity;
		
		pack_t[1].expectDirection[0] = steeringWheelArray[1].expectDirection;//2和3
		pack_t[1].expectDirection[1] = steeringWheelArray[2].expectDirection;
		pack_t[1].expextVelocity[0] = steeringWheelArray[1].expextVelocity;
		pack_t[1].expextVelocity[1] = steeringWheelArray[2].expextVelocity;
		
    pack_t[0].crc = crc_ccitt(0, (uint8_t *)&pack_t[0], sizeof(Pack_TransRemote_t)-2);
		pack_t[1].crc = crc_ccitt(0, (uint8_t *)&pack_t[1], sizeof(Pack_TransRemote_t)-2);
		
		pack_t[0].Mode = Mode;
		pack_t[1].Mode = Mode;
		pack_t[0].Action_Sign = One_Four_Sign;
		pack_t[1].Action_Sign = Two_Three_Sign;
		pack_t[0].key = recv_pack.Key;
		pack_t[1].key = recv_pack.Key;
		if(HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY)
		{
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&pack_t[0], sizeof(Pack_TransRemote_t));
		}
		
		if(HAL_UART_GetState(&huart6) == HAL_UART_STATE_READY)
		{
			HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&pack_t[1], sizeof(Pack_TransRemote_t));
		}
		
		//抓杆
		Arm_Remote_Data.key = recv_pack.Key;
		Arm_Remote_Data.Action_Sign = Action_Sign;
		Arm_Remote_Data.mode = Mode;
		Arm_Remote_Data.crc = crc_ccitt(0, (uint8_t *)&Arm_Remote_Data, sizeof(Arm_Remote_Data)-2);
		HAL_UART_Transmit_DMA(&huart4, (uint8_t *)&Arm_Remote_Data, sizeof(Arm_TransRemote_t));
		
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(30));
  }
}

void CAN_Laser_ReceiveHandler(LASER_SEND_Typedef *laser_data, uint8_t *buf) {
	laser_data->spi1 = (uint16_t)(buf[0] | (buf[1] << 8));
	laser_data->spi2 = (uint16_t)(buf[2] | (buf[3] << 8));
	laser_data->spi3 = (uint16_t)(buf[4] | (buf[5] << 8));
	laser_data->adc = (uint16_t)(buf[6] | (buf[7] << 8));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{	
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff,size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
   	__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(huart);
		// 重置HAL状态
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_READY;
		huart->gState = HAL_UART_STATE_READY;
		
		// 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
		uint32_t isrflags = READ_REG(huart->Instance->SR);
		
		// 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
		{
				// 对于ORE、NE、FE错误，需要先读SR再读DR
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        
		}
		
		if (isrflags & USART_SR_PE)
		{
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
		}
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}

//中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8];
	uint32_t ID = CAN_Receive_DataFrame(&hcan1, Recv);
	Motor3508Recv(&Joint.RM3508_motor,hcan, ID, Recv);
	RobStrideRecv_Handle(&Joint.Rs_motor[2], hcan, ID, Recv);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	RobStrideRecv_Handle(&Joint.Rs_motor[0], hcan, ID, Recv);
	RobStrideRecv_Handle(&Joint.Rs_motor[1], hcan, ID, Recv);
	if (hcan->Instance == CAN2)
  {
    if (ID == 0x610)
    {
      CAN_Laser_ReceiveHandler(&DT_35_Len, Recv);
    }
  }
}

//斜坡
bool RampToTarget(float *val, float target, float step)
{
    float diff = target - *val;

    if(fabsf(diff) <= step)
    {
        *val = target;
        return true;
    }
    else
    {
        *val += (diff > 0.0f ? step : -step);
        return false;
    }
}
