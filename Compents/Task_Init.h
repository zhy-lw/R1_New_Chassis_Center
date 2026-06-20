#ifndef __TASK_INIT_H
#define __TASK_INIT_H

#include "drive_callback.h"
#include "ForceChassis.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usb_device.h"
#include "motorEx.h"
#include "PID_old.h"
#include "RobStride2.h"
#include "motorEx.h"
#include "WT901CTTL.h"

#define MAX_ROBOT_VEL 1.2f
#define MAX_ROBOT_OMEGA ANGLE2RAD(60.0f)
#define arm1 200.0f
#define arm2 210.0f
#define Initial_Laser_Distance 150.0f

extern Wit_Data_t my_imu;

void Task_Init(void);

typedef struct{
    RobStride_t   Rs_motor[3];
    Motor3508Ex_t RM3508_motor;
		
    float pos_offset[3];
    float exp_rad[2];
    PID2  Rs_vel_pid[2];
    PID2  Rs_pos_pid[2];
}Joint_t;

typedef struct{
	uint8_t Left_Key_Up;         
	uint8_t Left_Key_Down;       
	uint8_t Left_Key_Left;       
	uint8_t Left_Key_Right;       
	uint8_t Left_Switch_Up;       
	uint8_t Left_Switch_Down;
	uint8_t Left_Broadside_Key;

	uint8_t Right_Key_Up;        
	uint8_t Right_Key_Down;      
	uint8_t Right_Key_Left;      
	uint8_t Right_Key_Right;     
	uint8_t Right_Switch_Up;      
	uint8_t Right_Switch_Down;      
	uint8_t Right_Broadside_Key;
} hw_key_t;

typedef struct {
    float Ex;
    float Ey;
    float Eomega;
    hw_key_t First,Second;
} Remote_Handle_t;

typedef enum{
    STOP,
    REMOTE,
    AUTO,
		STRETCH
}ChassisMode;

#pragma pack(1)
typedef struct
{
    uint8_t head;
    float expectDirection[2];
    float expextVelocity[2];
		uint32_t key;
		ChassisMode Mode;
		uint8_t Action_Sign;
    uint8_t tail;
		uint16_t crc;
} Pack_TransRemote_t;
#pragma pack()

#pragma pack(1)
typedef struct {
    uint16_t spi1;
    uint16_t spi2;
    uint16_t spi3;
    uint16_t adc;
} LASER_SEND_Typedef;   //激光句柄
#pragma pack()

#pragma pack(1)
struct Pose3DPacket
{
    uint8_t header;
    float x;
    float y;
    float yaw;
    uint8_t tail;  //车体坐标
};
#pragma pack()

#pragma pack(1)
typedef struct {
	uint8_t head;
	uint32_t key;
	ChassisMode mode;
	uint8_t Action_Sign;
	uint8_t tail;
	uint16_t crc;
}Arm_TransRemote_t;
#pragma pack()

bool RampToTarget(float *val, float target, float step);

#endif
