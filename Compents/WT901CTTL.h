#ifndef __WT901CTTL_H_
#define __WT901CTTL_H_

#include "main.h"    
#include "stdbool.h"

#define WIT_FRAME_LEN   11  // 维特协议固定单帧长度：11字节
#define WIT_RX_BUF_SIZE 64  // DMA 接收缓冲区大小

typedef struct {
    // 0x51 加速度包 (单位: g)
    float acc_x;
    float acc_y;
    float acc_z;
    
    // 0x52 角速度包 (单位: deg/s)
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    // 0x53 角度包 (单位: deg)
    float roll;   // 横滚角 (X轴)
    float pitch;  // 俯仰角 (Y轴)
    float yaw;    // 航向角 (Z轴)
	
		// 0x54 磁场输出包 (单位: 毫高斯 mGauss 或 磁场原始单位)
    float mag_x;
    float mag_y;
    float mag_z;
} Wit_Data_t;

//外部接口函数
void Wit_IMU_Init(UART_HandleTypeDef *huart);
bool Wit_IMU_IRQHandler(UART_HandleTypeDef *huart);
void Wit_IMU_GetData(Wit_Data_t *p_out_data);

#endif

