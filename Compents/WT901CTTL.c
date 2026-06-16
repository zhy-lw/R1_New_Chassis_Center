#include "WT901CTTL.h"
#include "string.h"

static Wit_Data_t          Wit_output = {0};
static UART_HandleTypeDef   *imu_huart = NULL;
static uint8_t             wit_dma_buf[WIT_RX_BUF_SIZE] = {0};
static volatile bool       is_lock = false; // 简易数据读写互斥锁

static void Wit_Parse_DataFrame(uint8_t *frame);
static void Wit_UART_Idle_Handler(uint8_t *dma_buffer, uint16_t rx_len);


/**
 * @brief  初始化 IMU 串口配置并开启 DMA 接收
 */
void Wit_IMU_Init(UART_HandleTypeDef *huart) 
{
    if (huart == NULL)
			return;
    imu_huart = huart;

    HAL_UART_Receive_DMA(imu_huart, wit_dma_buf, WIT_RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(imu_huart, UART_IT_IDLE);
}

/**
 * @brief  在串口的中断服务函数中调用的空闲中断封装
 */
bool Wit_IMU_IRQHandler(UART_HandleTypeDef *huart) 
{
	  /**检查串口**/
    if (huart == NULL || imu_huart == NULL || huart->Instance != imu_huart->Instance) 
		{
        return false; 
    }

    /**检查是否为空闲中断触发**/
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) 
		{
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_DMAStop(huart);
        uint16_t rx_len = WIT_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
			
        Wit_UART_Idle_Handler(wit_dma_buf, rx_len);

        memset(wit_dma_buf, 0, WIT_RX_BUF_SIZE);        // 缓冲区清零
			
        HAL_UART_Receive_DMA(huart, wit_dma_buf, WIT_RX_BUF_SIZE);
        
        return true; 
    }
    return false;
}

/**
 * @brief  应用层安全获取最新IMU解算数据的接口
 */
void Wit_IMU_GetData(Wit_Data_t *p_out_data) {
    if (p_out_data == NULL) 
			return;
    
    while(is_lock);     // 如果中断正在往 Wit_output 写入数据，这里会等待锁释放，避免数据撕裂
    
    is_lock = true;
    *p_out_data = Wit_output; // 结构体整体值拷贝(嘿嘿 好东西)
    is_lock = false;
}
/**
 * @brief  解析单帧 11 字节数据包并转换为物理量
 */
static void Wit_Parse_DataFrame(uint8_t *frame) 
{
    uint8_t type = frame[1];     // 此时进入本函数的帧头和校验和已经在 Idle_Handler 中校验过了，直接解算

    int16_t data1 = (int16_t)((frame[3] << 8) | frame[2]);
    int16_t data2 = (int16_t)((frame[5] << 8) | frame[4]);
    int16_t data3 = (int16_t)((frame[7] << 8) | frame[6]);

    is_lock = true; // 开启互斥锁，防止此时应用层来读取
    switch (type) {
        case 0x51: // 加速度
            Wit_output.acc_x = (float)data1 / 32768.0f * 16.0f;				//硬件原始量/32768 * 16
            Wit_output.acc_y = (float)data2 / 32768.0f * 16.0f;
            Wit_output.acc_z = (float)data3 / 32768.0f * 16.0f;
            break;
        case 0x52: // 角速度
            Wit_output.gyro_x = (float)data1 / 32768.0f * 2000.0f;
            Wit_output.gyro_y = (float)data2 / 32768.0f * 2000.0f;
            Wit_output.gyro_z = (float)data3 / 32768.0f * 2000.0f;
            break;
        case 0x53: // 角度 (欧拉角)
            Wit_output.roll  = (float)data1 / 32768.0f * 180.0f; 
            Wit_output.pitch = (float)data2 / 32768.0f * 180.0f; 
            Wit_output.yaw   = (float)data3 / 32768.0f * 180.0f; 
            break;
        case 0x54: // 磁场
            Wit_output.mag_x = (float)data1; 
            Wit_output.mag_y = (float)data2;
            Wit_output.mag_z = (float)data3;
            break;
        default:
            break; 
    }
    is_lock = false; // 释放锁
}

/**
 * @brief  对于断帧数据的处理与解析
 */
static void Wit_UART_Idle_Handler(uint8_t *dma_buffer, uint16_t rx_len) {
    uint16_t i = 0;
    while (i + WIT_FRAME_LEN <= rx_len)
		{  
      if (dma_buffer[i] == 0x55) 
			{
					/**计算校验和**/
					uint8_t checksum = 0;
					for (uint8_t j = 0; j < 10; j++) 
					{
							checksum += dma_buffer[i + j];
					}

					if (checksum == dma_buffer[i + 10]) 
					{
							Wit_Parse_DataFrame(&dma_buffer[i]);   
							i += WIT_FRAME_LEN;                   //跳过11个字节,进入下一包数据
					} 
					else {
							i++;                                  //校验失败,则逐字节寻找帧头
					}
			} 
			else 
			{
					i++;                                      // 不是 0x55 帧头，继续扫下一个字节
			}
    }
}
