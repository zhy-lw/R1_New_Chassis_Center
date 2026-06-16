#ifndef _NAVIGATION__H_
#define _NAVIGATION__H_

#include <stdint.h>
#include <stdbool.h>

// ============================================
// Pure Pursuit 控制器结构体
// ============================================
typedef struct {
    // ====== 输入：当前位置（从传感器获取）======
    float current_x;       // 当前X坐标（米）
    float current_y;       // 当前Y坐标（米）
    float current_theta;   // 当前航向角（度）
    
    // ====== 输入：目标位置 ======
    float target_x;        // 目标X坐标（米）
    float target_y;        // 目标Y坐标（米）
    float target_theta;    // 目标航向角（度）
    
    // ====== 控制参数 ======
    float max_velocity;       // 最大速度（米/秒）
		float accel_distance;    	// 加速距离（米）：从0加速到vmax所需距离
		float update_dt;         // 控制器更新周期（秒），用于每周期加速度限速
    float decel_distance;     // 减速距离（米），开始减速时距离目标的距离
	
    // ====== 容差参数 ======
    float position_tolerance;  // 位置容差（米）
    
    // ====== 输出：控制量（车体坐标系）======
    float vx;              // X方向速度（米/秒）
    float vy;              // Y方向速度（米/秒）
    
    // ====== 运行状态 ======
    bool is_running;          // 是否正在运行
		bool has_arrived;         // 是否已经到达并计数
		
		// ====== 梯形规划内部状态 ======
    float total_distance;     // SetTarget时记录的总距离（米）
    float last_speed;         // 上一次的规划速度（米/秒），用于每周期加速度限速
} PurePursuitController;

void PurePursuit_SetSpeedParams(PurePursuitController *controller, float accel_distance, float decel_distance);
void PurePursuit_Init(PurePursuitController *controller);
void PurePursuit_SetTarget(PurePursuitController *controller, float target_x, float target_y, float target_yaw);
void PurePursuit_Update(PurePursuitController* controller);
void PurePursuit_SetCurrent(PurePursuitController *controller, float current_x, float current_y, float current_theta);

#endif