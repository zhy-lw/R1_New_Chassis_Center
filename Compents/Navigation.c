#include "Navigation.h"
#include <math.h>
#include "Task_Init.h"
#include "arm_math.h"

uint8_t run_count = 0;

void PurePursuit_Init(PurePursuitController *controller)
{
    // 初始化控制器参数
    controller->max_velocity = 1.0f;//最大速度
    controller->k_lateral = 0.0f;//横向误差增益

    // 容差参数
    controller->position_tolerance = 0.04f;//位置容差

    // 初始化输出控制量
    controller->vx = 0.0f;
    controller->vy = 0.0f;

    controller->is_running = false;
}

void PurePursuit_SetTarget(PurePursuitController *controller, float target_x, float target_y, float target_yaw)
{
    controller->target_x = target_x;
    controller->target_y = target_y;
		controller->target_theta = target_yaw;
    controller->is_running = true;
    controller->has_arrived = false;  //设置新目标时重置标志
}

void PurePursuit_SetCurrent(PurePursuitController *controller, float current_x, float current_y, float current_theta)
{
    controller->current_x = current_x;
    controller->current_y = current_y;
		controller->current_theta = current_theta;
}

float NormalizeAngle(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

float CalculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

float CalculateLateralError(float current_x, float current_y,
                           float target_x, float target_y,
                           float current_theta) {
    float dx = target_x - current_x;
    float dy = target_y - current_y;
    float theta_rad = current_theta;
    return -arm_sin_f32(theta_rad) * dx + arm_cos_f32(theta_rad) * dy;
}
bool x_ok = false;
bool y_ok = false;
void PurePursuit_Update(PurePursuitController* controller)
{
    float dx_world = controller->target_x - controller->current_x;
    float dy_world = controller->target_y - controller->current_y;
		
    float cos_theta = arm_cos_f32(controller->current_theta);
    float sin_theta = arm_sin_f32(controller->current_theta);
		
    float dx_body = -(dx_world * cos_theta + dy_world * sin_theta);
    float dy_body = dx_world * sin_theta - dy_world * cos_theta;
    
    x_ok = fabs(dx_body) < controller->position_tolerance;
    y_ok = fabs(dy_body) < controller->position_tolerance;
    
		if (controller->has_arrived) {
        return;
    }
	
    if (x_ok && y_ok) {
        controller->vx = 0.0f;
        controller->vy = 0.0f;
				vTaskDelay(20);
        controller->is_running = false;
				controller->has_arrived = true;  //标记为已到达
				run_count ++;
				return;
    }
    
    float distance_body = sqrtf(dx_body * dx_body + dy_body * dy_body);
    float speed_scale = distance_body;
    
    // 直接在车体坐标系下计算速度
    float target_angle_body = atan2(dy_body, dx_body);
    controller->vx = arm_cos_f32(target_angle_body) * controller->max_velocity * speed_scale;
    controller->vy = arm_sin_f32(target_angle_body) * controller->max_velocity * speed_scale;
    
    // 那个方向到位置，那个速度为0
    if (x_ok) {
        controller->vx = 0.0f;
    }
    if (y_ok) {
        controller->vy = 0.0f;
    }
    
    float v_total = sqrtf(controller->vx * controller->vx + controller->vy * controller->vy);
    if (v_total > controller->max_velocity) {
        float scale = controller->max_velocity / v_total;
        controller->vx = controller->vx * scale;
        controller->vy = controller->vy * scale;
    }
}
