#include "Navigation.h"
#include <math.h>
#include "Task_Init.h"
#include "arm_math.h"

uint8_t run_count = 0;

void PurePursuit_Init(PurePursuitController *controller)
{
	// 初始化控制器参数
	controller->max_velocity = 1.0f;//最大速度
	controller->accel_distance = 0.3f;       // 加速距离
	controller->update_dt = 0.01f;           // 更新周期 10ms
	controller->position_tolerance = 0.04f;    // 位置容差 m
	controller->decel_distance = 0.3f;  // 减速距离 m

	// 初始化输出控制量
	controller->vx = 0.0f;
	controller->vy = 0.0f;

	controller->is_running = false;
	controller->has_arrived = false;
	
	controller->total_distance = 0.0f;
	controller->last_speed = 0.0f;
}

void PurePursuit_SetTarget(PurePursuitController *controller, float target_x, float target_y, float target_yaw)
{
	controller->target_x = target_x;
	controller->target_y = target_y;
	controller->target_theta = target_yaw;
	controller->is_running = true;
	controller->has_arrived = false;  //设置新目标时重置标志

	float dx = target_x - controller->current_x;
	float dy = target_y - controller->current_y;
	controller->total_distance = sqrtf(dx * dx + dy * dy);
}

/**
 * @brief  梯形/三角形速度规划
 *         输入：加速距离、减速距离、最大速度、总距离、当前剩余距离
 *         输出：当前周期应使用的速度（带每周期加速度限速，保证平滑）
 *
 *  物理原理：v² = 2 × a × d  →  a = v² / (2 × d)
 *    用加速距离反推加速度：a_accel = vmax² / (2 × d_acc)
 *    用减速距离反推导数度：a_decel = vmax² / (2 × d_dec)
 *
 *  两种情况：
 *    1) d_acc + d_dec <= d_total  →  梯形（加速→匀速→减速）
 *    2) d_acc + d_dec >  d_total  →  三角形（加速→减速，峰值低于vmax）
 *
 * @param remaining_distance  到目标点的剩余距离（米）
 * @param total_distance      起点到目标点的总距离（米）
 * @param accel_distance      加速距离（米）
 * @param decel_distance      减速距离（米）
 * @param max_velocity        最大速度（米/秒）
 * @param dt                  更新周期（秒）
 * @param last_speed          上一周期的速度（输入&输出，用于限速）
 * @return 当前规划的速度（米/秒）
 */
static float TrapezoidSpeedPlan(float remaining_distance, float total_distance,
                                 float accel_distance, float decel_distance,
                                 float max_velocity, float dt, float *last_speed)
{
    float a_accel = (max_velocity * max_velocity) / (2.0f * accel_distance);
    float a_decel = (max_velocity * max_velocity) / (2.0f * decel_distance);
    
    float v_peak;
    float d_acc_actual;

    if (accel_distance + decel_distance <= total_distance) {
        v_peak = max_velocity;
        d_acc_actual = accel_distance;
    } else {
        v_peak = sqrtf(2.0f * total_distance * a_accel * a_decel / (a_accel + a_decel));
        if (v_peak > max_velocity) v_peak = max_velocity;
        d_acc_actual = (v_peak * v_peak) / (2.0f * a_accel);
    }
    
    float traveled_distance = total_distance - remaining_distance;
    if (traveled_distance < 0.0f) traveled_distance = 0.0f;

    float target_speed;

    if (traveled_distance < d_acc_actual) {
        // 加速阶段
        target_speed = sqrtf(2.0f * a_accel * traveled_distance);
				
        float min_speed = a_accel * 0.1f;
        if (target_speed < min_speed && *last_speed < min_speed) {
            target_speed = min_speed;
        }
    } else if (remaining_distance > decel_distance) {
        target_speed = v_peak;
    } else {
        target_speed = sqrtf(2.0f * a_decel * remaining_distance);
    }

    if (target_speed > v_peak) target_speed = v_peak;
    if (target_speed < 0.0f)   target_speed = 0.0f;

    float max_increase = a_accel * dt;
    float max_decrease = a_decel * dt;

    float upper_limit = *last_speed + max_increase;
    float lower_limit = *last_speed - max_decrease;

    float planned_speed = target_speed;
    if (planned_speed > upper_limit) planned_speed = upper_limit;
    if (planned_speed < lower_limit) planned_speed = lower_limit;
    if (planned_speed < 0.0f) planned_speed = 0.0f;

    *last_speed = planned_speed;
    return planned_speed;
}

void PurePursuit_SetSpeedParams(PurePursuitController *controller, float accel_distance, float decel_distance)
{
    controller->accel_distance = accel_distance;//加速距离
    controller->decel_distance = decel_distance;//减速距离
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

bool x_ok = false;
bool y_ok = false;
float planned_speed = 0;
float dx_body;
float dy_body;
void PurePursuit_Update(PurePursuitController* controller)
{
    float dx_world = controller->target_x - controller->current_x;
    float dy_world = controller->target_y - controller->current_y;
    
    float cos_theta = arm_cos_f32(controller->current_theta);
    float sin_theta = arm_sin_f32(controller->current_theta);
    
    dx_body = -(dx_world * cos_theta + dy_world * sin_theta);
    dy_body = dx_world * sin_theta - dy_world * cos_theta;
    
		y_ok = fabsf(dy_body) <= controller->position_tolerance;
    x_ok = fabsf(dx_body) <= controller->position_tolerance; 
    if (controller->has_arrived) {
        controller->vx = 0.0f;
        controller->vy = 0.0f;
        return;
    }
    
    if (x_ok && y_ok) {
        controller->vx = 0.0f;
        controller->vy = 0.0f;
				vTaskDelay(20);
        controller->is_running = false;
        controller->has_arrived = true;
        run_count++;
        return;
    }
    
    //计算剩余距离
    float remaining_distance = sqrtf(dx_body * dx_body + dy_body * dy_body);
    
    //调用梯形速度规划，得到平滑后的速度大小
		planned_speed= TrapezoidSpeedPlan(
        remaining_distance,
        controller->total_distance,
        controller->accel_distance,
        controller->decel_distance,
        controller->max_velocity,
        controller->update_dt,
        &controller->last_speed
    );
    
    //保持运动方向不变，用规划后的速度大小乘以方向
    float target_angle_body = atan2f(dy_body, dx_body);
    controller->vx = arm_cos_f32(target_angle_body) * planned_speed;
    controller->vy = arm_sin_f32(target_angle_body) * planned_speed;
    
    //某个轴方向到了就把该方向速度清0
    if (x_ok) controller->vx = 0.0f;
    if (y_ok) controller->vy = 0.0f;
}
