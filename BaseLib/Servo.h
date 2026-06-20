#ifndef __SERVO_H__
#define __SERVO_H__

#include "tim.h"

int32_t RAMP_self( int32_t final, int32_t now, int32_t ramp );
void Servo_control(void) ;  
#endif
