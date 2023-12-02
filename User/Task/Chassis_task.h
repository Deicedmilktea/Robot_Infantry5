#ifndef CHASSIS_TASK_H
#define  CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include  "drv_can.h"
#include "rc_potocal.h"
#include "main.h"

typedef struct
{
    uint16_t can_id;
    int16_t  set_current;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
}motor_info_t;


typedef enum {
    CHAS_LF,
    CHAS_RF,
    CHAS_RB,
    CHAS_LB,
} chassis_motor_cnt_t;

extern int16_t Drifting_yaw;
extern uint16_t Down_ins_yaw;

void Chassis_task(void const *pvParameters);
void RC_to_Vector(void);
void chassis_motor_speed_calculate(void);
void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed);
void chassis_current_give(void);
#endif
