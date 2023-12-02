#ifndef PITCH_TASK_H
#define PITCH_TASK_H

#include "main.h"
#include "pid.h"

typedef struct
{
    pid_struct_t pid_angle;      // 云台电机angle的pid结构体
    pid_struct_t pid_speed;      // 云台电机speed的pid结构体
    fp32 pid_angle_value[3];     // 云台电机angle的pid参数
    fp32 pid_speed_value[3];     // 云台电机speed的pid参数
    fp32 target_angle;           // 云台电机的目标角度
    fp32 target_speed;           // 云台电机的目标速度
} pitch_t;


void Pitch_task(void const * argument);

// 初始化
void pitch_loop_init();

/*can1发送电流*/
void pitch_can1_cmd(int16_t v3);

// PID计算速度并发送电流
void pitch_current_give();

// pitch角度限制
void pitch_angle_limit(int16_t angle, int16_t angle1, int16_t angle2);

#endif