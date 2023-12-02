/*
*************pitch轴任务**************
采用3508，ID = 7，CAN2，motor_can2[4]
使用遥控右拨杆的上下控制
*/
#include "Pitch_task.h"
#include "Chassis_task.h"
#include "cmsis_os.h"

extern motor_info_t motor_can2[5];
extern RC_ctrl_t rc_ctrl;
pitch_t pitch;

void Pitch_task(void const * argument)
{
    pitch_loop_init();
    pitch.target_angle = motor_can2[4].rotor_angle;

    for(;;)
    {
        pitch.target_angle += rc_ctrl.rc.ch[1]/300;
        pitch_angle_limit(pitch.target_angle, 1600, 3400); //根据实际修改
		pitch_current_give();
        osDelay(1);
    }
}

/****************初始化****************/
void pitch_loop_init()
{
    pitch.pid_angle_value[0] = 10;
    pitch.pid_angle_value[1] = 0;
    pitch.pid_angle_value[2] = 0;

    pitch.pid_speed_value[0] = 5;
    pitch.pid_speed_value[1] = 0;
    pitch.pid_speed_value[2] = 0;

    pitch.target_angle = 0;
    pitch.target_speed = 0;

    pid_init(&pitch.pid_angle, pitch.pid_angle_value, 100, 3500);
    pid_init(&pitch.pid_speed, pitch.pid_speed_value, 100, 5000);
}

/********************************can1发送电流***************************/
void pitch_can2_cmd(int16_t v3)
{
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = 0x2FF;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
	
  tx_header.DLC   = 8;		//发送数据长度（字节）

  tx_data[0] =         NULL;
  tx_data[1] =         NULL;
  tx_data[2] =         NULL;
  tx_data[3] =         NULL;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] =         NULL;
  tx_data[7] =         NULL;

  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
}

/****************PID计算速度并发送电流***************/
void pitch_current_give()
{
    pitch.target_speed = pid_calc(&pitch.pid_angle, pitch.target_angle, motor_can2[4].rotor_angle);
    motor_can2[4].set_current = pid_calc(&pitch.pid_speed, pitch.target_speed, motor_can2[4].rotor_speed);
    pitch_can2_cmd(motor_can2[4].set_current);
}

/*************pitch角度限制********************/
void pitch_angle_limit(int16_t angle, int16_t angle1, int16_t angle2)
{
   if (angle < angle1)
   {
       angle = angle1;
   }
   else if (angle > angle2)
   {
       angle = angle2;
   }
}