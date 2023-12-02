/*
**********Shoot_task射击任务**********
包含对两个摩擦轮以及拨盘的控制
拨盘为2006，ID = 5，CAN2控制
弹仓盖为2006， ID = 6， CAN2控制
摩擦轮分别为3508，ID = 7（up）和 ID = 8（down），CAN2 控制
*/

#include "Shoot_task.h"
#include "pid.h"

shoot_t shoot_motor[4]; //id = 5678
motor_info_t motor_can2[5]; //[4]:pitch
extern RC_ctrl_t rc_ctrl;


void Shoot_task(void const * argument)
{
  shoot_loop_init();

  for(;;)
  {
    //遥控器右边拨到上，电机启动
    if(rc_ctrl.rc.s[0] == 2){
      shoot_start();
    }
    else{
      shoot_stop();
    }

    // //遥控器左边拨到下，弹仓盖打开
    // if(rc_ctrl.rc.s[1] == 2){
    //   shoot_lid_open();
    // }
    // else{
    //   shoot_lid_close();
    // }

    shoot_current_give();
    osDelay(1);
  }
}


/***************初始化***************/
void shoot_loop_init()
{
  // trigger
  shoot_motor[0].pid_value[0] = 10;
  shoot_motor[0].pid_value[1] = 0;
  shoot_motor[0].pid_value[2] = 0;

  // lid
  shoot_motor[1].pid_value[0] = 10;
  shoot_motor[1].pid_value[1] = 0;
  shoot_motor[1].pid_value[2] = 0;

  // friction_up
  shoot_motor[2].pid_value[0] = 10;
  shoot_motor[2].pid_value[1] = 0;
  shoot_motor[2].pid_value[2] = 0;

  // friction_down
  shoot_motor[3].pid_value[0] = 10;
  shoot_motor[3].pid_value[1] = 0;
  shoot_motor[3].pid_value[2] = 0;

  // 初始化目标速度
  shoot_motor[0].target_speed = 0;
  shoot_motor[1].target_speed = 0;
  shoot_motor[2].target_speed = 0;
  shoot_motor[3].target_speed = 0;

  // 初始化PID
  pid_init(&shoot_motor[0].pid, shoot_motor[0].pid_value, 1000, 1000); // trigger
  pid_init(&shoot_motor[1].pid, shoot_motor[1].pid_value, 1000, 1000); // lid
  pid_init(&shoot_motor[2].pid, shoot_motor[2].pid_value, 1000, 1000); // friction_up
  pid_init(&shoot_motor[3].pid, shoot_motor[3].pid_value, 1000, 1000); // friction_down
}

/***************射击模式*****************/
void shoot_start()
{
  shoot_motor[0].target_speed = 1000;
  shoot_motor[1].target_speed =    0; //弹舱盖特殊处理
  shoot_motor[2].target_speed = 1000;
  shoot_motor[3].target_speed = 1000;
}

/***************停止射击模式**************/
void shoot_stop()
{
  shoot_motor[0].target_speed = 0;
  shoot_motor[1].target_speed = 0;
  shoot_motor[2].target_speed = 0;
  shoot_motor[3].target_speed = 0;
}


// /********************************can2发送电流***************************/
// void shoot_can2_cmd(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
// {
//   uint32_t send_mail_box;
//   CAN_TxHeaderTypeDef tx_header;
//   uint8_t             tx_data[8];
    
//   tx_header.StdId = 0x1FF;
//   tx_header.IDE   = CAN_ID_STD;//标准帧
//   tx_header.RTR   = CAN_RTR_DATA;//数据帧
	
//   tx_header.DLC   = 8;		//发送数据长度（字节）

//   tx_data[0] = (v1>>8)&0xff;
//   tx_data[1] =    (v1)&0xff;
//   tx_data[2] = (v2>>8)&0xff;
//   tx_data[3] =    (v2)&0xff;
//   tx_data[4] = (v3>>8)&0xff;
//   tx_data[5] =    (v3)&0xff;
//   tx_data[6] = (v4>>8)&0xff;
//   tx_data[7] =    (v4)&0xff;

//   HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
// }


/********************************PID计算速度并发送电流****************************/
void shoot_current_give()
{
    for(int i=0 ; i<4; i++)
    {
      motor_can2[i].set_current = pid_calc(&shoot_motor[i].pid, shoot_motor[i].target_speed, motor_can2[i].rotor_speed);
    }

    set_motor_current_can2(1, motor_can2[0].set_current, motor_can2[1].set_current, motor_can2[2].set_current, motor_can2[3].set_current);
}

