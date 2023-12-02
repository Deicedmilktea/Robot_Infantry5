#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
pid_struct_t motor_pid_chassis[4];
pid_struct_t supercap_pid;
motor_info_t  motor_can1[8];       //can1电机信息结构体, 1234：底盘，5：云台
fp32 chassis_motor_pid [3]={30,0.5,10};   //用的原来的pid
fp32 superpid[3] = {120,0.1,0};
volatile int16_t Vx=0,Vy=0,Wz=0;
int16_t Temp_Vx;
int16_t Temp_Vy;
int fllowflag = 0;
volatile int16_t motor_speed_target[4];
extern RC_ctrl_t rc_ctrl;
extern ins_data_t ins_data;
extern float powerdata[4];
extern uint16_t shift_flag;
 
int error10 = 0;
fp32 speed10 = 0;
 
double rx=0.2,ry=0.2;
// Save imu data

int16_t chassis_mode = 1;//判断底盘状态，用于UI编写
int16_t shot_mode = 0;

//获取imu——Yaw角度差值参数
static void Get_Err(); 

//参数重置
static void Chassis_loop_Init(); 

//super_cap
void power_limit(int *speed);

int chassis_mode_flag =0;

void qe();

//speed mapping
int16_t Speedmapping(int value, int from_min, int from_max, int to_min, int to_max){
	  // 首先将输入值从 [a, b] 映射到 [0, 1] 范围内
    double normalized_value = (value*1.0 - from_min) / (from_max - from_min);
    
    // 然后将标准化后的值映射到 [C, D] 范围内
    int16_t mapped_value = (int16_t)(to_min + (to_max - to_min) * normalized_value);
    
    return mapped_value;
}

void Calculate_speed(){
	Vx=Speedmapping(rc_ctrl.rc.ch[2],-660,660,-1000,1000);// left and right
	Vy=Speedmapping(rc_ctrl.rc.ch[3],-660,660,-1000,1000);// front and back
	Wz=Speedmapping(rc_ctrl.rc.ch[4],-660,660,-1000,1000);// rotate      
}

void RC_move(){
		motor_speed_target[CHAS_LF] =  Vy + Vx + 3*Wz*(rx+ry);
    motor_speed_target[CHAS_RF] = -Vy + Vx + 3*Wz*(rx+ry);
    // motor_speed_target[CHAS_RB] = -Vy - Vx + 3*Wz*(rx+ry);
    // motor_speed_target[CHAS_LB] =  Vy - Vx + 3*Wz*(rx+ry);
    motor_speed_target[CHAS_RB] =  Vy - Vx + 3*Wz*(rx+ry);
    motor_speed_target[CHAS_LB] = -Vy - Vx + 3*Wz*(rx+ry); // 五号换位置
}
	
#define angle_valve 5
#define angle_weight 55
 
void Chassis_task(void const *pvParameters)
{
 			for (uint8_t i = 0; i < 4; i++)
			{
        pid_init(&motor_pid_chassis[i], chassis_motor_pid, 6000, 6000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
			}
				pid_init(&supercap_pid, superpid, 3000, 3000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384			
				
  
    for(;;)
    {   
				chassis_mode = rc_ctrl.rc.s[0];//1，3，2
				Calculate_speed();
				if(chassis_mode==1){
						motor_speed_target[CHAS_LF] =  1000;
						motor_speed_target[CHAS_RF] =  0;
						motor_speed_target[CHAS_RB] =  0;
						motor_speed_target[CHAS_LB] =  0;
				}
				if(chassis_mode==2){
						RC_move();
				}

				RC_move();

				chassis_current_give();
				error10++;
        osDelay(1);

    }

}



static void Chassis_loop_Init()
{
	Vx = 0;
	Vy = 0;
	Wz = 0;
}

//运动解算
void chassis_motor_speed_calculate()
{
	
	  motor_speed_target[CHAS_LF] =  0;
    motor_speed_target[CHAS_RF] =  0;
    motor_speed_target[CHAS_RB] =  0;
    motor_speed_target[CHAS_LB] =  0;
}
//运动解算
//速度限制函数
  void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed)  
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = limit_speed;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//求绝对值
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
					{
            motor_speed[i] *= rate;
					}

    }

}
//电机电流控制
void chassis_current_give() 
{
	
    uint8_t i=0;
        
    for(i=0 ; i<4; i++)
    {
        motor_can1[i].set_current = pid_calc(&motor_pid_chassis[i], motor_speed_target[i], motor_can1[i].rotor_speed);
    }
		
    	set_motor_current_can1(0, motor_can1[0].set_current,  motor_can1[1].set_current,  motor_can1[2].set_current, motor_can1[3].set_current);
}






