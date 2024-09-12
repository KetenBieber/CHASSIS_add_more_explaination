/**
 * @file chassis_task.cpp
 * @author Yang JianYi
 * @brief 舵轮底盘应用文件，包括底盘配置的初始化以及控制接口的调用
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "chassis_task.h"

/* 
    底盘任务函数

 */
void Chassis_Task(void *pvParameters)
{
    static Robot_Twist_t twist;// 创建Robot_Twist_t类型的结构体，存放输出的控制量
    Robot_Twist_t real_twist;// 创建Robot_Twist_t类型的结构体，存放逆向解算的底盘信息(机器人坐标系)
    for(;;)
    {   
        if(xQueueReceive(Chassia_Port, &twist, 0) == pdPASS)
        {
            //底盘控制、电机控制    
            chassis.Control(twist);// 底盘速度控制，准备电机速度，装入can信号帧
			chassis.Motor_Control();// 发送电机的can信号帧
        }
        
        //未测试，将逆向解算的底盘速度(机器人坐标系)发送到ROS
        // real_twist = chassis.Get_Robot_Speed();
        // xQueueSend(Send_ROS_Port, &real_twist, 0);
			
        osDelay(1);
    }
}

/* 
    底盘pid参数初始化
    
 */
void Chassis_Pid_Init(void)
{   
    chassis.accel_vel = 1.5;

    chassis.Pid_Param_Init(RUDDER_LEFT_FRONT_Speed_E,12, 0.1, 0, 400, 30000, 0);
    chassis.Pid_Param_Init(RUDDER_RIGHT_FRONT_Speed_E,12, 0.1, 0, 400, 30000, 0);
    chassis.Pid_Param_Init(RUDDER_LEFT_REAR_Speed_E,12, 0.1, 0, 400, 30000, 0);
    chassis.Pid_Param_Init(RUDDER_RIGHT_REAR_Speed_E,12, 0.1, 0, 400, 30000, 0);

    chassis.Pid_Param_Init(RUDDER_LEFT_FRONT_Pos_E,120, 0, 0.2, 400, 2000, 0.2);
    chassis.Pid_Param_Init(RUDDER_RIGHT_FRONT_Pos_E,120, 0, 0.2, 400, 2000, 0.2);
    chassis.Pid_Param_Init(RUDDER_LEFT_REAR_Pos_E,120, 0, 0.2, 400, 2000, 0.2);
    chassis.Pid_Param_Init(RUDDER_RIGHT_REAR_Pos_E,120, 0, 0.2, 400, 2000, 0.2);

    chassis.Pid_Mode_Init(RUDDER_LEFT_FRONT_Speed_E, 0.8, 1, true, true);
    chassis.Pid_Mode_Init(RUDDER_RIGHT_FRONT_Speed_E, 0.8, 1, true, true);
    chassis.Pid_Mode_Init(RUDDER_LEFT_REAR_Speed_E, 0.8, 1, true, true);
    chassis.Pid_Mode_Init(RUDDER_RIGHT_REAR_Speed_E, 0.8, 1, true, true);

    chassis.Pid_Mode_Init(RUDDER_LEFT_FRONT_Pos_E, 0.8, 0.1, true, false);
    chassis.Pid_Mode_Init(RUDDER_RIGHT_FRONT_Pos_E, 0.8, 0.1, true, false);
    chassis.Pid_Mode_Init(RUDDER_LEFT_REAR_Pos_E, 0.8, 0.1, true, false);
    chassis.Pid_Mode_Init(RUDDER_RIGHT_REAR_Pos_E, 0.8, 0.1, true, false);
	
	float lf_offset=(53.0f+15+180)/360 * 8192.0f;
	float rf_offset=(53.0f+60+180)/360 * 8192.0f;
	float rr_offset = (53.0f+120)/360 * 8192.0f;
	float lr_offset = (53.0f+0.0f)/360 * 8192.0f; 
    RudderMotor[0].set_encoder_offset(lf_offset);
    RudderMotor[1].set_encoder_offset(rf_offset);
    RudderMotor[2].set_encoder_offset(rr_offset);
    RudderMotor[3].set_encoder_offset(lr_offset);

    chassis.Speed_Max.linear.x = 3;
    chassis.Speed_Max.linear.y = 3;
    chassis.Speed_Max.angular.z = 4;
}
