#pragma once
#include "stdint.h"
#include "drive_uart.h"
#include "data_pool.h"
#include "tool.h"

/* 
    解包ROS方发送的数据
    
 */
typedef struct readFromRos
{
    float x;
    float y;
    float z;
    uint8_t ctrl_mode;
    uint8_t ctrl_flag;
    uint8_t chassis_init;
    Robot_Status_t status;
}readFromRos;

typedef uint32_t (*SystemTick_Fun)(void);

#ifdef __cplusplus

/* 
    ROS 通讯封装类
        继承自 Tool工具类
    1.构造函数中：进行头帧和尾帧的初始化
    2.成员函数包括：
        发送数据到ROS---Send_To_ROS
        接收数据从ROS---Recieve_From_ROS
        用来存放解析后的数据结构体---readFromRos
        用来注册获取系统时间的函数---getMicroTick_regist
        注册获取系统时间的函数指针---get_systemTick
    private：
        串口发送数据结构体
        头帧和尾帧
        数据长度
 */

class ROS : Tools
{
public:
    ROS()
    {
        header[0] = 0x55;
        header[1] = 0xAA;
        tail[0] = 0x0D;
        tail[1] = 0x0A;
    }
    void Send_To_ROS(Robot_Twist_t speed);
    int8_t Recieve_From_ROS(uint8_t *buffer);
    readFromRos readFromRosData;
    static uint8_t getMicroTick_regist(uint32_t (*getTick_fun)(void));
    static SystemTick_Fun get_systemTick;
    
private:

    UART_TxMsg TxMsg;
    uint8_t header[2];
    uint8_t tail[2];
    uint8_t lenth=0;
};

#endif
