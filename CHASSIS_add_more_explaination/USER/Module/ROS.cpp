/**
 * @file ROS.cpp
 * @brief 上下位机的通信文件，包括数据的打包和解包，数据的发送和接收。使用串口3DMA
 * @version 0.1
 * @date 2024-04-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "ROS.h"

/* 
    ros串口解包协议：
        头帧：2 个字节
        数据长度：1 个字节
        尾帧：2 个字节
        x、y、z 坐标：每个坐标 4 个字节，共 12 个字节
        控制模式、控制标志、底盘初始化：每个 1 个字节，共 3 个字节
        状态字段：每个字段 1 个字节，共 4 个字段（robot_init、path_mode、sensor、control_mode），共 4 个字节
        CRC 校验：1 个字节
    总共25个字节
 */

SystemTick_Fun ROS::get_systemTick = NULL;

union ROS_data
{
    float f;
    uint8_t c[4];
}x,y,z;


uint8_t ROS::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
    if(getTick_fun != NULL)
    {
        ROS::get_systemTick = getTick_fun;
        return 1;
    }
    else 
        return 0;
}


/**
 * @brief stm32 send data to ROS
 * @note 该函数用于将数据打包发送给ROS, 通过轮子速度解算后的速度转化为了mm/s和mRad/s
 */
void ROS::Send_To_ROS(Robot_Twist_t speed)
{
    uint8_t buffer[6+6];
    int index = 0;
    buffer[index++] = header[0];
    buffer[index++] = header[1];
    buffer[index++] = 1;
    _tool_buffer_append_int16(buffer, (int16_t)(speed.linear.x*1000), &index);
    _tool_buffer_append_int16(buffer, (int16_t)(speed.linear.y*1000), &index);
    _tool_buffer_append_int16(buffer, (int16_t)(speed.angular.z*1000), &index);
    buffer[index++] = serial_get_crc8_value(buffer, 4);
    buffer[index++] = tail[0];
    buffer[index++] = tail[1];
}


/**
 * @brief upack the data from ROS 解包ros 发送的数据包，并将数据存进readFromRosData中，可以通过调用该类的成员函数获取数据
 * @param buffer pack that recieved from ROS
 * @return int8_t unpack success return 0, else return 1
 */
int8_t ROS::Recieve_From_ROS(uint8_t *buffer)
{
    int index = 0;
    // 检查头帧
    for(int i = 0; i < 2; i++)
    {
        if(buffer[index++] != header[i])
            return 2;
    }
    // 将 代表数据长度（data）的一帧 放入 lenth 变量
    lenth = buffer[index++];

    // 检查尾帧
    for(int i=0; i<2; i++)
    {
        if(buffer[4+lenth+i] != tail[i])
            return 1;
    }
    
    for(int i=0; i<4; i++)
    {
        x.c[i] = buffer[index++];
    }

    for(int i=0; i<4; i++)
    {
        y.c[i] = buffer[index++];
    }

    for(int i=0; i<4; i++)
    {
        z.c[i] = buffer[index++];
    }

    readFromRosData.x = x.f;// 使用联合体直接实现多字节数据合并
    readFromRosData.y = y.f;
    readFromRosData.z = z.f;
    readFromRosData.ctrl_mode = buffer[index++];// uint8_t 是 1字节大小 的数据类型
    readFromRosData.ctrl_flag = buffer[index++];
    readFromRosData.chassis_init = buffer[index++];// bool 类型数据 也是 1字节大小
    readFromRosData.status.robot_init = (PLAYLIST)buffer[index++];// PLAYLIST 是枚举类型，枚举类型通常是用一个字节数据来表示的
    readFromRosData.status.path_mode = (PLAYLIST)buffer[index++];
    readFromRosData.status.sensor = (PLAYLIST)buffer[index++];
    readFromRosData.status.control_mode = (PLAYLIST)buffer[index++];

    if(buffer[index++]!=serial_get_crc8_value(buffer, lenth+3))// 检查最后一位 crc检验位
    {
        // 如果crc检验没过，直接撤销
        readFromRosData.x = 0;
        readFromRosData.y = 0;
        readFromRosData.z = 0;
        readFromRosData.ctrl_mode = NORMAL;
        readFromRosData.ctrl_flag = 0;
        readFromRosData.chassis_init = false;
    }

    return 0;
}
