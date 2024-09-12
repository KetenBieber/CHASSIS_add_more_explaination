/**
 * @file service_config.cpp
 * @author Yang JianYi
 * @brief 配置文件，用于初始化系统资源和我们自己定义的应用程序初始化函数
 * @version 0.1
 * @date 2024-05-16
 * 
 */
#include "service_config.h"
#include "chassis_task.h"

/* 
    创建一个舵轮底盘类
    构造函数参数为舵轮的轮子半径、舵轮的轴距、舵轮的轮子直径、舵轮的轮子数量
 */
Swerve_Chassis chassis(0.055,0,0.321,4);

void System_Resource_Init(void)
{
    DataPool_Init();
    Timer_Init(&htim4,USE_HAL_DELAY);
    PWM_ReInit(4200-1,40000-1,&htim10,TIM_CHANNEL_1);
    CAN_Init(&hcan1,CAN1_RxCallBack);
    CAN_Init(&hcan2,CAN2_RxCallBack);
    CAN_Filter_Init(&hcan1,CanFilter_0|CanFifo_0|Can_STDID|Can_DataType,0,0);
    CAN_Filter_Init(&hcan2,CanFilter_14|CanFifo_0|Can_EXTID|Can_DataType,0,0);
    CAN_Filter_Init(&hcan1,CanFilter_1|CanFifo_1|Can_STDID|Can_DataType,0,0);
    CAN_Filter_Init(&hcan2,CanFilter_15|CanFifo_1|Can_EXTID|Can_DataType,0,0);
    Uart_Init(&huart3, Uart3_Rx_Buff, 50, ROS_UART3_RxCallback);// 这里修改了 Buffer 为一个数据包的 两倍大小，测试 dma半满中断+接收完成中断
    App_Init();
}

/* 
    Application任务初始化:
        初始化数据池
        初始化定时器
        初始化PWM
        初始化CAN
        初始化UART
        初始化应用程序
    （可算是让我找到了
    在这里注册所有的定时器中断函数，在这里将Get_SystemTimer()注册到所有需要获取当前时间的类中
        （语法原理就是将函数指针填入）
    控制器执行需要准确的时间戳和时间间隔更新，所以需要使用硬件定时器来保证控制频率
    PioTimer
    AirJoy
    ROS
    Chassis_Base
    Broadcast
 */
void App_Init(void)
{
    Set_PwmDuty(&htim10, TIM_CHANNEL_1, 0);
    Chassis_Pid_Init();
    PidTimer::getMicroTick_regist(Get_SystemTimer);
    AirJoy::getMicroTick_regist(Get_SystemTimer);
    ROS::getMicroTick_regist(Get_SystemTimer);
    Chassis_Base::getMicroTick_regist(Get_SystemTimer);
    Broadcast::getMicroTick_regist(Get_SystemTimer);
    // motor_init();
}

void motor_init(void)
{
    DM43.Motor_Status = CMD_MOTOR_ENABLE;
    Motor_SendMsgs(&hcan1, DM43);
}
