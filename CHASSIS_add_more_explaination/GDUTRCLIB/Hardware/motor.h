/**
 * @file motor.h
 * @author Yang JianYi (287643517@qq.com)
 * @brief 电机调用函数，每款电机驱动器都已经封装成一个类，通过调用类的函数来实现电机的控制。包括C610、C620、GM6020、VESC等。
 *        如果要新增电机类，请务必继承Motor_Base类，以保证接口的统一性。
 *        考虑增加达妙电机的驱动文件。
 * 
 * ************************************************************************************************************
 * 使用说明：
 *   Motor_SendMsgs函数自动包含了所有电机的CAN报文处理函数，只需要调用Motor_SendMsgs函数即可。
 * 
 * 1.大疆系电机，创建一个PID，将PID计算后的结果赋值给Out(单位是mA)，然后调用Motor_SendMsgs函数发送数据。
 * 2.VESC电机，创建一个VESC类，先选定模式(VESC_MODE)，然后将Out(注意！！！，由于每种模式的不同，Out的单位也不同)赋值后，
 *   调用Motor_SendMsgs函数发送数据。
 *   SET_eRPM: Out单位是erpm。erpm = rpm * pole_pairs
 *   SET_CURRENT: Out单位是mA
 *   SET_DUTY: Out单位是0.01%
 *   SET_POS: 单位应是度，这里虽然流出了接口。但是最好还是不要使用，VESC的位置控制不太稳定(这是由于电机本身的编码器精度而决定)。
 *            范围只有0-360度，而且不支持负值(在上位机是只能测得0~360°)。
 *   SET_BRAKE: Out单位是mA
 * 3.达妙电机，创建一个DM_Driver类，此处的接口不适用Out。改为Vel_Out和Pos_Out，分别代表速度和位置。然后调用Motor_SendMsgs函数发送数据。
 *   CMD_MOTOR_SPEED: 速度控制，需要设置Vel_Out
 *   CMD_MOTOR_POSITION: 位置控制, 需要设置Pos_Out和Vel_Out
 *   CMD_MOTOR_ENABLE: 使能， 使能后电机才能运行。在程序初始化就可以开始使能。建议在service_config.cpp中进行调用。
 *   CMD_MOTOR_DISABLE: 失能
 *   CMD_MOTOR_ZERO_SET: 零点标定
 * 
 * *************************************************************************************************************
 * @version 2.0 将原本两个电机驱动接口合并为一个，增加了达妙电机的驱动接口
 * @date create: 2024-05-16     update: 2024-7-19
 * 
 */
#pragma once

#include <stdint.h>
#include "../Components/drive_can.h"
#include "../Components/drive_can.h"
#include "data_pool.h"
#include "tool.h"

#ifdef __cplusplus

/* 
    VESC 的驱动模式
    
 */
typedef enum VESC_MODE
{
    SET_NULL,
    SET_eRPM,
    SET_CURRENT,
    SET_DUTY,
    SET_POS,
    SET_BRAKE,
}VESC_MODE;

/* 
    函数模板：电机输出限幅
    input:`T`：为电机输出的数据类型
    函数：
        input:T *val --- 为电机输出的数据
              T min --- 为电机输出的最小值
              T max --- 为电机输出的最大值
 */
template <typename T>
void motor_constraint(T *val, T min, T max)
{
    if(*val > max)
    {
        *val = max;
    }
    else if(*val < min)
    {
        *val = min;
    }
}

/* 
    电机基类
        ---编写电机驱动器的基类，所有电机驱动器都继承自此类
        ---电机基类中包含了电机的基本属性和方法，如电机id、电机输出、电机角度、电机编码器值等
        ---为什么不在Motor_Base中处理 电机速度 ，因为不是所有类型的电机都要用于速度处理
           而电机角度、编码器属性是必须要有的，所以将其放在基类中
        ---为什么要分电调和电机，因为电调和电机的数据解析和发送方式不同，以及温度、电流、扭矩等属性也不同
    编写特色：
        构造函数：将id通过初始化列表的方式传入
        析构函数：定义为虚函数，以便于子类进行析构（封装基类的重要思想，因为子类可能会进行资源分配，而这些操作最好是在其自身的析构中进行，便于管理）
        public、protected、private的使用
            public：
                成员函数：
                        纯虚函数：update --- 对于电调，肯定有不同的can地址，所以需要自己在子类中进行重写，根据不同的can地址进行数据的解析
                                CanMsg_Process---对于电调，肯定有不同的can地址，所以需要自己在子类中进行重写，根据不同的can地址进行数据的发送
                        虚函数：
                                check_id --- 检查can_id是否与电机id相等
                                get_angle --- 获取电机角度
                                get_encoder --- 获取电机编码器值
                                CanMsg_Process --- 电机can发送函数
                成员变量：
                        ID --- 电机id
                        Out --- 电机输出,对发送给电机的速度进行限幅

            protected：
                成员函数：
                        update_angle --- 更新电机角度
                        update_speed --- 更新电机速度
                        receive_id_init --- 获取电机can接收id
                        GET_MOTOR_DESCRIPTION --- 获取电机描述
                        ENCODER_MAX --- 获取电机编码器最大值
                        ENCODER_ANGLE_RATIO --- 获取电机编码器角度比
                        MAX_CURRENT --- 获取电机最大电流
                成员变量：
                        encoder --- 电机编码器值
                        angle --- 电机角度
                        last_encoder --- 上一次的编码器值
                        encoder_is_init --- 编码器是否初始化
                        encoder_offset --- 编码器偏移量
                        round_cnt --- 编码器圈数
            private：
                成员函数：
                        prepareCANMsg --- 准备CAN发送数据
                成员变量：
                        tarque --- 电机扭矩
                        temperature --- 电机温度

    派生类：
        Motor_Speed --- 电机速度类
        再派生类：
            RM_Common --- 大疆电机类
            再再派生类：
                Motor_C610 --- C610电调
                Motor_C620 --- C620电调
                Motor_GM6020 --- GM6020电调
                VESC --- VESC电调
            DM_Driver --- 达妙电机电调
 */
class Motor_Base
{
public:
    Motor_Base(uint8_t id) : ID(id){}
    virtual ~Motor_Base(){}
    const uint8_t ID = 0;

    virtual void update(uint8_t can_rx_data[])=0;
    virtual bool check_id(uint32_t StdID) const { return StdID == this->receive_id_init() + (uint8_t)ID; }
    float get_angle() const { return angle; }
	float get_encoder() const { return encoder; }
    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) = 0;
    float encoder_offset = 0;
	float Out = 0; /*!< Output ampere value that sent to motor */ // 其实就是pid控制器所得到的Output 值，可以用于直接发送给电机
protected:
    uint16_t encoder = 0; 
    float angle = 0,  last_encoder = 0;
    bool encoder_is_init = false;

    virtual uint32_t receive_id_init() const { return 0; };
    virtual uint8_t GET_MOTOR_DESCRIPTION() const { return 1; }
    virtual void update_angle(uint8_t can_rx_data[])
    {
        encoder = (uint16_t)(can_rx_data[0] << 8 | can_rx_data[1]);
        if(encoder_is_init)
        {   
            if(this->encoder - this->last_encoder < -4096)
                this->round_cnt++;
            else if(this->encoder - this->last_encoder > 4096)
                this->round_cnt--;
            else{}
        }
        else 
        {
            encoder_offset = encoder;
            encoder_is_init = true;
        }

        this->last_encoder = this->encoder;
        int32_t total_encoder = round_cnt*8192 + encoder - encoder_offset;
        angle = total_encoder / ENCODER_ANGLE_RATIO()/GET_MOTOR_DESCRIPTION();
    }

private:
    int32_t round_cnt = 0;
    virtual int16_t ENCODER_MAX() const { return 8192; }
    virtual float ENCODER_ANGLE_RATIO() const { return 8192.0f / 360.0f; }
    virtual float MAX_CURRENT() const { return 65535; }
};

/* 
    电机速度类
        ---继承自电机基类，实现电机速度的获取，同时提供了更多对速度的额外处理
    public：
        成员函数：
            get_speed --- 获取电机速度
            update --- 更新电机数据
            CanMsg_Process --- 电机can发送函数
        成员变量：
            speed --- 电机速度
    protected：
        成员函数：
            update_speed --- 更新电机速度
    派生类：
        RM_Common --- 大疆电机类
        再派生类：
            Motor_C610 --- C610电调
            Motor_C620 --- C620电调
            Motor_GM6020 --- GM6020电调
            VESC --- VESC电调
        DM_Driver --- 达妙电机电调
 */
class Motor_Speed : public Motor_Base
{
public:
    Motor_Speed(uint8_t id) : Motor_Base(id){}
    virtual ~Motor_Speed(){}
    virtual int32_t get_speed() const { return this->speed; }
    virtual void update(uint8_t can_rx_data[]) = 0;

protected:
    int16_t speed = 0;
    virtual void update_speed(uint8_t can_rx_data[])
    {
        speed = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    }
};


/* 
    RM_Common 大疆电机解析类
        ---继承自Motor_Speed类，实现大疆电机的数据解析
    protected：
        成员函数：
            设置为虚函数，因为其子类是针对不同的大疆电调进行的，所以需要在子类中进行重写
                receive_id_init --- 获取电机can接收id
                send_id_high --- 获取电机can发送高位id
                send_id_low --- 获取电机can发送低位id
                MAX_CURRENT --- 获取电机最大电流
            prepareCANMsg --- 准备CAN发送数据
 */
class RM_Common : public Motor_Speed
{
public:
    RM_Common(uint8_t id) : Motor_Speed(id){}
    
protected:
    virtual ~RM_Common(){}
    virtual uint32_t receive_id_init() const { return 0x200; }
    virtual uint32_t send_id_high() const { return 0x1FF; }
    virtual uint32_t send_id_low()  const { return 0x200; }
    virtual float MAX_CURRENT() const { return 10000; }

    void prepareCANMsg(CAN_TxMsg &CAN_TxMsg, int16_t current_out) const {

        if (ID <= 4 && ID > 0) {
            CAN_TxMsg.data[ID * 2 - 2] = (uint8_t)(current_out >> 8) & 0xff;
            CAN_TxMsg.data[ID * 2 - 1] = (uint8_t)current_out & 0xff;
            CAN_TxMsg.id = send_id_low();
            CAN_TxMsg.len = 8;
        } else if (ID <= 8 && ID > 4) {
            CAN_TxMsg.data[ID * 2 - 10] = (uint8_t)(current_out >> 8) & 0xff;
            CAN_TxMsg.data[ID * 2 - 9] = (uint8_t)current_out & 0xff;
            CAN_TxMsg.id = send_id_high();
            CAN_TxMsg.len = 8;
        }
    }
};

/* 
    Motor_C610 C610电调类
        ---继承自RM_Common类，实现C610电调的数据解析
    public：
        成员函数：
            get_tarque --- 获取电机扭矩
            get_temperature --- 获取电机温度
            update --- 更新电机数据
            CanMsg_Process --- 电机can发送函数
    private：
        成员函数：
            GET_MOTOR_DESCRIPTION --- 获取电机描述
    成员变量：
        tarque --- 电机扭矩
        temperature --- 电机温度
 
 */
class Motor_C610 : public RM_Common
{
public:
    virtual ~Motor_C610(){}
    virtual float MAX_CURRENT() const { return 10000; }
    Motor_C610(uint8_t id) : RM_Common(id){}
    virtual void update(uint8_t can_rx_data[]) override
    {
        update_angle(can_rx_data);
        update_speed(can_rx_data);
        this->tarque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);    
    }
    int16_t get_tarque() const { return this->tarque; }
    uint8_t get_temperature() const { return this->temperature; }

    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) override
    {
        motor_constraint(&(this->Out), -MAX_CURRENT(), MAX_CURRENT());
        prepareCANMsg(CAN_TxMsg, (int16_t)(this->Out));
    }
    
protected:
    virtual uint8_t GET_MOTOR_DESCRIPTION() const { return 36; }

private:
    int16_t tarque = 0;
    uint8_t temperature = 0;
};

/* 
    Motor_C620 C620电调类
        ---继承自RM_Common类，实现C620电调的数据解析
    public：
        成员函数：
            get_tarque --- 获取电机扭矩
            get_temperature --- 获取电机温度
            update --- 更新电机数据
            CanMsg_Process --- 电机can发送函数
    private：
        成员函数：
            GET_MOTOR_DESCRIPTION --- 获取电机描述
    成员变量：
        tarque --- 电机扭矩
        temperature --- 电机温度
 
 */
class Motor_C620 : public RM_Common
{
public:
    virtual float MAX_CURRENT() const { return 16384; }
    Motor_C620(uint8_t id) : RM_Common(id){}
    int16_t get_tarque() const { return this->tarque; }
    uint8_t get_temperature() const { return this->temperature; }

    virtual void update(uint8_t can_rx_data[])
    {
        update_angle(can_rx_data);
        update_speed(can_rx_data);
        this->tarque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);    
    }

    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) override
    {
        motor_constraint(&(this->Out), -MAX_CURRENT(), MAX_CURRENT());
        prepareCANMsg(CAN_TxMsg, (int16_t)(this->Out));
    }
protected:
    virtual uint8_t GET_MOTOR_DESCRIPTION() const { return 19; }

private:
    int16_t tarque = 0;
    uint8_t temperature = 0;
};

/* 
    Motor_GM6020 GM6020电调类
        ---继承自RM_Common类，实现GM6020电调的数据解析
    public：
        成员函数：
            get_tarque --- 获取电机扭矩
            get_temperature --- 获取电机温度
            update --- 更新电机数据
            CanMsg_Process --- 电机can发送函数
    成员变量：
        tarque --- 电机扭矩
        temperature --- 电机温度
 */
class Motor_GM6020 : public RM_Common
{
public:
    virtual ~Motor_GM6020(){}
    virtual uint32_t recieve_id_init() const { return 0x204; }
    virtual uint32_t send_id_low() const { return 0x1ff; }
    virtual uint32_t send_id_high() const { return 0x2ff; }
    virtual float MAX_CURRENT() const { return 30000; }
    uint16_t set_encoder_offset(uint16_t offset)
    {
        this->encoder_offset = offset;
        this->last_encoder = offset;  
        this->encoder_is_init = true;
        return this->encoder;
    }
    Motor_GM6020(uint8_t id) : RM_Common(id){}
    virtual void update(uint8_t can_rx_data[])
    {
        this->update_angle(can_rx_data);
        this->update_speed(can_rx_data);
        this->tarque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
        this->temperature = can_rx_data[6]; 
    }

    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) override
    {
        motor_constraint(&(this->Out), -MAX_CURRENT(), MAX_CURRENT());
        prepareCANMsg(CAN_TxMsg, (int16_t)(this->Out));
    }

    int16_t get_tarque() const { return this->tarque; }
    uint8_t get_temperature() const { return this->temperature; }
private:
    int16_t tarque = 0;
    uint8_t temperature = 0;
};


/* 
    VESC VESC电调类
        ---继承自Motor_Speed类，实现VESC电调的数据解析

 */
class VESC : public Motor_Speed, public Tools
{
public:
    VESC(uint8_t id) : Motor_Speed(id){} //括号中为VESC的CAN ID
    virtual ~VESC(){}
    VESC_MODE Mode;
    void update_vesc(CAN_RxBuffer* Buffer)
    { 
        ID_check = Buffer->header.ExtId & 0xff;
        if( ID_check == this->ID)
        {
            cmd = (Buffer->header.ExtId >> 8);   //获取对应的帧头
            update(Buffer->data);
        }
    }
    virtual int32_t get_speed() const { return this->speed; }
    int16_t get_tarque() const { return this->tarque; }

    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) override
    {
        CAN_TxMsg.len = 8;
        switch (this->Mode)
        {
            case SET_eRPM:    //erpm = rpm * pole_pairs
            {
                int index=0;
                CAN_TxMsg.id = this->ID | ((uint32_t)CAN_PACKET_SET_RPM << 8);
                _tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)this->Out, &index);
                break;
            }

            case SET_CURRENT:
            {
                int index=0;
                CAN_TxMsg.id = this->ID | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);   //mA
                _tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)this->Out, &index);
                break;
            }

            case SET_DUTY:      //0.01%
            {
                int index=0;
                CAN_TxMsg.id = this->ID | ((uint32_t)CAN_PACKET_SET_DUTY << 8);
                _tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)(this->Out * 100000), &index);
                break;
            }

            case SET_POS:   
            {
                int index=0;
                CAN_TxMsg.id = this->ID | ((uint32_t)CAN_PACKET_SET_POS << 8);
                _tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)this->Out*1000000, &index);
                break;
            }

            case SET_BRAKE:
            {
                int index=0;
                CAN_TxMsg.id = this->ID | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
                _tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)this->Out, &index);
                break;
            }

            default: break;
        }
    }
protected:
    virtual void update(uint8_t can_rx_data[])
    {
        switch(cmd)
        {
            case CAN_PACKET_STATUS:
                update_speed(can_rx_data);
                break;
            case CAN_PACKET_STATUS_4:
                update_angle(can_rx_data);
                break;
            default:
                break;
        }
    }
 
    virtual void update_speed(uint8_t can_rx_data[])
    {
        index=0;
        this->speed = _tool_buffer_get_int32(can_rx_data, &index);
        this->tarque = (float)_tool_buffer_get_int16(can_rx_data, &index)*100.0f;   //mA
        this->duty = (float)_tool_buffer_get_int16(can_rx_data, &index)/1000.0f;
    }

    virtual void update_angle(uint8_t can_rx_data[])
    {
        angle = (float)(can_rx_data[6] << 8 | can_rx_data[7])/50.0f;
    }

private:
    uint8_t ID_check;
    uint16_t cmd;
    int index=0;
    float tarque=0;
    float duty=0;
};

/* 
    达妙电机电调类
        ---继承自Motor_Speed类，实现达妙电机电调的数据解析
 */
class DM_Driver : public Motor_Speed, public Tools
{
public:
    DM_Driver(uint8_t id) : Motor_Speed(id){} //括号中为VESC的CAN ID
    virtual ~DM_Driver(){}
    virtual void update(uint8_t can_rx_data[]) override
    {
        if(can_rx_data[0] == this->ID)
        {
            p_int=(can_rx_data[1]<<8)|can_rx_data[2];//电机位置数据
			v_int = (can_rx_data[3]<<4)|(can_rx_data[4]>>4);//电机速度数据
			i_int = ((can_rx_data[4]&0xF)<<8)|can_rx_data[5];//电机扭矩数据

            angle = DM_uint_to_float(p_int, MIT_P_MIN, MIT_P_MAX, 16);// 转子机械角度
            speed = DM_uint_to_float(v_int, MIT_V_MIN, MIT_V_MAX, 12);// 实际转子转速
            tarque = DM_uint_to_float(i_int, I_MIN, I_MAX, 12);		 // 实际转矩电流
        }
    }

    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) override
    {
        switch (this->Motor_Status)
        {
            case CMD_MOTOR_ENABLE:
            {
                CAN_TxMsg.id = 0x100+this->ID;
                CAN_TxMsg.len = 8;
                for(int i=0; i<7; i++)
                    CAN_TxMsg.data[i] = 0xFF;
                CAN_TxMsg.data[7] = (uint8_t)CMD_MOTOR_ENABLE;
                break;
            }
            
            case CMD_MOTOR_DISABLE:
            {
                CAN_TxMsg.id = 0x100+this->ID;
                CAN_TxMsg.len = 8;
                for(int i=0; i<7; i++)
                    CAN_TxMsg.data[i] = 0xFF;
                CAN_TxMsg.data[7] = (uint8_t)CMD_MOTOR_DISABLE;
                break;
            }

            case CMD_MOTOR_ZERO_SET:
            {
                CAN_TxMsg.id = 0x100+this->ID;
                CAN_TxMsg.len = 8;
                for(int i=0; i<7; i++)
                    CAN_TxMsg.data[i] = 0xFF;
                CAN_TxMsg.data[7] = (uint8_t)CMD_MOTOR_ZERO_SET;
                break;
            }

            case CMD_MOTOR_SPEED:
            {
                uint8_t *vbuf;
                vbuf=(uint8_t*)&this->Vel_Out;

                CAN_TxMsg.id = 0x200+(uint8_t)this->ID;
                CAN_TxMsg.len = 4;

                for(int i=0; i<4; i++)
                    CAN_TxMsg.data[i] = *(vbuf+i);
                break;
            }

            case CMD_MOTOR_POSITION:
            {
                uint8_t *vbuf, *pbuf;
                vbuf=(uint8_t*)&this->Vel_Out;
                pbuf=(uint8_t*)&this->Pos_Out; 

                CAN_TxMsg.id = 0x100+(uint8_t)this->ID;
                CAN_TxMsg.len = 8;

                for(int i=0; i<4; i++)
                    CAN_TxMsg.data[i] = *(pbuf+i);
                for(int i=4; i<8; i++)
                    CAN_TxMsg.data[i] = *(vbuf+i-4);
                break;
            }
            default:
                break;
        }
    }

    virtual int32_t get_speed() const { return this->speed; }
    int16_t get_tarque() const { return this->tarque; }
    virtual float get_angle() const { return this->angle;}
    DM_MOTORCMD Motor_Status;
    float Vel_Out=0, Pos_Out=0;
protected:
    
private:
    int p_int, v_int, i_int;
    int16_t tarque = 0;
    const float MIT_P_MIN = -12.5f, MIT_P_MAX = 12.5f;
    const float MIT_V_MIN = -500.0f, MIT_V_MAX = 500.0f;
    const float I_MAX = 18.0f, I_MIN = -18.0f;
};

/* 
    函数模板：电机can发送函数，用于多个电机的发送（存放多电机对象的数组）
    input:`Motor_Type`：为电机类
          'N'：为电机数量
    函数:
        input:CAN_HandleTypeDef *hcan --- 为CAN句柄
              Motor_Type (&motor)[N] --- 为电机数组
    使用Freertos中消息队列的通讯形式进行CAN发送
 */
template <class Motor_Type, int N>
void Motor_SendMsgs(CAN_HandleTypeDef *hcan, Motor_Type (&motor)[N])
{
    CAN_TxMsg CAN_TxMsg;

    for(int i=0; i<N; i++)
    {
        motor[i].CanMsg_Process(CAN_TxMsg);
    }

    if(hcan == &hcan1)
        xQueueSend(CAN1_TxPort, &CAN_TxMsg, portMAX_DELAY);
    else if(hcan == &hcan2)
        xQueueSend(CAN2_TxPort, &CAN_TxMsg, portMAX_DELAY);
}

/* 
    函数模板：电机can发送函数，用于单个电机的发送（单电机对象）
    使用Freertos中消息队列的通讯形式进行CAN发送
    input:`Motor_Type`：为电机类
    函数：
        input:CAN_HandleTypeDef *hcan --- 为CAN句柄
              Motor_Type &motor --- 为电机
 */
template <class Motor_Type>
void Motor_SendMsgs(CAN_HandleTypeDef *hcan, Motor_Type &motor)
{
    Motor_Type motor_arr[1] = {motor};
    Motor_SendMsgs(hcan, motor_arr);
}


#endif 
