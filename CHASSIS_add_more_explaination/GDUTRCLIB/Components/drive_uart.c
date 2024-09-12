/**
 * @file drive_uart.c
 * @author YangJianyi
 * @brief 1)串口底层驱动文件，使用该文件，需要在cubeMX中配置好串口硬件(参考大疆电机所需的串口的配置)，并在main.c中调用Uart_Init函数进行初始化
 *        2)默认使用串口DMA接收，当接收到数据时，会调用Uart_Rx_Idle_Callback函数。
 *        
 *        3)学院的串口协议默认使用以下格式（示例可查阅ROS.cpp文件）：
 *          前两位:包头: 0x55 0xAA     
 *          第三位:数据长度: 1字节(总的buf长度-6)
 *          倒数第四位:crc8校验位: 1字节
 *          末尾两位:包尾: 0x0D 0x0A
 *          
 *************************************************************************************************************************
 * 注意!!!!!!!!!!!
 *          使用该文件需要在stm32f4xx_it.c中的串口中断服务函数中添加中断接收函数，例如:Uart_Receive_Handler(&usart1_manager);
 *          该文件中包含了串口1、串口2、串口3、串口6的回调函数，如有需要，可自行添加其他串口
 * *************************************************************************************************************************
 * @version 0.1
 * @date 2024-04-03
 * 
 */

#include "drive_uart.h"

/* 
    创建串口管理结构体，用于存储串口的相关信息
        顺便初始化了结构体成员，将串口回调函数指针初始化为NULL
 */
usart_manager_t usart1_manager = {.call_back_fun = NULL};
usart_manager_t usart2_manager = {.call_back_fun = NULL};
usart_manager_t usart3_manager = {.call_back_fun = NULL};
usart_manager_t usart6_manager = {.call_back_fun = NULL}; 


/* 
    知识点：static 静态函数使用
    作用：
        封装性：static 函数只能在定义它们的文件中使用，防止其他文件意外调用这些函数，从而提高了代码的封装性和模块化。
        命名冲突：由于 static 函数的作用域仅限于文件内部，可以避免与其他文件中同名函数的命名冲突。
    使用：
        在源文件中声明 static 函数是为了在该文件中使用这些函数，同时限制它们的作用域，防止其他文件访问。
 */
static void Uart_Rx_Idle_Callback(usart_manager_t *manager);
static void DMA_Uart_Rx_HalfTransfer_Callback(usart_manager_t *manager);
static void DMA_Uart_Rx_FullTransfer_Callback(usart_manager_t *manager);


/*
    串口初始化函数封装：
        开启串口只需要在这个函数中进行好串口的初始化即可
    input：
        huart：串口句柄
        Rxbuffer：接收缓存
        len：缓存长度
        call_back_fun：回调函数

    串口3机制（ROS和32通信）：
        DMA 接收满一定数据触发DMA中断
        DMA中断触发空闲中断
        空闲中断触发回调函数
        在回调函数中处理数据
 */
void Uart_Init(UART_HandleTypeDef *huart, uint8_t *Rxbuffer, uint16_t len, usart_call_back call_back_fun)
{
    // 防御式编程
    if(huart == NULL)
        Error_Handler();
    else{}

    if(huart->Instance == USART1)
    {
        usart1_manager.uart_handle = huart;
        usart1_manager.rx_buffer = Rxbuffer;
        usart1_manager.rx_buffer_size = len;
        usart1_manager.call_back_fun = call_back_fun;
        __HAL_UART_CLEAR_IDLEFLAG(huart);// 清除UART的空闲中断标志位
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);// 使能UART的空闲中断
		HAL_UART_Receive_DMA(huart, Rxbuffer, len);// 开启DMA接收（在这一步里面会自动使能 DMA 接收完成中断）
    }
    else if(huart->Instance == USART2)
    {
        usart2_manager.uart_handle = huart;
        usart2_manager.rx_buffer = Rxbuffer;
        usart2_manager.rx_buffer_size = len;
        usart2_manager.call_back_fun = call_back_fun;
        __HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		HAL_UART_Receive_DMA(huart, Rxbuffer, len);
    }
    else if(huart->Instance == USART3)
    {
        usart3_manager.uart_handle = huart;
        usart3_manager.rx_buffer = Rxbuffer;
        usart3_manager.rx_buffer_size = len;
        usart3_manager.call_back_fun = call_back_fun;
        // 先试一下不用 串口空闲中断
        //__HAL_UART_CLEAR_IDLEFLAG(huart);
		//__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

        // 初始化先清除一下 dma 的中断标志位 
        __HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_IT_TC);
        __HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_IT_HT);
        // 使能 DMA 半满中断
        __HAL_DMA_ENABLE_IT(huart->hdmarx,DMA_IT_HT);
		HAL_UART_Receive_DMA(huart, Rxbuffer, len);// 其实也使能了接收完成中断，所以不重复操作
    }
    else if(huart->Instance == USART6)
    {
        usart6_manager.uart_handle = huart;
        usart6_manager.rx_buffer = Rxbuffer;
        usart6_manager.rx_buffer_size = len;
        usart6_manager.call_back_fun = call_back_fun;
        __HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		HAL_UART_Receive_DMA(huart, Rxbuffer, len);
    }
    else
    {
        Error_Handler();
    }
}


/**
 * @brief   Registered user callback function 注册 串口回调函数
 * @param   manager: serial port handle
 * @param   fun: user callback function
 * @retval  None
 */
void Usart_Rx_Callback_Register(usart_manager_t *manager, usart_call_back fun)
{
  /* Check the parameters */
	assert_param(fun != NULL);
	assert_param(manager != NULL);
	
	manager->call_back_fun = fun;
	return;
}


/**
 * @brief   Determine if the idle interrupt is triggered and process the received data
 * @param   manager: serial port handle
 * @retval  None
 */
void Uart_Receive_Handler(usart_manager_t *manager)
{
	if(__HAL_UART_GET_FLAG(manager->uart_handle,UART_FLAG_IDLE)!=RESET)// 检查UART的空闲中断标志位是否置位
	{
		Uart_Rx_Idle_Callback(manager);// 调用串口空闲中断回调函数
	}
}

void DMA_Receive_Handler(usart_manager_t *manager)
{
    if(manager->uart_handle->Instance == USART3)
    {
        /* 检查 dma 半满中断标志位是否被置位 */
        if(__HAL_DMA_GET_FLAG(manager->uart_handle->hdmarx,DMA_FLAG_HTIF1_5)!=RESET)
        {
            DMA_Uart_Rx_HalfTransfer_Callback(manager);// 调用串口DMA接收半满中断回调函数
        }
        /* 检查 dma 接收完成中断标志位是否被置位 */
        if(__HAL_DMA_GET_FLAG(manager->uart_handle->hdmarx,DMA_FLAG_TCIF1_5)!=RESET)
        {
            DMA_Uart_Rx_FullTransfer_Callback(manager);// 调用串口DMA接收完全中断回调函数
        }
    }
}

/** 串口空闲中断回调函数
 * @brief   clear idle it flag after uart receive a frame data
 * @note    call in uart_receive_handler() function
 * @param   uart IRQHandler id
 * @retval  None
 */
static void Uart_Rx_Idle_Callback(usart_manager_t *manager)
{
    /* Check the parameters */
	assert_param(manager != NULL);
	
    /* Private variables */
	static uint16_t usart_rx_num;

    /* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(manager->uart_handle);// 清除UART的空闲中断标志位

    /* clear DMA transfer complete flag */
	HAL_UART_DMAStop(manager->uart_handle);// 回调函数过程中，停止DMA传输

    /* handle received data in idle interrupt */
    // 通过操作 DMA 的 NDTR 寄存器来获取接收到的数据长度
	usart_rx_num = manager->rx_buffer_size - ((DMA_Stream_TypeDef*)manager->uart_handle->hdmarx->Instance)->NDTR;

	if(manager->call_back_fun != NULL)// 经典判空操作
		manager->call_back_fun(manager->rx_buffer, usart_rx_num);// 调用数据处理回调函数
	
    // 重新开启 dma 传输
	HAL_UART_Receive_DMA(manager->uart_handle, manager->rx_buffer, 2*(manager->rx_buffer_size));
}

/* 
    串口接收半满中断回调函数
 */
static void DMA_Uart_Rx_HalfTransfer_Callback(usart_manager_t *manager)
{
    /* 检查输入参数，防御式编程 */
    assert_param(manager != NULL);
    if(manager->uart_handle->Instance == USART3)
    {
        static uint16_t usart_rx_num;

        /* 清除 dma 接收半满中断标志位 */
        __HAL_DMA_CLEAR_FLAG(manager->uart_handle, DMA_FLAG_HTIF1_5);
        // 通过操作 DMA 的 NDTR 寄存器来获取接收到的数据长度
        usart_rx_num = manager->rx_buffer_size - ((DMA_Stream_TypeDef*)manager->uart_handle->hdmarx->Instance)->NDTR;
        
        if(manager->call_back_fun != NULL)
            manager->call_back_fun(manager->rx_buffer,usart_rx_num);// 调用数据处理回调函数

        manager->last_rx_len = usart_rx_num;// 更新上一次接收的数据长度
    }
}
/* 
    串口接收完全中断回调函数
 */
static void DMA_Uart_Rx_FullTransfer_Callback(usart_manager_t *manager)
{
    /* 检查输入参数，防御式编程 */
    assert_param(manager != NULL);
    if(manager->uart_handle->Instance == USART3)
    {
        static uint16_t usart_rx_num;
        /* 清除 dma 接收完成中断 标志位 */
        __HAL_DMA_CLEAR_FLAG(manager->uart_handle, DMA_FLAG_TCIF1_5);
        /* 重新开启 dma 接收 */
        HAL_UART_Receive_DMA(manager->uart_handle, manager->rx_buffer, 2*(manager->rx_buffer_size));
        // 通过操作 DMA 的 NDTR 寄存器来获取接收到的数据长度
        usart_rx_num = manager->rx_buffer_size - manager->last_rx_len;// 这次的数据长度就是 全部的 - 上一次的

        if(manager->call_back_fun != NULL)
            manager->call_back_fun(manager->rx_buffer+manager->last_rx_len,usart_rx_num);// 调用数据处理回调函数
        
        manager->last_rx_len = 0;// 更新上一次接收的数据长度
    }
}

/**
 * @brief 校验位函数Crc8
 * @param 传入数组
 * @param 当前数组长度
 * @return 检验值
*/
unsigned char serial_get_crc8_value(unsigned char *tem_array, unsigned char len)
{
    unsigned char crc = 0;
    unsigned char i;
    while(len--)
    {
        crc ^= *tem_array++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}

