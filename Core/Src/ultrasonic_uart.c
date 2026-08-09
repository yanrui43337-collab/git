#include "ultrasonic_uart.h"
#include "debug_log.h"
#include "usart.h" // 包含 huart5 的定义
#include <stdio.h> // 用于 debug 打印

// 存储最新测得的距离 (单位: mm)，默认 -1 代表还未获取到数据
static volatile int ultrasonic_distance = -1; 

// 串口接收缓冲区
uint8_t ultra_rx_buf[16]; 

HAL_StatusTypeDef Ultrasonic_UART_Restart(void)
{
    HAL_StatusTypeDef status;

    HAL_NVIC_DisableIRQ(UART5_IRQn);
    (void)HAL_UART_AbortReceive(&huart5);

    __HAL_UART_CLEAR_OREFLAG(&huart5);
    __HAL_UART_CLEAR_FEFLAG(&huart5);
    __HAL_UART_CLEAR_NEFLAG(&huart5);
    __HAL_UART_CLEAR_PEFLAG(&huart5);
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    __HAL_UART_SEND_REQ(&huart5, UART_RXDATA_FLUSH_REQUEST);
    HAL_NVIC_ClearPendingIRQ(UART5_IRQn);

    status = HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, sizeof(ultra_rx_buf));
    HAL_NVIC_EnableIRQ(UART5_IRQn);

    return status;
}

// 初始化函数：开启 UART5 的空闲中断接收
void Ultrasonic_UART_Init(void) 
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t attempt;

    for (attempt = 0U; attempt < 3U; attempt++)
    {
        status = Ultrasonic_UART_Restart();
        if (status == HAL_OK)
        {
            break;
        }
        HAL_Delay(20U);
    }

    if (status == HAL_OK)
    {
        printf("✅ 超声波串口 IT 监听开启成功！\r\n");
    }
    else
    {
        printf("🚨 超声波串口监听启动失败，错误码: %d\r\n", status);
    }
}

// 获取距离接口 (供主循环调用)
int Ultrasonic_Get_Distance(void) 
{
    return ultrasonic_distance;
}

// 核心解析函数 (适配 24 位高精度数据公式)
// 核心解析函数 (智能倒序拾取数据版)
void Ultrasonic_Parse_Frame(uint8_t *buf, uint16_t size) 
{
    // 只要收到至少 3 个字节，就说明里面肯定包含距离数据
    if (size >= 3) 
    {
        // 🌟 神级操作：不管总长是多少，永远只拿最后 3 个字节！
        uint8_t b1 = buf[size - 3]; // 高 8 位
        uint8_t b2 = buf[size - 2]; // 中 8 位
        uint8_t b3 = buf[size - 1]; // 低 8 位

        // 套用客服公式
        uint32_t raw_data = ((uint32_t)b1 << 16) + ((uint32_t)b2 << 8) + b3;
        int temp_dist = raw_data / 1000;
        
        // 简单防错：限制在合理量程内 (比如 10米 以内)
        if (temp_dist >= 0 && temp_dist < 10000) 
        {
            ultrasonic_distance = temp_dist;
        }
    }
}
