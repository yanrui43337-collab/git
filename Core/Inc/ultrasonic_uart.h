#ifndef __ULTRASONIC_UART_H
#define __ULTRASONIC_UART_H

#include "main.h"

// 对外暴露的初始化和获取距离函数
void Ultrasonic_UART_Init(void);
int Ultrasonic_Get_Distance(void);

// 供串口回调中断内部使用的解析函数
void Ultrasonic_Parse_Frame(uint8_t *buf, uint16_t size);

#endif
