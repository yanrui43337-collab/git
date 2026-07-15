#ifndef __MY_IMU_H
#define __MY_IMU_H

#include <stdint.h>
#include "main.h"

// 环形缓冲区大小定义
#define IMU_UART_RX_BUF_SIZE 256

// 新协议包头定义 (根据协议手册 0x7E 0x23)
#define FRAME_HEAD1 0x7E
#define FRAME_HEAD2 0x23

// 欧拉角数据包功能码定义 (根据协议手册 0x26)
#define IMU_FUNC_EULER 0x26

// 外部接口声明
void IMU_UART_RxByte_Callback(uint8_t data);
void IMU_UART_RxBytes(uint8_t *data, uint16_t len);
void IMU_UART_Process(void);
int IMU_UART_GetEuler(float out[3]);
float IMU_Get_Yaw(void);

#endif
