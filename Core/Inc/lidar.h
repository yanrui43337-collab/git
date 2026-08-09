#ifndef __LIDAR_H
#define __LIDAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"    // 包含 HAL 库相关定义
#include <stdint.h>

/* --- 全局变量声明 --- */

/**
 * @brief 存放 360 度距离值的数组
 * 索引 0~359 对应角度，值为距离 (mm)。
 */
extern uint16_t Lidar_Distance_Array[360];
extern volatile uint32_t Lidar_ScanSequence;
extern volatile uint32_t Lidar_LastUpdateTick;

/**
 * @brief Copy the latest fully completed 360-degree scan.
 * @param destination Caller-owned 360-element distance buffer.
 * @param sequence Optional completed-scan sequence output.
 * @return 1 when a complete scan is available, otherwise 0.
 */
uint8_t Lidar_CopyCompletedScan(uint16_t destination[360], uint32_t *sequence);

/**
 * @brief 避障常用的方向最小距离变量
 */
extern int left_min; 
extern int right_min;
extern int front_min;
extern int rear_min; 


/* --- 函数原型声明 --- */

/**
 * @brief 雷达驱动初始化
 * 开启串口空闲中断并启动 DMA 循环接收。
 */
 
 
void Lidar_Init(void);


void Send_Lidar_Data_To_VOFA(void);


void Compress_And_Send_Lidar_Data(void);


/**
 * @brief 获取指定角度范围内的最小有效距离
 * @param start_angle 起始角度 (0~359)
 * @param end_angle   结束角度 (0~359)
 * @return 范围内最小的有效距离 (mm)，若无有效点则返回 0。
 */
uint16_t Lidar_Get_Min_Distance_In_Range(uint16_t start_angle, uint16_t end_angle);


#ifdef __cplusplus
}
#endif

#endif /* __LIDAR_H */
