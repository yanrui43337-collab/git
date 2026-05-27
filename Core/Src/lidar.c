/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lidar.c
  * @brief   This file provides code for the configuration
  *          of the lidar	instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "usart.h"     // 包含你的串口头文件，确保有 huart2 的定义
#include "string.h"
#include "math.h"
#include <stdint.h>
#include <stdio.h>

// --- 宏定义区 ---
#define LIDAR_FRAME_LEN     58          // 协议中一帧数据的固定长度
#define RX_BUF_SIZE         (64 * 10)   // 接收缓冲区大小，设为帧长的整数倍，防止数据溢出

// --- 全局变量区 ---
/* * ?? H7 避坑指南：必须保证 DMA 使用的内存在 32 字节对齐，
 * 或者直接将其放在不经过 D-Cache 的 RAM 区域 (如 D2 SRAM)。
 * 这里我们用 ALIGN_BEGIN 强制对齐，并在代码中手动 Invalidate Cache。
 */
// 给数组加一个绝对定位，放到 SRAM4 或 D2 域（通常 0x30000000 开头）
uint8_t Lidar_RxBuf[640] __attribute__((section(".RAM_D2"))) __attribute__((aligned(32)));
uint16_t Lidar_WriteIndex = 0; // DMA 写入位置
uint16_t Lidar_ReadIndex = 0;  // 我们解析的读取位置

uint16_t Lidar_Distance_Array[360] = {0};

int left_min =0;   //左侧最小距离
int right_min = 0; //右侧最小距离
int front_min =0;  //前方最小距离
int rear_min =0;   //后方最小距离
extern UART_HandleTypeDef huart4; // 告诉编译器 huart4 在其他地方定义了（通常在 usart.c）

// --- 1. 雷达初始化函数 (在 main.c 的 while(1) 之前调用) ---
void Lidar_Init(void)
{
    HAL_StatusTypeDef status;
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Lidar_RxBuf, 640);
    
    if(status != HAL_OK)
    {
        // 如果这里打印了，说明启动就失败了
        printf("Lidar DMA Start Failed! Code: %d\n", status);
        // 如果 Code 是 2 (HAL_BUSY)，说明串口被占用了
        // 如果 Code 是 1 (HAL_ERROR)，通常是配置参数不匹配
    }
    else
    {
        printf("Lidar DMA Start OK!\n");
    }
}


// --- 2. 核心解析函数：提取 360 度距离数据 ---
 void Lidar_ParseSingleFrame(uint8_t *frame)
{
    // 1. 校验 CRC (Byte_0 到 Byte_56 的累加和)
    uint8_t sum = 0;
    for(int i = 0; i < 57; i++) 
	  {
        sum += frame[i];
    }
    if(sum != frame[57]) 
		{
				//printf("CRC Error! Sum:%02X, Frame:%02X\n", sum, frame[57]);
        return; // 校验失败，直接丢弃这帧脏数据
    }

    // 2. 解算起始角度和结束角度 (Byte 5-6, Byte 55-56，高位在前)
    // 除以 100.0f 是因为协议规定数值是实际角度的 100 倍
    float start_angle = ((frame[5] << 8) | frame[6]) / 100.0f;
    float stop_angle = ((frame[55] << 8) | frame[56]) / 100.0f;

    // 3. 计算这一帧的角度跨度 (必须处理跨越 0 度极点的情况)
    float diff_angle = stop_angle - start_angle;
    if (diff_angle < 0) 
		{
        diff_angle += 360.0f; // 例如 start=358, stop=3, diff = 3 - 358 + 360 = 5度
    }

    // 4. 计算每个点之间的角度步长 (16个点，15个间隔)
    float angle_step = diff_angle / 15.0f;
        
    // 5. 遍历这一帧的 16 个数据点
    for (int i = 0; i < 16; i++)
    {
        // 计算当前点的精确角度
        float current_angle = start_angle + angle_step * i;
        
        // 如果角度超过 360 度，进行归一化 (比如 361 度变成 1 度)
        if (current_angle >= 360.0f) {
            current_angle -= 360.0f;
        }

        // 将浮点角度四舍五入，并转为 0~359 的整数数组索引
        // 加 0.5f 是为了 C 语言强制类型转换时的四舍五入
        int index = (int)(current_angle + 0.5f) % 360; 

        // 6. 提取距离数据 (Byte_7 到 Byte_54)
        // 根据协议：第 i 个点的距离高位 = 7 + i*3，距离低位 = 8 + i*3
        uint8_t dist_H = frame[7 + i * 3];
        uint8_t dist_L = frame[8 + i * 3];
        uint16_t distance = (dist_H << 8) | dist_L;
        
        // 7. 更新数组 (过滤掉距离为 0 的无效噪点)
        if (distance > 0) {
            Lidar_Distance_Array[index] = distance;
        }
    }
}


// 假设这是你解析雷达数据后存放 360 度距离值的数组
// 索引 0~359 对应角度，值为距离 (mm)。如果某角度没有有效数据，值为 0
extern uint16_t Lidar_Distance_Array[360]; 

#define MAX_VALID_DISTANCE 0xFFFF // 定义一个足够大的初始值

/**
 * @brief  获取指定角度范围内的最小有效距离
 * @param  start_angle: 起始角度 (0~359)
 * @param  end_angle:   结束角度 (0~359)
 * @retval 范围内的最小距离 (mm)。如果范围内全为无效点，则返回 0
 */
uint16_t Lidar_Get_Min_Distance_In_Range(uint16_t start_angle, uint16_t end_angle)
{
    uint16_t min_dist = MAX_VALID_DISTANCE;
    
    // 安全容错：限制在 0-359 范围内
    start_angle %= 360;
    end_angle %= 360;

    // 情况 1：普通扇区，没有跨越 0 度极点 (例如：左侧 30 度到 90 度)
    if (start_angle <= end_angle) 
    {
        for (uint16_t i = start_angle; i <= end_angle; i++) 
        {
            // 过滤掉距离为 0 的无效噪点
            if (Lidar_Distance_Array[i] > 0 && Lidar_Distance_Array[i] < min_dist) 
            {
                min_dist = Lidar_Distance_Array[i];
            }
        }
    }
    // 情况 2：跨越了 0 度极点 (例如：正前方从 330 度到 30 度)
    else 
    {
        // 先遍历 start_angle 到 359 度
        for (uint16_t i = start_angle; i < 360; i++) 
        {
            if (Lidar_Distance_Array[i] > 0 && Lidar_Distance_Array[i] < min_dist) 
            {
                min_dist = Lidar_Distance_Array[i];
            }
        }
        // 再遍历 0 度 到 end_angle
        for (uint16_t i = 0; i <= end_angle; i++) 
        {
            if (Lidar_Distance_Array[i] > 0 && Lidar_Distance_Array[i] < min_dist) 
            {
                min_dist = Lidar_Distance_Array[i];
            }
        }
    }

    // 如果遍历完发现 min_dist 还是初始值，说明这个区域里全是无效点（比如超出量程）
    if (min_dist == MAX_VALID_DISTANCE) 
    {
        return 0; 
    }
    
    return min_dist;
}


// 假设此函数在你每次采集完一圈雷达数据后调用
void Send_Lidar_Data_To_VOFA(void) 
{
    for (int i = 0; i < 360; i++) {
        // 忽略无效或距离为0的点（可选，根据你的传感器情况）
        
			  if (Lidar_Distance_Array[i] == 0) continue; 
        
        // 1. 将角度转换为弧度
        float radian = i * 3.1415926f / 180.0f;
        
        // 2. 极坐标转直角坐标 (x, y)
        float x = Lidar_Distance_Array[i] * cos(radian);
        float y = Lidar_Distance_Array[i] * sin(radian);
        
        // 3. 按照 FireWater 格式输出: ch0(X), ch1(Y) + 换行符
        printf("%.2f,%.2f\n", x, y);
    }
}



// ============================================================================
// 新增：雷达点云 360° 降维压缩与蓝牙发送函数
// ============================================================================

void Compress_And_Send_Lidar_Data(void)
{
    uint16_t compressed_sectors[72]; // 72 个扇区
    static uint8_t send_buf[148];           // 帧头(2) + 数据(144) + 帧尾(2)

    // 1. 初始化压缩数组，填入定义好的最大值 (MAX_VALID_DISTANCE)
    for(int i = 0; i < 72; i++) 
    {
        compressed_sectors[i] = MAX_VALID_DISTANCE;
    }

    // 2. 遍历 360 度的原始数组，将其压缩为 72 个扇区
    for(int i = 0; i < 360; i++) 
    {
        if(Lidar_Distance_Array[i] > 0) // 过滤雷达测出的无效 0 距离
        { 
            int sector_index = i / 5; // 每 5 度划分一个扇区 (360/5 = 72)
            
            // 核心逻辑：只保留该 5度 扇区内的最小距离 (离机器人最近的障碍物)
            if(Lidar_Distance_Array[i] < compressed_sectors[sector_index]) 
            {
                compressed_sectors[sector_index] = Lidar_Distance_Array[i];
            }
        }
    }

    // 3. 将没有检测到任何有效障碍物的扇区距离设为 0，方便手机端处理
    for(int i = 0; i < 72; i++) 
    {
        if(compressed_sectors[i] == MAX_VALID_DISTANCE) 
        {
            compressed_sectors[i] = 0; 
        }
    }

    // 4. 组装发送给手机蓝牙的数据包格式
    send_buf[0] = 0xAA; // 帧头 1
    send_buf[1] = 0xBB; // 帧头 2
    
    // 使用 memcpy 将 72 个 uint16_t (144字节) 快速拷贝进发送缓冲区
    memcpy(&send_buf[2], compressed_sectors, 144);
    
    send_buf[146] = 0x0D; // 帧尾 '\r'
    send_buf[147] = 0x0A; // 帧尾 '\n'

    // 5. 通过串口将数据发给蓝牙模块
    // ========================================================================
    // ★★★ 请在这里修改你的蓝牙串口句柄 ★★★
    // 假设你的 ECB02 蓝牙模块接在 USART3 上，就把下面的 &huart2 改成 &huart3
    // ========================================================================
    HAL_UART_Transmit_DMA(&huart4, send_buf, 148);
}


