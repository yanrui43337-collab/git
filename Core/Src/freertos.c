/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include "tuoluo.h"
#include "stdio.h"
#include "pid.h"
#include "Emm_V5.h"
#include <stdbool.h>
#include <lidar.h>
#include <Robot_control.h>
#include <ultrasonic_uart.h>
#include <stdlib.h>

extern uint8_t bt_rx_buf[64];
extern volatile int32_t step_motor_pos;
extern uint8_t Lidar_RxBuf[640];
extern uint16_t Lidar_WriteIndex;
extern uint16_t Lidar_ReadIndex;
extern void Lidar_ParseSingleFrame(uint8_t *frame);


// 🌟 为自动爬楼任务引入外部传感器变量和底层函数
extern int left_min;       // 雷达左侧最短距离 (由雷达代码解算)
extern int right_min;      // 雷达右侧最短距离
extern int Ultrasonic_Get_Distance(void); // 获取超声波距离
extern void Motor_Contro2(int m1_speed, int m2_speed, int m3_speed, int m4_speed); 
													 // 第二套驱动电机控制
													 
extern volatile uint32_t motor_update_count; // 必须加 extern，告诉任务这个变量在 main.c 里定义了
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 定义底盘队列的包裹格式 (正好 12 字节)
typedef struct {
    int x;
    int y;
    int w;
} ChassisMsg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for StepperTask */
osThreadId_t StepperTaskHandle;
const osThreadAttr_t StepperTask_attributes = {
  .name = "StepperTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for AutoClimbTask */
osThreadId_t AutoClimbTaskHandle;
const osThreadAttr_t AutoClimbTask_attributes = {
  .name = "AutoClimbTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ChassisQueue */
osMessageQueueId_t ChassisQueueHandle;
const osMessageQueueAttr_t ChassisQueue_attributes = {
  .name = "ChassisQueue"
};
/* Definitions for StepperQueue */
osMessageQueueId_t StepperQueueHandle;
const osMessageQueueAttr_t StepperQueue_attributes = {
  .name = "StepperQueue"
};
/* Definitions for Mutex_Stepper */
osMutexId_t Mutex_StepperHandle;
const osMutexAttr_t Mutex_Stepper_attributes = {
  .name = "Mutex_Stepper"
};
/* Definitions for Sem_BtRx */
osSemaphoreId_t Sem_BtRxHandle;
const osSemaphoreAttr_t Sem_BtRx_attributes = {
  .name = "Sem_BtRx"
};
/* Definitions for Sem_SensorRx */
osSemaphoreId_t Sem_SensorRxHandle;
const osSemaphoreAttr_t Sem_SensorRx_attributes = {
  .name = "Sem_SensorRx"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCommTask(void *argument);
void StartChassisTask(void *argument);
void StartStepperTask(void *argument);
void StartAutoClimbTask(void *argument);
void StartSensorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	
	
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of Mutex_Stepper */
  Mutex_StepperHandle = osMutexNew(&Mutex_Stepper_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Sem_BtRx */
  Sem_BtRxHandle = osSemaphoreNew(1, 1, &Sem_BtRx_attributes);

  /* creation of Sem_SensorRx */
  Sem_SensorRxHandle = osSemaphoreNew(1, 1, &Sem_SensorRx_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ChassisQueue */
  ChassisQueueHandle = osMessageQueueNew (4, 12, &ChassisQueue_attributes);

  /* creation of StepperQueue */
  StepperQueueHandle = osMessageQueueNew (4, 4, &StepperQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CommTask */
  CommTaskHandle = osThreadNew(StartCommTask, NULL, &CommTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);

  /* creation of StepperTask */
  StepperTaskHandle = osThreadNew(StartStepperTask, NULL, &StepperTask_attributes);

  /* creation of AutoClimbTask */
  AutoClimbTaskHandle = osThreadNew(StartAutoClimbTask, NULL, &AutoClimbTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartCommTask */
/**
  * @brief  Function implementing the CommTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  extern int robot_mode; 
  extern uint32_t last_bt_heartbeat;   // 引用 main.c 里的看门狗时间戳
  static bool is_bt_connected = false; // 记录当前蓝牙连接状态

  osThreadSuspend(AutoClimbTaskHandle); 
  
  for(;;)
  {
      if (osSemaphoreAcquire(Sem_BtRxHandle, 500) == osOK) 
      {
          // ==========================================
          // 收到数据，喂狗！
          // ==========================================
          last_bt_heartbeat = HAL_GetTick(); 
          
          if (is_bt_connected == false) {
              is_bt_connected = true;
              printf("📱 [蓝牙状态] 蓝牙已连接成功！\r\n");
          }

          // === 解析自动指令 ===
          if (strstr((char *)bt_rx_buf, "AUTO")) 
          {
              robot_mode = 1; 
              printf("🤖 收到切换自动模式指令! 准备爬楼...\r\n");
              osThreadResume(AutoClimbTaskHandle);   
          }
          // === 解析摇杆数据 ===
          else if (strstr((char *)bt_rx_buf, "X:")) 
          {
              int cmd_x = 0, cmd_y = 0, cmd_w = 0, cmd_z = 0; 
              char *p;
              
              if ((p = strstr((char *)bt_rx_buf, "X:")) != NULL) cmd_x = atoi(p + 2);
              if ((p = strstr((char *)bt_rx_buf, "Y:")) != NULL) cmd_y = atoi(p + 2);
              if ((p = strstr((char *)bt_rx_buf, "W:")) != NULL) cmd_w = atoi(p + 2);
              if ((p = strstr((char *)bt_rx_buf, "Z:")) != NULL) cmd_z = atoi(p + 2);
              
              if (robot_mode == 1) {
                  if (abs(cmd_x) > 20 || abs(cmd_y) > 20 || abs(cmd_z) > 20) {
                      printf("🛑 紧急打断：检测到人为干预，退出自动模式！\r\n");
                      osThreadSuspend(AutoClimbTaskHandle);  
                      extern void Motor_Contro2(int,int,int,int);
                      Motor_Contro2(0, 0, 0, 0); 
                      int stop_z = 0;
                      osMessageQueuePut(StepperQueueHandle, &stop_z, 0, 0); 
                      robot_mode = 0; 
                  }
              }
              
              if (robot_mode == 0) {
                  osThreadResume(ChassisTaskHandle);     
                  osThreadResume(StepperTaskHandle);     
                  
                  ChassisMsg_t msg = {cmd_x, cmd_y, cmd_w};
                  osMessageQueuePut(ChassisQueueHandle, &msg, 0, 0);
                  osMessageQueuePut(StepperQueueHandle, &cmd_z, 0, 0);
              }
          }
      }
      else 
      {
          // ==========================================
          // 500ms 没收到数据，检查是否超时
          // ==========================================
          if (is_bt_connected == true && (HAL_GetTick() - last_bt_heartbeat > 1000)) 
          {
              // 🌟 核心修复：只有在【手动模式】下，才判定为掉线急刹车！
              if (robot_mode == 0) 
              {
                  is_bt_connected = false;
                  printf("❌ [蓝牙状态] 蓝牙已断开连接，启动紧急保护！\r\n");
                  
                  extern void Motor_Contro2(int,int,int,int);
                  Motor_Contro2(0, 0, 0, 0); 
                  int stop_z = 0;
                  osMessageQueuePut(StepperQueueHandle, &stop_z, 0, 0); 
                  
                  osThreadSuspend(AutoClimbTaskHandle); 
              }
          }
      }
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  ChassisMsg_t rx_msg;
  for(;;)
  {		
      // 盯着传送带，拿到数据瞬间跑路
      if (osMessageQueueGet(ChassisQueueHandle, &rx_msg, NULL, osWaitForever) == osOK) 
      {
          Move_Mecanum(rx_msg.x, rx_msg.y, rx_msg.w);
      }
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartStepperTask */
/**
* @brief Function implementing the StepperTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStepperTask */
void StartStepperTask(void *argument)
{
  /* USER CODE BEGIN StartStepperTask */
  int target_z = 0;
  int last_sent_z = 0;            
  bool is_stepper_stopped = true; 
  extern int robot_mode;          
  
  #define MOTOR_MIN_LIMIT    0       
  #define MOTOR_MAX_LIMIT    18000   

  for(;;)
  {
      osMessageQueueGet(StepperQueueHandle, &target_z, NULL, 20); 

      osMutexAcquire(Mutex_StepperHandle, osWaitForever);
      
      // === 摇杆【向上】推 ===
      if (target_z > 20)  
      { 	
          if (step_motor_pos >= MOTOR_MAX_LIMIT) {
              if (!is_stepper_stopped) {
                  printf("⚠️ 触顶拦截！切换履带驱动！\r\n");
                  Emm_V5_Stop_Now(1, false); 
                  is_stepper_stopped = true;
                  last_sent_z = 0;
              }
              if (robot_mode == 0) {
                  int crawler_speed = target_z * 300; 
                  Motor_Contro2(crawler_speed, crawler_speed, crawler_speed, crawler_speed);
              }
          } 
          else {
              if (robot_mode == 0) Motor_Contro2(0, 0, 0, 0); 
              
              if (is_stepper_stopped || abs(target_z - last_sent_z) > 5) {
                  printf("🟢 步进任务: 执行【上升】指令 Z=%d，当前位置=%ld\r\n", target_z, step_motor_pos);
                  Emm_V5_Vel_Control(1, 0, target_z * 3, 50, false);
                  is_stepper_stopped = false;
                  last_sent_z = target_z;
              }
          }
      } 
      // === 摇杆【向下】推 ===
      else if (target_z < -20) 
      { 
          if (robot_mode == 0) Motor_Contro2(0, 0, 0, 0); 

          if (step_motor_pos <= MOTOR_MIN_LIMIT) {
              if (!is_stepper_stopped) {
                  printf("⛔ 触底拦截！已在最低点 0，禁止下降！\r\n");
                  Emm_V5_Stop_Now(1, false); 
                  is_stepper_stopped = true;
                  last_sent_z = 0;
              }
          } 
          else {
              if (is_stepper_stopped || abs(target_z - last_sent_z) > 5) {
                  printf("🔴 步进任务: 执行【下降】指令 Z=%d，当前位置=%ld\r\n", target_z, step_motor_pos);
                  Emm_V5_Vel_Control(1, 1, (-target_z) * 3, 50, false);
                  is_stepper_stopped = false;
                  last_sent_z = target_z;
              }
          }
      } 
      // === 松开摇杆居中 ===
      else 
      { 
          if (!is_stepper_stopped) {
              Emm_V5_Stop_Now(1, false); 
              is_stepper_stopped = true;
              last_sent_z = 0;
          }
          if (robot_mode == 0) Motor_Contro2(0, 0, 0, 0); 
      }
      
      osMutexRelease(Mutex_StepperHandle);
  }
  /* USER CODE END StartStepperTask */
}
/* USER CODE BEGIN Header_StartAutoClimbTask */
/**
* @brief Function implementing the AutoClimbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAutoClimbTask */
void StartAutoClimbTask(void *argument)
{
  /* USER CODE BEGIN StartAutoClimbTask */
  
  // 🎯 定义动作阈值 (方便你后续在实车上调参)
  #define AUTO_TARGET_UP       18000   // 步进电机目标升高位置
  #define AUTO_TARGET_DOWN     500     // 步进电机降落目标位置
  #define ULTRA_STOP_DIST      150     // 超声波停止距离
  #define LIDAR_STOP_DIST      200     // 雷达左右停止距离
  
  ChassisMsg_t msg_stop = {0, 0, 0};
  ChassisMsg_t msg_left = {0, 100, 0};   // 麦克纳姆轮左移
  ChassisMsg_t msg_right = {0, -100, 0}; // 麦克纳姆轮右移
  
  int stepper_cmd = 0; 

  for(;;)
  {
      printf("🤖 [自动模式] 启动新一轮爬楼循环！\r\n");

      // ==========================================================
      // 动作 1：步进电机逆时针旋转，升高到设定位置
      // ==========================================================
      printf("➡️ 动作1: 步进电机上升中...\r\n");
      stepper_cmd = 100; 
      osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
      
      // 🌟 监控升级：每 100ms 打印一次实时位置！
      while (step_motor_pos < AUTO_TARGET_UP) { 
          printf("🔄 [步进监控-上升] 当前位置: %ld / 目标: %d\r\n", step_motor_pos, AUTO_TARGET_UP);
          osDelay(100); 
      }
      
      stepper_cmd = 0;
      osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);

			// ==========================================================
      // 动作 2 & 3：Motor_Contro2 控制前进，直到超声波达到阈值
      // ==========================================================
      printf("➡️ 动作2: 履带/前轮开始前进(启用防断电软启动)...\r\n");
      
      // 🌟 核心修复 1：工业级大功率电机必须“软启动”！
      // 防止瞬间满载导致主板电压跌落，把超声波模块给“饿”重启了！
      for(int speed = 5000; speed <= 30000; speed += 5000) {
          Motor_Contro2(speed, speed, speed, speed);
//					uint8_t trig_cmd[1] = {0xA0}; 
//					HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
				
          osDelay(30); // 每 30ms 加一点速度，丝滑起步
      }
      
      int current_dist = 0;
      while (1) 
      { 
          // 发送前清空一下串口，防止被电磁干扰的残渣卡住
          __HAL_UART_CLEAR_OREFLAG(&huart5);
          
          uint8_t trig_cmd[1] = {0xA0}; 
          HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
          
          osDelay(60); // 🌟 稍微多给超声波 10ms 的回声等待时间

          current_dist = Ultrasonic_Get_Distance();
          printf("👀 [超声波监控] 当前前方距离: %d mm\r\n", current_dist);
          
          if (current_dist > 0 && current_dist <= ULTRA_STOP_DIST) {
              printf("🛑 触发避障！距离达标(%d mm)，紧急刹车！\r\n", current_dist);
              break; 
          }
      }
      Motor_Contro2(0, 0, 0, 0); 
      osDelay(500); // 刹车同样需要 0.5 秒消散反电动势
			
      // ==========================================================
      // 动作 4：步进电机顺时针旋转，回到原位置 (下降)
      // ==========================================================
      printf("➡️ 动作4: 步进电机下降中...\r\n");
      stepper_cmd = -100; 
      osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
      
      // 🌟 监控升级：每 100ms 打印一次实时位置！
      while (step_motor_pos > AUTO_TARGET_DOWN) { 
          printf("🔄 [步进监控-下降] 当前位置: %ld / 目标: %d\r\n", step_motor_pos, AUTO_TARGET_DOWN);
          osDelay(100); 
      }
      
      stepper_cmd = 0;
      osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);

      // ==========================================================
      // 动作 5 & 6：麦轮控制左移，直到雷达左侧达到阈值
      // ==========================================================
      printf("➡️ 动作5: 麦轮开始左移...\r\n");
      osMessageQueuePut(ChassisQueueHandle, &msg_left, 0, 0);
      
      // 🌟 监控升级：一边左移，一边疯狂打印雷达左侧距离！
      while (left_min > LIDAR_STOP_DIST || left_min <= 0) { 
          printf("📡 [雷达监控-左侧] 当前距离: %d mm\r\n", left_min);
          osDelay(100); 
      }
      
      printf("🛑 雷达左侧达标(%d mm)，停止左移。\r\n", left_min);
      osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
      osDelay(500); 


      // ==========================================================
      // 动作 7 & 8：麦轮控制右移，直到雷达右侧达到阈值
      // ==========================================================
      printf("➡️ 动作7: 麦轮开始右移...\r\n");
      osMessageQueuePut(ChassisQueueHandle, &msg_right, 0, 0);
      
      // 🌟 监控升级：一边右移，一边疯狂打印雷达右侧距离！
      while (right_min > LIDAR_STOP_DIST || right_min <= 0) { 
          printf("📡 [雷达监控-右侧] 当前距离: %d mm\r\n", right_min);
          osDelay(100); 
      }
      
      printf("🛑 雷达右侧达标(%d mm)，停止右移。\r\n", right_min);
      osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
      osDelay(500); 

      // ==========================================================
      printf("✅ 单次循环完成！\r\n");
  }
  /* USER CODE END StartAutoClimbTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  
  uint32_t last_send_time = 0; 
  extern void Compress_And_Send_Lidar_Data(void); 
  // 引入你在 lidar.c 写的距离计算函数
  extern uint16_t Lidar_Get_Min_Distance_In_Range(uint16_t start_angle, uint16_t end_angle);
  
  // 引入全局状态变量
  extern int left_min;       
  extern int right_min;      
  extern int front_min;

  for(;;)
  {
      if (osSemaphoreAcquire(Sem_SensorRxHandle, osWaitForever) == osOK) 
      {
          // === 1. 雷达解包核心逻辑 (保持不变) ===
          while (Lidar_ReadIndex != Lidar_WriteIndex)
          {
              if(Lidar_RxBuf[Lidar_ReadIndex] == 0xA5)
              {
                  uint16_t next_index = (Lidar_ReadIndex + 1) % 640;
                  if(Lidar_RxBuf[next_index] == 0x5A)
                  {
                      uint16_t available_data;
                      if(Lidar_WriteIndex >= Lidar_ReadIndex) {
                          available_data = Lidar_WriteIndex - Lidar_ReadIndex;
                      } else {
                          available_data = 640 - Lidar_ReadIndex + Lidar_WriteIndex;
                      }

                      if(available_data >= 58)
                      {
                          uint8_t single_frame[58];
                          for(int i = 0; i < 58; i++) {
                              single_frame[i] = Lidar_RxBuf[(Lidar_ReadIndex + i) % 640];
                          }
                          
                          Lidar_ParseSingleFrame(single_frame);
                          
                          Lidar_ReadIndex = (Lidar_ReadIndex + 58) % 640;
                          continue; 
                      }
                      else { break; }
                  }
              }
              Lidar_ReadIndex = (Lidar_ReadIndex + 1) % 640;
          }
          // === 解包结束 ===

          // 🌟 核心修复 2：每次解包完，立刻计算出左中右的最短距离供自动模式使用！
          // 注意：这里的角度(如260~280)请根据你雷达实际车头朝向微调
          front_min = Lidar_Get_Min_Distance_In_Range(350, 10);  // 前方正负10度
          left_min  = Lidar_Get_Min_Distance_In_Range(260, 280); // 左侧 270度 附近
          right_min = Lidar_Get_Min_Distance_In_Range(80, 100);  // 右侧 90度 附近

          // 🌟 核心修复 3：如你所愿，自动模式不发点云省算力！只在手动模式(robot_mode == 0)时才发！
					extern int robot_mode; 
          if (robot_mode == 0 && (HAL_GetTick() - last_send_time > 100)) 
          {
              Compress_And_Send_Lidar_Data();
              last_send_time = HAL_GetTick(); 
          }
      }
  }
  /* USER CODE END StartSensorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// 🌟 终极版：HAL库官方错误回调函数（专治一切电磁干扰导致的死锁）
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart); // 清除追尾
        __HAL_UART_CLEAR_FEFLAG(huart);  // 清除帧错误
        __HAL_UART_CLEAR_NEFLAG(huart);  // 🌟 核心：清除电机下降时产生的噪声错误！
        __HAL_UART_CLEAR_PEFLAG(huart);  // 清除奇偶校验错误
        
        extern uint8_t rxCmd[];
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, rxCmd, 128); // 满血复活
    }
    else if (huart->Instance == UART5)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        
        extern uint8_t ultra_rx_buf[];
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16);
    }
}
/* USER CODE END Application */

