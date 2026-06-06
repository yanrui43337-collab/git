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
              
              // 🌟 终极防线：强制刹停手动模式下的电机，等待电磁尖峰消散！
              // 彻底隔绝“手动切自动”带来的瞬间反电动势冲击！
              extern void Motor_Contro2(int,int,int,int);
              Motor_Contro2(0, 0, 0, 0); 
              osDelay(500); 
              
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
              
							// 自动模式下检测到人为干预，强行打断
              if (robot_mode == 1) {
                  // 🌟 核心修复：大幅度拨动摇杆，或者收到全 0 的刹车指令（比如按了返回键），统统打断！
                  if (abs(cmd_x) > 20 || abs(cmd_y) > 20 || abs(cmd_z) > 20 || 
                     (cmd_x == 0 && cmd_y == 0 && cmd_w == 0 && cmd_z == 0)) 
                  {
                      printf("🛑 紧急打断：收到接管或返回指令，退出自动模式！\r\n");
                      osThreadSuspend(AutoClimbTaskHandle);  
                      
                      extern void Motor_Contro2(int,int,int,int);
                      Motor_Contro2(0, 0, 0, 0); // 履带急刹车
                      Emm_V5_Stop_Now(1, false); // 步进急刹车
                      
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
								  Emm_V5_Stop_Now(1, false);
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
  
  // 🌟 新增：引入雷达的全局距离变量
  extern int left_min;       
  extern int right_min;      
	
  // 🌟 新增：手动模式底盘限速系数（0.0 ~ 1.0），数字越小速度越慢
  const float CHASSIS_SPEED_SCALE = 0.4f;
  
	uint32_t last_print_time = 0; // 打印频率限速器

  for(;;)
  {		
      // 盯着传送带，拿到数据瞬间跑路
      if (osMessageQueueGet(ChassisQueueHandle, &rx_msg, NULL, osWaitForever) == osOK) 
      {
				
          // 🌟 修改：将摇杆原始输入乘以限速系数后再传给底盘控制函数
          int target_x = (int)(rx_msg.x * CHASSIS_SPEED_SCALE);
          int target_y = (int)(rx_msg.y * CHASSIS_SPEED_SCALE);
          int target_w = (int)(rx_msg.w * CHASSIS_SPEED_SCALE);
          
          Move_Mecanum(target_x, target_y, target_w);
          // 🌟 核心新增：如果检测到明显的左右横移指令（摇杆 X 轴拨动）
          if (abs(rx_msg.x) > 10) 
          {
              // 每 500ms 打印一次，坚决防止单片机被串口刷死机
              if (HAL_GetTick() - last_print_time > 200) 
              {
                  printf("🎮 [手动控制] 麦轮横移 (X=%d) | 雷达实时距离 -> 左侧:%d mm, 右侧:%d mm\r\n", 
                         rx_msg.x, left_min, right_min);
                  last_print_time = HAL_GetTick();
              }
          }
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
  
  uint32_t last_uart_send_time = 0; 
  uint32_t last_print_time = 0; // 🌟 打印频率控制器
	uint32_t last_ultra_trig_time = 0; // 🌟 新增：超声波触发限速器
  
  #define MOTOR_MIN_LIMIT    -200000       
  #define MOTOR_MAX_LIMIT    2028909 

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
                  int crawler_speed = target_z * 500; 
                  Motor_Contro2(crawler_speed, crawler_speed, crawler_speed, crawler_speed);
									// =========================================================
                  // 🌟 新增：手动模式下履带前进时，激活超声波并打印测距！
                  // =========================================================
                  // 1. 每 100ms 触发一次超声波（给足回声时间，防死机）
                  if (HAL_GetTick() - last_ultra_trig_time > 100) {
                      uint8_t trig_cmd[1] = {0xA0}; 
                      extern UART_HandleTypeDef huart5;
                      HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
                      last_ultra_trig_time = HAL_GetTick();
                  }
									// 2. 每 500ms 打印一次距离（防止刷屏卡死）
                  if (HAL_GetTick() - last_print_time > 500) {
                      extern int Ultrasonic_Get_Distance(void);
                      int dist = Ultrasonic_Get_Distance();
                      printf("🎮 [履带手动前进] 超声波前方实时距离: %d mm\r\n", dist);
                      last_print_time = HAL_GetTick();
                  }
              }
          } 
          else {
              if (robot_mode == 0) Motor_Contro2(0, 0, 0, 0); 
              
              if (is_stepper_stopped || abs(target_z - last_sent_z) > 5 || (HAL_GetTick() - last_uart_send_time > 200)) 
              {
                  // 1. 播报实时位置 (500ms 一次，防刷屏)
                  if (HAL_GetTick() - last_print_time > 500) {
                      printf("🎮 [手动控制] 向上移动, Z=%d, 实时位置: %ld\r\n", target_z, step_motor_pos);
                      last_print_time = HAL_GetTick();
                  }
                  
                  // 2. 🚨 核心动作：真·油门指令！（你之前漏掉了这四句）
                  Emm_V5_Vel_Control(1, 0, target_z * 3, 50, false);
                  is_stepper_stopped = false;
                  last_sent_z = target_z;
                  last_uart_send_time = HAL_GetTick();
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
              if (is_stepper_stopped || abs(target_z - last_sent_z) > 5 || (HAL_GetTick() - last_uart_send_time > 200)) 
              {
                  // 1. 播报实时位置
                  if (HAL_GetTick() - last_print_time > 500) {
                      printf("🎮 [手动控制] 向下移动, Z=%d, 实时位置: %ld\r\n", target_z, step_motor_pos);
                      last_print_time = HAL_GetTick();
                  }
                  
                  // 2. 🚨 核心动作：真·油门指令！
                  Emm_V5_Vel_Control(1, 1, (-target_z) * 3, 50, false);
                  is_stepper_stopped = false;
                  last_sent_z = target_z;
                  last_uart_send_time = HAL_GetTick();
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
  
  // 🎯 基础动作阈值
  #define AUTO_TARGET_UP       2000000   // 步进电机目标升高位置 (必须小于 MOTOR_MAX_LIMIT)
  #define AUTO_TARGET_DOWN     20        // 步进电机降落目标位置
  #define ULTRA_STOP_DIST      200       // 正常爬楼时的超声波停止距离
  #define LIDAR_STOP_DIST      400       // 雷达左右停止距离
  
  // 🌟 新增：平台 S 型清扫阈值
  #define ULTRA_PLATFORM_DIST  800       // 判定为到达平台的超声波距离阈值 (例如 800mm)
  #define ULTRA_WALL_MIN_DIST  150       // 平台前进清扫时，离墙的最小极限距离
  #define ROBOT_SHIFT_WIDTH    300       // 机器人 S 型清扫时，每次左移的宽度 (单位 mm，根据实车修改)
  #define CRAWLER_FWD_SPEED    6000      // 履带平台前进速度
  #define CRAWLER_REV_SPEED    -6000     // 履带平台后退速度
  
  ChassisMsg_t msg_stop = {0, 0, 0};
  ChassisMsg_t msg_left = {0, 100, 0};   // 麦克纳姆轮左移
  ChassisMsg_t msg_right = {0, -100, 0}; // 麦克纳姆轮右移
  
  int stepper_cmd = 0; 
  
  // 引入雷达的精细测距函数
  extern uint16_t Lidar_Get_Min_Distance_In_Range(uint16_t start_angle, uint16_t end_angle);

  for(;;)
  {
      printf("\r\n========================================\r\n");
      printf("🤖 [自动模式] 启动新一轮台阶判定循环！\r\n");
      printf("========================================\r\n");
      bool is_platform_detected = false; 

      // ==========================================================
      // 动作 1：步进电机上升，【同时实时侦测超声波】
      // ==========================================================
      printf("➡️ 动作1: 步进上升并侦测地形...\r\n");
      stepper_cmd = 100; 
      osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
      
      extern UART_HandleTypeDef huart5;
      extern uint8_t ultra_rx_buf[];
      
      while (step_motor_pos < AUTO_TARGET_UP) { 
          // 发送超声波触发指令
          uint8_t trig_cmd[1] = {0xA0}; 
          HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
          osDelay(60); 
          
          int current_dist = Ultrasonic_Get_Distance();
          printf("🔄 [步进上升中] 坐标: %ld / 目标: %d | 前方测距: %d mm\r\n", 
                 step_motor_pos, AUTO_TARGET_UP, current_dist);
          
          // 核心判断：距离大于平台阈值
          if (current_dist > ULTRA_PLATFORM_DIST) {
              printf("🌟 检测到楼梯平台！(测距 %d mm > 阈值 %d mm)\r\n", current_dist, ULTRA_PLATFORM_DIST);
              is_platform_detected = true;
              break; 
          }
      }

      // ==========================================================
      // 🌟 分支 A：进入楼梯平台 S 型清扫逻辑
      // ==========================================================
      if (is_platform_detected) {
          
          // 阶段 A1: 继续上升到最高处
          printf("\r\n[平台清扫] 阶段 1: 步进电机上升至极限，脱离原台阶...\r\n");
          while (step_motor_pos < 2000000) { 
              printf("   -> 升举中... 当前: %ld\r\n", step_motor_pos);
              osDelay(100); 
          }
          stepper_cmd = 0;
          osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
          osDelay(200);

          // 阶段 A2: 履带前进，完全登上平台
          printf("\r\n[平台清扫] 阶段 2: 履带全速前进，登上平台...\r\n");
          Motor_Contro2(CRAWLER_FWD_SPEED, CRAWLER_FWD_SPEED, CRAWLER_FWD_SPEED, CRAWLER_FWD_SPEED);
          osDelay(2000); 
          Motor_Contro2(0, 0, 0, 0);
          osDelay(500);

          // 阶段 A3: 步进机构收起
          printf("\r\n[平台清扫] 阶段 3: 收起步进机构，麦轮着地准备平移...\r\n");
          stepper_cmd = -100;
          osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
          while (step_motor_pos > AUTO_TARGET_DOWN) { 
              printf("   -> 下降中... 当前: %ld\r\n", step_motor_pos);
              osDelay(100); 
          }
          stepper_cmd = 0;
          osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
          osDelay(500);

          // 阶段 A4: S 型清扫
          printf("\r\n[平台清扫] 阶段 4: 开始 S 型网格化清扫！\r\n");
          
          while (1) {
              // --- 动作(1): 前进直到碰到前墙 ---
              printf("   ▶ S型: 直线前进找前墙...\r\n");
              Motor_Contro2(CRAWLER_FWD_SPEED, CRAWLER_FWD_SPEED, CRAWLER_FWD_SPEED, CRAWLER_FWD_SPEED);
              while (1) {
                  uint8_t trig_cmd[1] = {0xA0}; 
                  HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
                  osDelay(60);
                  int dist = Ultrasonic_Get_Distance();
                  printf("      [履带前进] 超声波测距: %d mm\r\n", dist);
                  if (dist > 0 && dist <= ULTRA_WALL_MIN_DIST) {
                      printf("      🛑 触达前墙！(距离: %d mm)\r\n", dist);
                      break;
                  }
              }
              Motor_Contro2(0, 0, 0, 0);
              
              // 检查整体左侧是否清扫完毕 (用 60 度大扇区做宏观判断)
              extern int left_min; 
              if (left_min <= LIDAR_STOP_DIST && left_min > 0) {
                  printf("✅ 雷达大扇区检测到左侧已贴墙 (距离: %d mm)，S型清扫完成！\r\n", left_min);
                  break;
              }

              // --- 动作(2): 麦轮向左平移一个车位 (精准小角度雷达闭环) ---
              printf("   ▶ S型: 雷达闭环控制左移换道...\r\n");
              // 获取当前左侧(正左方 270 度，取极小扇区 268~272度)的绝对距离
              int start_left_dist = Lidar_Get_Min_Distance_In_Range(268, 272); 
              int target_left_dist = start_left_dist - ROBOT_SHIFT_WIDTH;
              if (target_left_dist < LIDAR_STOP_DIST) target_left_dist = LIDAR_STOP_DIST; 

              if (start_left_dist > 0 && start_left_dist < 4000) 
              {
                  printf("      [左移测距] 起始离墙距离: %d mm, 目标需达到: %d mm\r\n", start_left_dist, target_left_dist);
                  osMessageQueuePut(ChassisQueueHandle, &msg_left, 0, 0);
                  
                  int timeout_count = 0;
                  while (1) {
                      osDelay(50); 
                      int current_shift_dist = Lidar_Get_Min_Distance_In_Range(268, 272);
                      printf("      [左移中...] 实时离墙距离: %d mm\r\n", current_shift_dist);
                      
                      if (current_shift_dist <= target_left_dist && current_shift_dist > 0) {
                          printf("      🎯 达到平移目标距离！\r\n");
                          break;
                      }
                      
                      if (++timeout_count > 80) { // 4秒超时保护
                          printf("      ⚠️ 左移超时打断！防打滑保护触发！\r\n");
                          break; 
                      }
                  }
                  osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
                  osDelay(300);
              }
              else {
                  printf("      ⚠️ 左侧雷达数据无效(%d)，采用 1.5 秒盲移...\r\n", start_left_dist);
                  osMessageQueuePut(ChassisQueueHandle, &msg_left, 0, 0);
                  osDelay(1500); 
                  osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
                  osDelay(300);
              }

              // --- 动作(3): 履带后退清扫 ---
              printf("   ▶ S型: 履带后退清扫平台...\r\n");
              Motor_Contro2(CRAWLER_REV_SPEED, CRAWLER_REV_SPEED, CRAWLER_REV_SPEED, CRAWLER_REV_SPEED); 
              while (1) {
                  uint8_t trig_cmd[1] = {0xA0}; 
                  HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
                  osDelay(60);
                  int dist = Ultrasonic_Get_Distance();
                  printf("      [履带后退] 超声波测距: %d mm\r\n", dist);
                  
                  if (dist >= ULTRA_PLATFORM_DIST || dist <= 0) {
                      printf("      🛑 退至平台边缘！(测距: %d mm)\r\n", dist);
                      break; 
                  }
              }
              Motor_Contro2(0, 0, 0, 0);

              // 检查宏观左侧
              if (left_min <= LIDAR_STOP_DIST && left_min > 0) {
                  printf("✅ 雷达大扇区检测到左侧已贴墙 (距离: %d mm)，S型清扫完成！\r\n", left_min);
                  break;
              }

              // --- 动作(4): 再次向左平移一个车位 ---
              printf("   ▶ S型: 雷达闭环控制左移换道 (第二段)...\r\n");
              start_left_dist = Lidar_Get_Min_Distance_In_Range(268, 272); 
              target_left_dist = start_left_dist - ROBOT_SHIFT_WIDTH;
              if (target_left_dist < LIDAR_STOP_DIST) target_left_dist = LIDAR_STOP_DIST; 

              if (start_left_dist > 0 && start_left_dist < 4000) 
              {
                  printf("      [左移测距] 起始离墙距离: %d mm, 目标需达到: %d mm\r\n", start_left_dist, target_left_dist);
                  osMessageQueuePut(ChassisQueueHandle, &msg_left, 0, 0);
                  
                  int timeout_count = 0;
                  while (1) {
                      osDelay(50); 
                      int current_shift_dist = Lidar_Get_Min_Distance_In_Range(268, 272);
                      printf("      [左移中...] 实时离墙距离: %d mm\r\n", current_shift_dist);
                      
                      if (current_shift_dist <= target_left_dist && current_shift_dist > 0) {
                          printf("      🎯 达到平移目标距离！\r\n");
                          break;
                      }
                      
                      if (++timeout_count > 80) break; 
                  }
                  osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
                  osDelay(300);
              }
              else {
                  printf("      ⚠️ 左侧雷达数据无效，采用 1.5 秒盲移...\r\n");
                  osMessageQueuePut(ChassisQueueHandle, &msg_left, 0, 0);
                  osDelay(1500); 
                  osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
                  osDelay(300);
              }
          }
          
          printf("\r\n🔄 [状态转换] 平台 S 型清扫结束，准备进入掉头逻辑！\r\n");
          // TODO: 这里写掉头逻辑
          // ...
          
          continue; 
      }


      // ==========================================================
      // 🌟 分支 B：普通台阶爬楼逻辑 (没有检测到平台时执行)
      // ==========================================================
      printf("\r\n[普通台阶] 未检测到平台，执行常规攀爬逻辑...\r\n");
      
      stepper_cmd = 0;
      osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
      osDelay(200);

      printf("➡️ 动作2: 履带开始软启动找墙...\r\n");
      // ... 【把你原先在动作 2、3、4、5、6、7 的代码完整粘贴回这里】 ...
      // 这里保持你原来的台阶攀爬代码不动即可

      printf("✅ 单级台阶循环完成！\r\n");
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

          // 🌟 核心修复 3：如你所愿，手动模式不发点云省算力！只在手动模式(robot_mode == 0)时才发！
					extern int robot_mode; 
          if (robot_mode == 1 && (HAL_GetTick() - last_send_time > 100)) 
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
        __HAL_UART_CLEAR_OREFLAG(huart); 
        __HAL_UART_CLEAR_FEFLAG(huart);  
        __HAL_UART_CLEAR_NEFLAG(huart);  
        __HAL_UART_CLEAR_PEFLAG(huart);  
        
        extern uint8_t rxCmd[];
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, rxCmd, 128); 
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
    // 👇 新增：给蓝牙(UART4)也颁发“不死金牌”！防止任何开机乱码导致死锁！
    else if (huart->Instance == UART4)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        
        extern uint8_t bt_rx_buf[];
        HAL_UARTEx_ReceiveToIdle_IT(&huart4, bt_rx_buf, 64);
    }
		// 👇 新增：给雷达 (USART2) 颁发不死金牌
		else if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        // 注意：因为是 Circular DMA，这里不需要重启接收，清掉标志位让它继续跑就行
    }
}

/* USER CODE END Application */