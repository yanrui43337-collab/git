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
extern TIM_HandleTypeDef htim15; // 引入 TIM15
// 🌟 定义开启扫地机构的绝招

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 定义底盘队列的包裹格式 (正好 12 字节)
typedef struct {
    int x;
    int y;
    int w;
} ChassisMsg_t;

/* USER CODE BEGIN FunctionPrototypes */
// 🌟 终极安全机制：带容差的极速防滑行刹车
void Safe_Stepper_Stop(void) 
{
    extern osMessageQueueId_t StepperQueueHandle; 
    int stop_cmd_val = 0;
    int32_t last_pos = step_motor_pos; 
    int static_count = 0;

    printf("\r\n🛑 [安全机制] 步进电机紧急刹车...\r\n");

    // 1. 同步清零队列，防止堆积
    osMessageQueuePut(StepperQueueHandle, &stop_cmd_val, 0, 0);

    // 2. 发送刹车指令
    Emm_V5_Stop_Now(1, false);

    while (1) 
    {
        // 观察周期从冗长的 50ms 缩短为 30ms
        osDelay(30); 

        // 🌟 核心修复：允许 ±10 个脉冲的闭环震荡误差！
        // 闭环电机在锁轴时会轻微抖动，严苛的 == 会导致程序死循环！
        if (abs(step_motor_pos - last_pos) <= 10) {
            static_count++;
            if (static_count >= 2) {
                // 连续 60ms 稳定，立刻认定停稳，绝不拖泥带水
                break;
            }
        } else {
            static_count = 0;
            Emm_V5_Stop_Now(1, false); // 如果真在滑，补发刹车
        }
        
        last_pos = step_motor_pos;
    }
    
    // 停稳缓冲时间从 200ms 大幅缩短到 50ms！
    osDelay(50); 
}
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
						
              // 🌟 核心修复 1：清空缓存，防止无限死循环触发 AUTO
              memset(bt_rx_buf, 0, 64); 
              // 🌟 核心修复 2：重置看门狗时间，防止刚才的 Delay 消耗了时间
              last_bt_heartbeat = HAL_GetTick();
						
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
                      STOP_SWEEPER(); // 🌟 加在这里！紧急关闭扫地机构！
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
                  
									// 🌟🌟🌟 新增：手动模式下扫地机构的动态联动控制 🌟🌟🌟
                  // 摇杆死区阈值：底盘(10)，步进电机(20)
                  if (abs(cmd_z) > 20) {
                      // 1. 如果正在操作步进电机（Z轴有动作），优先强制关闭扫地机构
                      STOP_SWEEPER();
                  } 
                  else if (abs(cmd_x) > 10 || abs(cmd_y) > 10 || abs(cmd_w) > 10) {
                      // 2. 如果步进没动，且底盘摇杆有动作（无论是左右Y、前后X还是旋转W），开启扫地机构
                    
                  } 
                  else {
                      // 3. 如果摇杆全部居中（全车静止），关闭扫地机构
                      STOP_SWEEPER();
                  }
                  // 🌟🌟🌟 新增结束 🌟🌟🌟
									
                  ChassisMsg_t msg = {cmd_x, cmd_y, cmd_w};
                  osMessageQueuePut(ChassisQueueHandle, &msg, 0, 0);
                  osMessageQueuePut(StepperQueueHandle, &cmd_z, 0, 0);
              }
          }
      }
			else 
      {
          // ==========================================
          // 500ms 没收到有效数据，开始检查是否超时断连
          // ==========================================
          if (is_bt_connected == true && robot_mode == 0 && (HAL_GetTick() - last_bt_heartbeat > 1000)) 
          {
              is_bt_connected = false;
              printf("❌ [蓝牙卫士] 信号丢失超过 1 秒，判定为断连！执行全车紧急刹车！\r\n");
              
              // 1. 紧急切断外围机构
              STOP_SWEEPER(); 
              
              // 2. 履带/麦轮底盘物理急刹
              extern void Motor_Contro2(int,int,int,int);
              Motor_Contro2(0, 0, 0, 0); 
              
              // 3. 步进电机锁轴抱死
              Emm_V5_Stop_Now(1, false);
              int stop_z = 0;
              osMessageQueuePut(StepperQueueHandle, &stop_z, 0, 0); 
              
              // 4. 如果正在自动爬楼，立刻挂起任务并切回手动模式！
              if (robot_mode == 1) {
                  osThreadSuspend(AutoClimbTaskHandle); 
                  robot_mode = 0; 
                  printf("⚠️ 已强制打断自动爬楼任务！\r\n");
              }

              // 5. 🌟 终极防死锁：暴力重启蓝牙 UART4 接收链路
              // 很多时候连不上是因为底层的 ORE (溢出错误) 把串口卡死了
              extern UART_HandleTypeDef huart4;
              HAL_UART_AbortReceive(&huart4);
              __HAL_UART_CLEAR_OREFLAG(&huart4);
              __HAL_UART_CLEAR_NEFLAG(&huart4);
              __HAL_UART_CLEAR_FEFLAG(&huart4);
              HAL_UARTEx_ReceiveToIdle_IT(&huart4, (uint8_t *)bt_rx_buf, 64);
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
  rx_msg.x = 0; rx_msg.y = 0; rx_msg.w = 0;
  
  // 🌟 你的截图里引入的 PID 变量
  extern float target_rpm_m1, target_rpm_m2, target_rpm_m3, target_rpm_m4;
  extern float actual_rpm_m1, actual_rpm_m2, actual_rpm_m3, actual_rpm_m4;
  
  const float CHASSIS_SPEED_SCALE = 0.3f;
  uint32_t last_print_time = 0; 
  
  // 🌟 纠偏专用的魔法变量
  float target_yaw = 0.0f;       
  bool is_yaw_locked = false;    
  //float Kp_yaw = -1.0f;           // 如果车子左右摆动太厉害，改小至 1.0f
	float Kp_yaw = 0.0f;
  for(;;)
  {        
      // 1. 疯狂处理环形缓冲区里的数据包
      extern void IMU_UART_Process(void);
      IMU_UART_Process();
      
      // 2. 拿到最新鲜的车头角度
      extern float IMU_Get_Yaw(void);
      float current_yaw = IMU_Get_Yaw();

      // 降频打印，防止串口阻塞
      if (HAL_GetTick() - last_print_time > 500) {
          //printf("【IMU】Yaw: %.2f | RPM_M1: %.1f\r\n", current_yaw, actual_rpm_m1);
          last_print_time = HAL_GetTick();
      }

      if (osMessageQueueGet(ChassisQueueHandle, &rx_msg, NULL, 5) == osOK) 
      {
          int target_x = (int)(rx_msg.x * CHASSIS_SPEED_SCALE);
          int target_y = (int)(rx_msg.y * CHASSIS_SPEED_SCALE);
          int target_w = (int)(rx_msg.w * CHASSIS_SPEED_SCALE);
          
          // ==========================================================
          // 🌟 核心魔法：底盘平移闭环纠偏逻辑
          // ==========================================================
          if (target_x != 0 || target_y != 0) 
          {
              if (target_w == 0) 
              {
                  if (!is_yaw_locked) {
                      target_yaw = current_yaw; 
                      is_yaw_locked = true;
                  }
                  
                  float yaw_error = target_yaw - current_yaw;
                  
                  if (yaw_error > 180.0f) yaw_error -= 360.0f;
                  if (yaw_error < -180.0f) yaw_error += 360.0f;
                  
                  int comp_w = (int)(Kp_yaw * yaw_error);
                  
                  if (comp_w > 40) comp_w = 40;
                  if (comp_w < -40) comp_w = -40;
                  
                  target_w += comp_w; 
              }
              else {
                  is_yaw_locked = false; 
              }
          }
          else {
              is_yaw_locked = false; 
          }
          
          // 把处理好的指令发给底层电机驱动
          Move_Mecanum(target_x, target_y, target_w);
      }
      osDelay(5);
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
	
	uint32_t last_poll_time = 0; // 🌟 1. 轮询时间戳
  
  #define MOTOR_MIN_LIMIT    -80000       
  #define MOTOR_MAX_LIMIT    2000000 

  for(;;)
  {
      osMessageQueueGet(StepperQueueHandle, &target_z, NULL, 20); 
			
      osMutexAcquire(Mutex_StepperHandle, osWaitForever);
      
      // === 摇杆【向上】推 ===
      if (target_z > 20)  
      { 	
          // 🌟 引入容差区间 (-200)，只要进入这个范围就认为触顶，防止反复横跳
          if (step_motor_pos >= (MOTOR_MAX_LIMIT - 200)) {
              if (!is_stepper_stopped) {
                  printf("⚠️ 触顶拦截！切换履带驱动！\r\n");
                  Emm_V5_Stop_Now(1, false); 
                  is_stepper_stopped = true;
                  last_sent_z = 0; // 重置记忆变量，让下方的履带限频逻辑能立即触发一次
              }
              
              if (robot_mode == 0) {
                  // 🌟 修复 1：调整摇杆乘积，并加入强制最高限幅，防止溢出变负数
                  int crawler_speed = target_z * 250; // 如果 target_z 最大是 100，这里大概是 10000
                  if (crawler_speed > 15000) crawler_speed = 15000; // 封死上限
                  
                  // 🌟 修复 3：为履带控制也套上“限频保护壳”，避免串口发死机
                  if (abs(target_z - last_sent_z) > 5 || (HAL_GetTick() - last_uart_send_time > 200)) 
                  {
                      Motor_Contro2(crawler_speed, crawler_speed, crawler_speed, crawler_speed);
                      last_sent_z = target_z;
                      last_uart_send_time = HAL_GetTick();
                  }

                  // --- 下面保留你原本的超声波雷达探测打印逻辑 ---
                  if (HAL_GetTick() - last_ultra_trig_time > 100) {
                      uint8_t trig_cmd[1] = {0xA0}; 
                      extern UART_HandleTypeDef huart5;
                      HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
                      last_ultra_trig_time = HAL_GetTick();
                  }
                  if (HAL_GetTick() - last_print_time > 500) {
                      extern int Ultrasonic_Get_Distance(void);
                      int dist = Ultrasonic_Get_Distance();
                      printf("🎮 [履带手动前进] 超声波前方实时距离: %d mm\r\n", dist);
                      last_print_time = HAL_GetTick();
                  }
              }
          } 
          else {
              // 🌟 仅当真正离开顶部容差区间时，才刹停履带，避免误杀
              if (robot_mode == 0 && is_stepper_stopped) {
                  Motor_Contro2(0, 0, 0, 0); 
              }
              
              if (is_stepper_stopped || abs(target_z - last_sent_z) > 5 || (HAL_GetTick() - last_uart_send_time > 200)) 
              {
                  if (HAL_GetTick() - last_print_time > 500) {
                      printf("🎮 [手动控制] 向上移动, Z=%d, 实时位置: %ld\r\n", target_z, step_motor_pos);
                      last_print_time = HAL_GetTick();
                  }
                  
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
  /* Infinite loop */
  for(;;)
  {
		AutoClimb_Process();
    osDelay(1);
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
  extern uint16_t Lidar_Get_Min_Distance_In_Range(uint16_t start_angle, uint16_t end_angle);
  
  extern int left_min;       
  extern int right_min;      
  extern int front_min;

  for(;;)
  {
      // 🌟 核心修复 1：将 osWaitForever 改为 500！500ms 没收到数据说明雷达死了！
      if (osSemaphoreAcquire(Sem_SensorRxHandle, 500) == osOK) 
      {
          // === 雷达解包核心逻辑 ===
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

          // === 计算距离 ===
          front_min = Lidar_Get_Min_Distance_In_Range(350, 10);  
          left_min  = Lidar_Get_Min_Distance_In_Range(250, 290); 
          right_min = Lidar_Get_Min_Distance_In_Range(80, 100);  

          extern int robot_mode; 
          if (robot_mode == 1 && (HAL_GetTick() - last_send_time > 100)) 
          {
              Compress_And_Send_Lidar_Data();
              last_send_time = HAL_GetTick(); 
          }
      }
      else 
      {
          // 🌟 核心修复 2：超时触发！强制打断卡死的串口，重新唤醒雷达 DMA 通道！
          printf("🚨 [警告] 雷达数据停更超时，尝试强行重启 DMA 通道！\r\n");
          extern UART_HandleTypeDef huart2;
          HAL_UART_AbortReceive(&huart2);
          __HAL_UART_CLEAR_OREFLAG(&huart2);
          HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Lidar_RxBuf, 640);
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

