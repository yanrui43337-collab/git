#include "auto_climb.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include <string.h>
#include "stdio.h"
#include "Emm_V5.h"
#include <stdbool.h>
#include <lidar.h>
#include <Robot_control.h>
#include <ultrasonic_uart.h>
#include <stdlib.h>
#include "main.h" 

/* ========================================================== */
/* ==================== 1. 宏定义区域 ======================= */
/* ========================================================== */
#define AUTO_TARGET_UP       2098909   
#define AUTO_TARGET_DOWN     -10000    
#define ULTRA_STOP_DIST      50        
#define ULTRA_STEP_FWD_DIST  125       

#define LIDAR_LEFT_STOP_DIST       450   
#define LIDAR_RIGHT_STOP_DIST      500   
#define LIDAR_RIGHT_WALL_MIN_DIST  210   
#define LIDAR_RIGHT_PLATFORM_LIMIT 890   
#define ROBOT_FWD_STEP_DIST        150   
#define LIDAR_FRONT_WALL_MIN_DIST  210   

#define ULTRA_PLATFORM_DETECT  1700      
#define MAX_SHIFT_TIMES        8         

#define LIDAR_LAST_STEP_DIST    1814  	 
#define LIDAR_PRE_TURN_TARGET   1440  	 

#define CRAWLER_FWD_SPEED      6000      
#define CRAWLER_REV_SPEED      -6000     


extern osMessageQueueId_t ChassisQueueHandle;
extern osMessageQueueId_t StepperQueueHandle;
extern uint16_t Lidar_Get_Min_Distance_In_Range(uint16_t start_angle, uint16_t end_angle);
extern volatile int32_t step_motor_pos;
extern osThreadId_t AutoClimbTaskHandle;
extern void Motor_Contro2(int m1_speed, int m2_speed, int m3_speed, int m4_speed);

static ChassisMsg_t msg_stop  = {0, 0, 0};
static ChassisMsg_t msg_fwd   = {50, 0, 0};   
static ChassisMsg_t msg_rev   = {-50, 0, 0};  
static ChassisMsg_t msg_left  = {2, -80, 0};  
static ChassisMsg_t msg_right = {2, 80, 0};   
static int stepper_cmd = 0;

extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern float IMU_Get_Yaw(void); 

void START_SWEEPER(void)
{
//    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);    
//    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 10000);
//    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 30000);
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET); 
//    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 30000);
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); 
}

void STOP_SWEEPER(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);    
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);   
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);   
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); 
}

void AutoClimb_Process(void)
{
      printf("\r\n========================================\r\n");
      printf("🤖 [自动模式] 启动新一轮任务循环！\r\n");
      printf("========================================\r\n");

      // ==========================================================
      // 动作 0：麦轮直行寻找台阶起点
      // ==========================================================
      printf("▶ 动作 0: 麦轮开始直行，寻找台阶起点...\r\n");
      
      extern UART_HandleTypeDef huart5;
      extern uint8_t ultra_rx_buf[];
      HAL_UART_Abort(&huart5); 
      huart5.gState = HAL_UART_STATE_READY;  
      huart5.RxState = HAL_UART_STATE_READY; 
      HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16);

      osMessageQueuePut(ChassisQueueHandle, &msg_fwd, 0, 0); 
      
      int seek_lock_counter = 0; 
      while (1) {
          uint8_t trig_cmd[1] = {0xA0}; 
          HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
          osDelay(60); 
          
          int dist = Ultrasonic_Get_Distance();
          
          if (dist > 0) {
              seek_lock_counter = 0;
              
              if (dist <= ULTRA_STOP_DIST) {
                  printf("✅ 触达台阶起点(%d mm)！停止直行，准备启动升降机构！\r\n", dist);
                  break; 
              }
          } else {
              seek_lock_counter++;
              if (seek_lock_counter >= 5) {
                  printf("🚨 [安全警报] 寻迹时超声波失联！强制刹停！\r\n");
                  osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
                  osDelay(200);
                  extern int robot_mode;
                  robot_mode = 0; 
                  osThreadSuspend(AutoClimbTaskHandle); 
              }
          }
      }
      
      osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
      osDelay(500); 

      bool is_platform_detected = false;
            
      // ==========================================================
      // 动作 1：步进电机上升，【同时实时侦测超声波】
      // ==========================================================
      printf("▶ 动作 1: 步进上升并侦测地形...\r\n");
      stepper_cmd = 80; 
      // 🌟 核心修复 1：回归原版，只在循环外发一次指令！
      osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
      
      while (step_motor_pos < AUTO_TARGET_UP) { 
          uint8_t trig_cmd[1] = {0xA0}; 
          HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
          osDelay(60); 
          
          int current_dist = Ultrasonic_Get_Distance();
          printf("🔄 [步进上升中] 坐标: %ld | 前方测距: %d mm\r\n", step_motor_pos, current_dist);
          
          if (current_dist > ULTRA_PLATFORM_DETECT) {
              printf("🚩 检测到楼梯平台！(测距 %d mm > %d mm)\r\n", current_dist, ULTRA_PLATFORM_DETECT);
              is_platform_detected = true;
              break; 
          }
      }

      // ==========================================================
      // 🌟 分支 A：进入楼梯平台 S 型清扫逻辑
      // ==========================================================
      if (is_platform_detected) {
          
          printf("\r\n[平台清扫] 阶段 1: 确保步进电机已升至设定最高处...\r\n");
          stepper_cmd = 150;
          // 🌟 核心修复 2：回归原版，循环外发送！
          osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
          
          while (step_motor_pos < AUTO_TARGET_UP) { 
              osDelay(100); 
          } 
          extern void Safe_Stepper_Stop(void);
          Safe_Stepper_Stop();

          printf("\r\n[平台清扫] 阶段 2: 履带持续前进，直到后轮登上最后一个台阶...\r\n");
          
          int final_climb_speed = 12000; 

          while (1) {
              Motor_Contro2(final_climb_speed, final_climb_speed, final_climb_speed, final_climb_speed); 
              osDelay(50); 
              int dist = Lidar_Get_Min_Distance_In_Range(358, 2); 
              
              printf("   -> 登台阶测距(雷达): %d mm (目标: <= %d mm)\r\n", dist, LIDAR_LAST_STEP_DIST);
              
              if (dist > 0 && dist <= LIDAR_LAST_STEP_DIST) {
                  printf("✅ 后轮已成功登上台阶！(当前雷达距离: %d mm)\r\n", dist);
                  break; 
              }
          }
          Motor_Contro2(0, 0, 0, 0); 
          osDelay(500);

          // =========================================================
          printf("\r\n[平台清扫] 阶段 3: 下降到 %d 位置，麦轮着地...\r\n", AUTO_TARGET_DOWN);
          
          stepper_cmd = -40; 
          // 🌟 核心修复 3：回归原版，循环外发送！
          osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
          
          while (step_motor_pos > AUTO_TARGET_DOWN) { 
              printf("🔄 [步进监控-下降] 当前位置: %ld / 目标: %d\r\n", step_motor_pos, AUTO_TARGET_DOWN);
              osDelay(100); 
          }
          
          Safe_Stepper_Stop();
          osDelay(300); 
          // =========================================================
                    
          printf("\r\n[平台清扫] 阶段 3.5: 旋转前微调，麦轮直行驶离边缘至安全距离...\r\n");
          
          osMessageQueuePut(ChassisQueueHandle, &msg_fwd, 0, 0); 
          
          while (1) {
              osDelay(50); 
              int dist = Lidar_Get_Min_Distance_In_Range(358, 2);
              printf("   -> 旋转前直行测距(雷达): %d mm (目标: <= %d mm)\r\n", dist, LIDAR_PRE_TURN_TARGET);
              
              if (dist > 0 && dist <= LIDAR_PRE_TURN_TARGET) {
                  printf("✅ 已到达安全旋转腹地！(当前雷达距离: %d mm)\r\n", dist);
                  break; 
              }
          }
          
          osMessageQueuePut(ChassisQueueHandle, &msg_fwd, 0, 0); 
          osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
          osDelay(800);
          
          // =========================================================
          // 🌟 阶段 4：利用新 IMU 绝对角度精准左转 90 度
          // =========================================================
          printf("\r\n🔄 [平台清扫] 阶段 4: 逆时针旋转 90 度，调整机头朝向！\r\n");

          float start_yaw = IMU_Get_Yaw();          
          float target_yaw = start_yaw + 90.0f;     

          if (target_yaw > 180.0f) target_yaw -= 360.0f;
          
          float Kp = 1.2f;  
          
          while (1) {
              float current_yaw = IMU_Get_Yaw();
              float err = target_yaw - current_yaw;

              if (err > 180.0f) err -= 360.0f;
              if (err < -180.0f) err += 360.0f;

              int w_speed = (int)(Kp * err); 

              if (abs((int)err) <= 1) { 
                  printf("✅ 逆时针 90 度掉头精准完成！最终累计角度: %.2f\r\n", current_yaw);
                  break;
              }

              if (w_speed > 60) w_speed = 60;   
              if (w_speed < -60) w_speed = -60;
              if (w_speed >= 0 && w_speed < 12) w_speed = 12; 
              if (w_speed <= 0 && w_speed > -12) w_speed = -12;

              ChassisMsg_t msg_turn = {0, 0, w_speed};
              osMessageQueuePut(ChassisQueueHandle, &msg_turn, 0, 0);
              
              osDelay(20); 
          }
          
          osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
          osDelay(800); 
                    
          printf("\r\n✨ [平台清扫] 阶段 5: 开始雷达侧向闭环 S 型清扫！\r\n");
          int shift_count = 0; 
          extern int right_min;
          extern int left_min;
          
          while (shift_count < MAX_SHIFT_TIMES) {
              printf("   ▶ S型 (第%d次): 麦轮右移...\r\n", shift_count + 1);
              START_SWEEPER(); 
              osMessageQueuePut(ChassisQueueHandle, &msg_right, 0, 0); 
              while (1) {
                  osDelay(50);
                  if (right_min > 0 && right_min <= LIDAR_RIGHT_WALL_MIN_DIST) break; 
              }
              osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
              osDelay(300);
              STOP_SWEEPER(); 
              
              printf("   ▶ S型: 麦轮精准前进换轨 %d mm...\r\n", ROBOT_FWD_STEP_DIST);
              int start_front_dist = Lidar_Get_Min_Distance_In_Range(358, 2); 
              int target_front_dist = start_front_dist - ROBOT_FWD_STEP_DIST; 
              
              if (target_front_dist < LIDAR_FRONT_WALL_MIN_DIST) {
                  target_front_dist = LIDAR_FRONT_WALL_MIN_DIST;
              }

              if (start_front_dist > 0 && start_front_dist < 4000) {
                  osMessageQueuePut(ChassisQueueHandle, &msg_fwd, 0, 0);
                  int timeout_count = 0;
                  while (1) {
                      osDelay(50);
                      int current_front_dist = Lidar_Get_Min_Distance_In_Range(358, 2);
                      if (current_front_dist > 0 && current_front_dist <= target_front_dist) break;
                      if (++timeout_count > 80) break; 
                  }
                  osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
                  osDelay(300);
              }
              shift_count++;
              
              if (shift_count >= MAX_SHIFT_TIMES) break;

              printf("   ▶ S型 (第%d次): 麦轮左移退回边缘...\r\n", shift_count + 1);
              osMessageQueuePut(ChassisQueueHandle, &msg_left, 0, 0); 
              START_SWEEPER(); 
              while (1) {
                  osDelay(50);
                  if (right_min > 0 && right_min >= LIDAR_RIGHT_PLATFORM_LIMIT) break; 
              }
              osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
              osDelay(300);
              STOP_SWEEPER(); 
              
              printf("   ▶ S型: 麦轮精准前进换轨 %d mm...\r\n", ROBOT_FWD_STEP_DIST);
              start_front_dist = Lidar_Get_Min_Distance_In_Range(358, 2); 
              target_front_dist = start_front_dist - ROBOT_FWD_STEP_DIST;
              
              if (target_front_dist < LIDAR_FRONT_WALL_MIN_DIST) {
                  target_front_dist = LIDAR_FRONT_WALL_MIN_DIST;
              }

              if (start_front_dist > 0 && start_front_dist < 4000) {
                  osMessageQueuePut(ChassisQueueHandle, &msg_fwd, 0, 0);
                  int timeout_count = 0;
                  while (1) {
                      osDelay(50);
                      int current_front_dist = Lidar_Get_Min_Distance_In_Range(358, 2);
                      if (current_front_dist > 0 && current_front_dist <= target_front_dist) break;
                      if (++timeout_count > 80) break;
                  }
                  osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
                  osDelay(300);
              }
              shift_count++;
          }

          // =========================================================
          // 🌟 阶段 6：利用新 IMU 绝对角度精准右转 90 度回正
          // =========================================================
          printf("\r\n🔄 [平台清扫结束] 阶段 6: 顺时针旋转 90 度回正...\r\n");
          
          start_yaw = IMU_Get_Yaw();          
          target_yaw = start_yaw - 90.0f;     

          if (target_yaw < -180.0f) target_yaw += 360.0f;

          while (1) {
              float current_yaw = IMU_Get_Yaw();
              float err = target_yaw - current_yaw;

              if (err > 180.0f) err -= 360.0f;
              if (err < -180.0f) err += 360.0f;

              int w_speed = (int)(Kp * err); 

              if (abs((int)err) <= 1) { 
                  printf("✅ 顺时针 90 度回转精准完成！最终角度: %.2f\r\n", current_yaw);
                  break;
              }

              if (w_speed > 60) w_speed = 60;
              if (w_speed < -60) w_speed = -60;
              if (w_speed >= 0 && w_speed < 12) w_speed = 12; 
              if (w_speed <= 0 && w_speed > -12) w_speed = -12;

              ChassisMsg_t msg_turn = {0, 0, w_speed};
              osMessageQueuePut(ChassisQueueHandle, &msg_turn, 0, 0);
              
              osDelay(20); 
          }
          
          osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
          osDelay(800);
          
      }
      
      // ==========================================================
      // 🌟 分支 B：常规台阶攀爬/对齐逻辑
      // ==========================================================
      printf("\r\n[普通台阶] 开始执行常规台阶攀爬及对齐逻辑...\r\n");
      
      extern void Safe_Stepper_Stop(void);
      Safe_Stepper_Stop();
      
      // ==========================================================
      printf("▶ 动作 2: 履带/前轮开始前进(启用防断电软启动)...\r\n");

      HAL_UART_Abort(&huart5); 
      huart5.gState = HAL_UART_STATE_READY;  
      huart5.RxState = HAL_UART_STATE_READY; 
      HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16);

      for(int speed = 4000; speed <= 16000; speed += 4000) {
          Motor_Contro2(speed, speed, speed, speed);
          uint8_t trig_cmd[1] = {0xA0}; 
          HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
          osDelay(60); 
      }
      
      int current_dist = 0;
      int ultra_lock_counter = 0; 
      
      while (1) 
      { 
          if (huart5.RxState != HAL_UART_STATE_BUSY_RX || huart5.gState != HAL_UART_STATE_READY) {
              HAL_UART_Abort(&huart5); 
              huart5.gState = HAL_UART_STATE_READY;  
              huart5.RxState = HAL_UART_STATE_READY; 
              HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16); 
          }
          
          uint8_t trig_cmd[1] = {0xA0}; 
          HAL_UART_Transmit(&huart5, trig_cmd, 1, 50); 
          
          osDelay(60); 

          current_dist = Ultrasonic_Get_Distance();
          printf("🦇 [超声波监控] 当前前方距离: %d mm\r\n", current_dist);
          
          if (current_dist > 0) {
              ultra_lock_counter = 0; 
              if (current_dist <= ULTRA_STEP_FWD_DIST) {
                  printf("✅ 触发避障！距离达标(%d mm)，紧急刹车！\r\n", current_dist);
                  break; 
              }
          }
          else {
              ultra_lock_counter++;
              if (ultra_lock_counter >= 3) {
                  printf("🚨 [安全警报] 传感器失联！盲开危险，立刻紧急停车！\r\n");
                  Motor_Contro2(0, 0, 0, 0);
                  osDelay(200);

                  printf("🔄 正在执行重置 (DeInit/Init)\r\n");
                  extern void MX_UART5_Init(void); 
                  HAL_UART_DeInit(&huart5);        
                  MX_UART5_Init();                 
                  HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16); 
                  
                  printf("✅ 链路重置完成，履带重新软启动前进\r\n");
                  for(int speed = 5000; speed <= 15000; speed += 5000) {
                      Motor_Contro2(speed, speed, speed, speed);
                      HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
                      osDelay(60); 
                  }
                  ultra_lock_counter = 0; 
              }
          }
      }
      Motor_Contro2(0, 0, 0, 0); 
      osDelay(500);
            
      // ==========================================================
      printf("▶ 动作 4: 步进电机下降中\r\n");
      stepper_cmd = -80; 
      // 🌟 核心修复 4：回归原版，循环外发送！
      osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
      
      while (step_motor_pos > AUTO_TARGET_DOWN) { 
          printf("🔄 [步进监控-下降] 当前位置: %ld / 目标: %d\r\n", step_motor_pos, AUTO_TARGET_DOWN);
          osDelay(100); 
      }
      
      Safe_Stepper_Stop();

      // ==========================================================
      printf("▶ 动作 4.5: 麦轮直行贴近下一层台阶，准备左右对中...\r\n");
      
      if (huart5.RxState != HAL_UART_STATE_BUSY_RX || huart5.gState != HAL_UART_STATE_READY) {
          HAL_UART_Abort(&huart5); 
          huart5.gState = HAL_UART_STATE_READY;  
          huart5.RxState = HAL_UART_STATE_READY; 
          HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16); 
      }
      
      osMessageQueuePut(ChassisQueueHandle, &msg_fwd, 0, 0); 
      int post_step_lock_counter = 0; 
      
      while (1) {
          uint8_t trig_cmd[1] = {0xA0}; 
          HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
          osDelay(60); 
          
          int dist = Ultrasonic_Get_Distance();
          printf("🦇 [贴墙寻迹] 当前前方距离: %d mm (目标 <= %d mm)\r\n", dist, ULTRA_STOP_DIST);
          
          if (dist > 0) {
              post_step_lock_counter = 0;
              if (dist <= ULTRA_STOP_DIST) {
                  printf("✅ 触达下一层起跳点(%d mm)！停止直行，开始左右扫平！\r\n", dist);
                  break; 
              }
          } else {
              post_step_lock_counter++;
              if (post_step_lock_counter >= 5) {
                  printf("🚨 [安全警报] 贴墙寻迹时超声波失联！强制刹停！\r\n");
                  osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
                  osDelay(200);
                  extern int robot_mode;
                  robot_mode = 0; 
                  osThreadSuspend(AutoClimbTaskHandle); 
              }
          }
      }
      
      osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
      osDelay(500); 
       
      // ==========================================================
      printf("▶ 动作 5: 麦轮开始左移...\r\n");
      START_SWEEPER(); 
      osMessageQueuePut(ChassisQueueHandle, &msg_left, 0, 0);
      
			
      while (left_min > LIDAR_LEFT_STOP_DIST || left_min <= 0) { 
          printf("🎯 [雷达监控-左侧] 当前距离: %d mm\r\n", left_min);
          osDelay(100); 
      }
      
      printf("✅ 雷达左侧达标(%d mm)，停止左移。\r\n", left_min);
      osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
      osDelay(500); 
      STOP_SWEEPER(); 

      // ==========================================================
      printf("▶ 动作 7: 麦轮开始右移...\r\n");
      START_SWEEPER(); 
      osMessageQueuePut(ChassisQueueHandle, &msg_right, 0, 0);
      
      while (right_min > LIDAR_RIGHT_STOP_DIST || right_min <= 0) { 
          printf("🎯 [雷达监控-右侧] 当前距离: %d mm\r\n", right_min);
          osDelay(100); 
      }
      
      printf("✅ 雷达右侧达标(%d mm)，停止右移。\r\n", right_min);
      osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
      osDelay(500); 
      STOP_SWEEPER(); 

      printf("🎉 单级台阶循环完成！\r\n");
}