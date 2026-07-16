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
#define AUTO_TARGET_UP       2098909   // 步进电机上升目标脉冲数（台阶高度）
#define AUTO_TARGET_DOWN     -10000    // 步进电机下降目标脉冲数（底盘着地）
#define ULTRA_STOP_DIST      50        // 超声波停止直行距离（贴紧台阶）
#define ULTRA_STEP_FWD_DIST  125       // 超声波安全爬坡前进距离 

#define LIDAR_LEFT_STOP_DIST       450   // 避障/对齐：雷达左侧安全停止距离
#define LIDAR_RIGHT_STOP_DIST      500   // 避障/对齐：雷达右侧安全停止距离
#define LIDAR_RIGHT_WALL_MIN_DIST  450   // S型扫平：右侧贴墙最小距离（换轨标志）
#define LIDAR_RIGHT_PLATFORM_LIMIT 1000   // S型扫平：右侧平台边缘极限距离（防跌落）
#define ROBOT_FWD_STEP_DIST        190   // S型扫平：每次换轨前行的精准物理距离 (已按你要求改大至 200)
#define PLATFORM_FRONT_END_DIST    380    // S型扫平终极目标，离前方墙壁 40mm 结束清扫

#define ULTRA_PLATFORM_DETECT  1700      // 核心标志：超声波突变阈值（大于此值判定前方是空旷平台）

#define LIDAR_LAST_STEP_DIST    1834  	 // 核心判定：后轮登顶台阶时，车头雷达距离前方墙壁的距离
#define LIDAR_PRE_TURN_TARGET   1440  	 // 核心安全：旋转前直行，驶离台阶边缘的安全避开距离

#define CRAWLER_FWD_SPEED      6000      // 爬坡履带前进基准速度
#define CRAWLER_REV_SPEED      -6000     // 爬坡履带后退基准速度


extern osMessageQueueId_t ChassisQueueHandle;
extern osMessageQueueId_t StepperQueueHandle;
extern uint16_t Lidar_Get_Min_Distance_In_Range(uint16_t start_angle, uint16_t end_angle);
extern volatile int32_t step_motor_pos;
extern osThreadId_t AutoClimbTaskHandle;
extern void Motor_Contro2(int m1_speed, int m2_speed, int m3_speed, int m4_speed);

static ChassisMsg_t msg_stop  = {0, 0, 0};
static ChassisMsg_t msg_fwd   = {50, 0, 0};   
static ChassisMsg_t msg_rev   = {-50, 0, 0};  
static ChassisMsg_t msg_left  = {5, -90, 0};  
static ChassisMsg_t msg_right = {5, 90, 0};   
static int stepper_cmd = 0;

extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern float IMU_Get_Yaw(void); 
extern UART_HandleTypeDef huart2;  // 🌟 全局声明雷达串口，方便随时施放复活甲

void START1_SWEEPER(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);    
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 10000);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 30000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);   
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET); 
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 30000);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);   
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); 
}
void START2_SWEEPER(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);    
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 10000);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 30000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET); 
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 30000);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); 
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

      osMessageQueuePut(ChassisQueueHandle, &msg_fwd, 0, 0); 
      
      int seek_lock_counter = 0; 
      while (1) {
          if (huart5.RxState != HAL_UART_STATE_BUSY_RX || huart5.gState != HAL_UART_STATE_READY) {
              HAL_UART_Abort(&huart5); 
              huart5.gState = HAL_UART_STATE_READY;  
              huart5.RxState = HAL_UART_STATE_READY; 
              HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16); 
          }

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
      osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
      
      while (step_motor_pos < AUTO_TARGET_UP) { 
          if (huart5.RxState != HAL_UART_STATE_BUSY_RX || huart5.gState != HAL_UART_STATE_READY) {
              HAL_UART_Abort(&huart5); 
              huart5.gState = HAL_UART_STATE_READY;  
              huart5.RxState = HAL_UART_STATE_READY; 
              HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16); 
          }

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
          osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
          
          while (step_motor_pos < AUTO_TARGET_UP) { 
              osDelay(100); 
          } 
          extern void Safe_Stepper_Stop(void);
          Safe_Stepper_Stop();

          // =================================================================
          // 🌟 平台清扫 阶段2: 履带持续前进 (带 5 帧防抖滤波，绝不提前触发！)
          // =================================================================
          printf("\r\n[平台清扫] 阶段 2: 履带持续前进，直到后轮登上最后一个台阶...\r\n");
          
          int final_climb_speed = 12000; 
          int climb_timeout_cnt = 0;   
          int valid_reach_frames = 0;  

          while (1) {
              Motor_Contro2(final_climb_speed, final_climb_speed, final_climb_speed, final_climb_speed); 
              osDelay(50); 
              int dist = Lidar_Get_Min_Distance_In_Range(358, 2); 
              
              if (dist > 100 && dist <= LIDAR_LAST_STEP_DIST) {
                  valid_reach_frames++; 
                  printf("   -> 登台阶测距(雷达): %d mm (目标: <= %d mm) | 🎯 达标防抖计数: %d/5 | 计时: %d ms\r\n", 
                          dist, LIDAR_LAST_STEP_DIST, valid_reach_frames, climb_timeout_cnt * 50);
              } else {
                  if (valid_reach_frames > 0) {
                      printf("   ⚠️ [闪变拦截] 距离突然回弹到 %d mm！判定刚才为晃动噪声，计数器清零！\r\n", dist);
                  }
                  valid_reach_frames = 0; 
              }
              
              if (valid_reach_frames >= 5) {
                  printf("✅ [确认登顶] 经历连续 5 帧严格验证，后轮确已登上台阶！最终雷达距离: %d mm\r\n", dist);
                  break; 
              }

              if (++climb_timeout_cnt >= 60) { 
                  printf("⚠️ [超时保护触发] 爬坡已达 4 秒上限！判定后轮已登顶，强行进入下一阶段！\r\n");
                  break;
              }
          }
          Motor_Contro2(0, 0, 0, 0); 
          osDelay(500);

          // =========================================================
          printf("\r\n[平台清扫] 阶段 3: 下降到 %d 位置，麦轮着地...\r\n", AUTO_TARGET_DOWN);
          
          stepper_cmd = -70; 
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
          
          int retry_cnt = 0;
          int last_valid_dist = 1600; 
          
          while (1) {
              osDelay(50); 
              
              if (++retry_cnt >= 10) { 
                  __HAL_UART_CLEAR_OREFLAG(&huart2);
                  __HAL_UART_CLEAR_FEFLAG(&huart2);
                  retry_cnt = 0;
              }
              
              if (huart5.RxState != HAL_UART_STATE_BUSY_RX || huart5.gState != HAL_UART_STATE_READY) {
                  HAL_UART_Abort(&huart5); 
                  huart5.gState = HAL_UART_STATE_READY;  
                  huart5.RxState = HAL_UART_STATE_READY; 
                  HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16); 
              }
              uint8_t trig_cmd[1] = {0xA0}; 
              HAL_UART_Transmit(&huart5, trig_cmd, 1, 50);
              int ultra_dist = Ultrasonic_Get_Distance();
              
              int raw_lidar_dist = Lidar_Get_Min_Distance_In_Range(358, 2);
              int final_filtered_dist = raw_lidar_dist;
              
              bool is_lidar_fault = false;
              
              if (ultra_dist > 0 && ultra_dist < 1800 && raw_lidar_dist > 2500) {
                  is_lidar_fault = true;
                  printf("⚠️ [防撞预警] 雷达读数突变(5000+)，超声波正常(%d mm)，判定雷达出现幻觉！\r\n", ultra_dist);
              }
              
              if (raw_lidar_dist > 0 && abs(raw_lidar_dist - last_valid_dist) > 300) {
                  is_lidar_fault = true;
                  printf("⚠️ [阶跃滤波] 雷达数据突变(从 %d -> %d)，判定为乱数据，已拦截！\r\n", last_valid_dist, raw_lidar_dist);
              }
              
              if (is_lidar_fault || raw_lidar_dist <= 0) {
                  final_filtered_dist = last_valid_dist; 
              } else {
                  last_valid_dist = raw_lidar_dist;      
              }
              
              printf("   -> 旋转前直行测距(雷达滤波后): %d mm (目标: <= %d mm) | 辅测超声波: %d mm\r\n", 
                      final_filtered_dist, LIDAR_PRE_TURN_TARGET, ultra_dist);
              
              if (final_filtered_dist > 0 && final_filtered_dist <= LIDAR_PRE_TURN_TARGET) {
                  printf("✅ 已安全到达旋转腹地！(当前雷达最终滤波距离: %d mm)\r\n", final_filtered_dist);
                  break; 
              }
          }
          
          osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
          osDelay(800);
          
          // =========================================================
          // 🌟 阶段 4：逆时针旋转 90 度
          // =========================================================
          printf("\r\n🔄 [平台清扫] 阶段 4: 逆时针旋转 90 度，调整机头朝向！\r\n");

          float start_yaw = IMU_Get_Yaw() * 57.29578f;   
          float target_yaw = start_yaw + 90.0f;     

          if (target_yaw > 180.0f) target_yaw -= 360.0f;

          float Kp = 1.3f;  
          
          while (1) {
              float current_yaw = IMU_Get_Yaw() * 57.29578f;
              float err = target_yaw - current_yaw;

              if (err > 180.0f) err -= 360.0f;
              if (err < -180.0f) err += 360.0f;

              int w_speed = (int)(Kp * err); 

              if (abs((int)err) <= 2) { 
                  printf("✅ 逆时针 90 度掉头精准完成！最终角度: %.2f°\r\n", current_yaw);
                  break;
              }

              if (w_speed > 45) w_speed = 45;     
              if (w_speed < -45) w_speed = -45;
              
              if (abs((int)err) > 5) {
                  if (w_speed >= 0 && w_speed < 14) w_speed = 14; 
                  if (w_speed <= 0 && w_speed > -14) w_speed = -14;
              } else {
                  if (w_speed >= 0 && w_speed < 8) w_speed = 8; 
                  if (w_speed <= 0 && w_speed > -8) w_speed = -8;
              }

              ChassisMsg_t msg_turn = {0, 0, -w_speed};
              osMessageQueuePut(ChassisQueueHandle, &msg_turn, 0, 0);
              
              osDelay(20); 
          }
          
          osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
          osDelay(800);
          
					// 🌟🌟🌟 你的绝招：记住此时最完美的车身角度！ 🌟🌟🌟
          float platform_locked_yaw = IMU_Get_Yaw() * 57.29578f;
          printf("🎯 [航向锁死] 阶段 4 旋转完毕，平台清扫标准角度已锁死为: %.2f°\r\n", platform_locked_yaw);

          // =========================================================
          // 🌟 阶段 5: 开始雷达侧向闭环 S 型清扫
          // =========================================================
          printf("\r\n✨ [平台清扫] 阶段 5: 启动动态自适应 S 型清扫（航向锁定 + 实时防撞）！\r\n");
          extern int right_min;
          extern int left_min;
          
          bool s_shape_finished = false; // 队友新增：全局跳出标志
          float Kp_lateral = 1.8f;       // 你的新增：陀螺仪平移纠偏力度

          while (!s_shape_finished) { 
              
              int check_dist = Lidar_Get_Min_Distance_In_Range(358, 2);
              if (check_dist > 0 && check_dist <= PLATFORM_FRONT_END_DIST) {
                  printf("✅ 前方距离仅剩 %d mm，平台已清扫到底！\r\n", check_dist);
                  break;
              }

              // --- 动作 1：麦轮右移 (融合陀螺仪纠偏) ---
              printf("   ▶ S型: 麦轮右移 (全局航向锁定中)...\r\n");
              START2_SWEEPER(); 
              int side_retry1 = 0; 
              while (1) {
                  // 🌟 你的逻辑：计算偏航误差并动态生成纠偏指令
                  float current_yaw = IMU_Get_Yaw() * 57.29578f;
                  float err = platform_locked_yaw - current_yaw;
                  if (err > 180.0f) err -= 360.0f; 
                  if (err < -180.0f) err += 360.0f;
                  
                  int comp_w = (int)(Kp_lateral * err);
                  if (comp_w > 30) comp_w = 30; if (comp_w < -30) comp_w = -30;
                  
                  // 发送复合指令 (右移 + 纠偏)
                  ChassisMsg_t move_cmd = {msg_right.x, msg_right.y, -comp_w};
                  osMessageQueuePut(ChassisQueueHandle, &move_cmd, 0, 0);

                  osDelay(50);
                  
                  if (++side_retry1 % 10 == 0) {
                      __HAL_UART_CLEAR_OREFLAG(&huart2);
                      __HAL_UART_CLEAR_FEFLAG(&huart2);
                  }
                  
                  // 🌟 队友逻辑：实时读取前方距离
                  check_dist = Lidar_Get_Min_Distance_In_Range(358, 2);
                  printf("   -> [侧移监控] 右移... right_min=%d mm | 前方=%d mm | 角度偏离: %.2f°\r\n", right_min, check_dist, err);
                  
                  // 🚨 队友逻辑：侧移撞墙紧急打断
                  if (check_dist > 0 && check_dist <= PLATFORM_FRONT_END_DIST) {
                      printf("🚨 [紧急中断] 右移时发现前方距墙仅 %d mm，强制结束 S 型循环！\r\n", check_dist);
                      s_shape_finished = true;
                      break; 
                  }

                  if (right_min > 0 && right_min <= LIDAR_RIGHT_WALL_MIN_DIST) break; 
              }
              osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
              osDelay(300);
              STOP_SWEEPER(); 
              
              if (s_shape_finished) break; 

              // --- 动作 2：麦轮精准换轨前进 (融合陀螺仪纠偏) ---
              printf("   ▶ S型: 麦轮精准前进换轨 %d mm...\r\n", ROBOT_FWD_STEP_DIST);
              int start_front_dist = Lidar_Get_Min_Distance_In_Range(358, 2); 
              int target_front_dist = start_front_dist - ROBOT_FWD_STEP_DIST; 
              
              if (target_front_dist <= PLATFORM_FRONT_END_DIST) { 
                  target_front_dist = PLATFORM_FRONT_END_DIST;  
                  s_shape_finished = true;                     
              }

              if (start_front_dist > PLATFORM_FRONT_END_DIST && start_front_dist < 4000) {
                  int timeout_count = 0;
                  while (1) {
                      // 🌟 你的逻辑：换轨前进同样加入纠偏
                      float current_yaw = IMU_Get_Yaw() * 57.29578f;
                      float err = platform_locked_yaw - current_yaw;
                      if (err > 180.0f) err -= 360.0f; 
                      if (err < -180.0f) err += 360.0f;
                      int comp_w = (int)(Kp_lateral * err);
                      if (comp_w > 30) comp_w = 30; if (comp_w < -30) comp_w = -30;
                      
                      // 发送复合指令 (前进 + 纠偏)
                      ChassisMsg_t move_cmd = {msg_fwd.x, msg_fwd.y, -comp_w};
                      osMessageQueuePut(ChassisQueueHandle, &move_cmd, 0, 0);

                      osDelay(50);
                      if (++timeout_count % 10 == 0) {
                          __HAL_UART_CLEAR_OREFLAG(&huart2);
                          __HAL_UART_CLEAR_FEFLAG(&huart2);
                      }
                      int current_front_dist = Lidar_Get_Min_Distance_In_Range(358, 2);
                      
                      printf("   -> [换轨监控] 前进中... 前方距离=%d mm (目标<=%d) | 角度偏离: %.2f°\r\n", current_front_dist, target_front_dist, err);
                      
                      if (current_front_dist > 0 && current_front_dist <= target_front_dist) break;
                      if (timeout_count > 80) break; 
                  }
                  osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
                  osDelay(300);
              }
              
              if (s_shape_finished) {
                  printf("✅ 换轨前进已抵达极限，结束S型循环！\r\n");
                  break;
              }

              // --- 动作 3：麦轮左移退回边缘 (融合陀螺仪纠偏) ---
              printf("   ▶ S型: 麦轮左移退回边缘 (全局航向锁定中)...\r\n");
              osMessageQueuePut(ChassisQueueHandle, &msg_left, 0, 0); 
              START1_SWEEPER(); 
              int side_retry2 = 0; 
              while (1) {
                  // 🌟 你的逻辑：左移纠偏
                  float current_yaw = IMU_Get_Yaw() * 57.29578f;
                  float err = platform_locked_yaw - current_yaw;
                  if (err > 180.0f) err -= 360.0f; 
                  if (err < -180.0f) err += 360.0f;
                  int comp_w = (int)(Kp_lateral * err);
                  if (comp_w > 30) comp_w = 30; if (comp_w < -30) comp_w = -30;
                  
                  // 发送复合指令 (左移 + 纠偏)
                  ChassisMsg_t move_cmd = {msg_left.x, msg_left.y, -comp_w};
                  osMessageQueuePut(ChassisQueueHandle, &move_cmd, 0, 0);

                  osDelay(50);
                  if (++side_retry2 % 10 == 0) {
                      __HAL_UART_CLEAR_OREFLAG(&huart2);
                      __HAL_UART_CLEAR_FEFLAG(&huart2);
                  }
                  
                  // 🌟 队友逻辑：左移时同样监视前方
                  check_dist = Lidar_Get_Min_Distance_In_Range(358, 2);
                  printf("   -> [侧移监控] 左移... right_min=%d mm | 前方=%d mm | 角度偏离: %.2f°\r\n", right_min, check_dist, err);
                  
                  // 🚨 队友逻辑：侧移撞墙紧急打断
                  if (check_dist > 0 && check_dist <= PLATFORM_FRONT_END_DIST) {
                      printf("🚨 [紧急中断] 左移时发现前方距墙仅 %d mm，强制结束 S 型循环！\r\n", check_dist);
                      s_shape_finished = true;
                      break; 
                  }

                  if (right_min > 0 && right_min >= LIDAR_RIGHT_PLATFORM_LIMIT) break; 
              }
              osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
              osDelay(300);
              STOP_SWEEPER(); 
              
              if (s_shape_finished) break; 
              
              // --- 动作 4：麦轮再次精准换轨前进 (融合陀螺仪纠偏) ---
              printf("   ▶ S型: 麦轮再次精准前进换轨 %d mm...\r\n", ROBOT_FWD_STEP_DIST);
              start_front_dist = Lidar_Get_Min_Distance_In_Range(358, 2); 
              target_front_dist = start_front_dist - ROBOT_FWD_STEP_DIST;
              
              if (target_front_dist <= PLATFORM_FRONT_END_DIST) {
                  target_front_dist = PLATFORM_FRONT_END_DIST;
                  s_shape_finished = true;
              }

              if (start_front_dist > PLATFORM_FRONT_END_DIST && start_front_dist < 4000) {
                  int timeout_count = 0;
                  while (1) {
                      // 🌟 你的逻辑：第二次换轨同样加入纠偏
                      float current_yaw = IMU_Get_Yaw() * 57.29578f;
                      float err = platform_locked_yaw - current_yaw;
                      if (err > 180.0f) err -= 360.0f; 
                      if (err < -180.0f) err += 360.0f;
                      int comp_w = (int)(Kp_lateral * err);
                      if (comp_w > 30) comp_w = 30; if (comp_w < -30) comp_w = -30;
                      
                      // 发送复合指令 (前进 + 纠偏)
                      ChassisMsg_t move_cmd = {msg_fwd.x, msg_fwd.y, -comp_w};
                      osMessageQueuePut(ChassisQueueHandle, &move_cmd, 0, 0);

                      osDelay(50);
                      if (++timeout_count % 10 == 0) {
                          __HAL_UART_CLEAR_OREFLAG(&huart2);
                          __HAL_UART_CLEAR_FEFLAG(&huart2);
                      }
                      int current_front_dist = Lidar_Get_Min_Distance_In_Range(358, 2);
                      
                      printf("   -> [换轨监控] 前进中... 前方距离=%d mm (目标<=%d) | 角度偏离: %.2f°\r\n", current_front_dist, target_front_dist, err);
                      
                      if (current_front_dist > 0 && current_front_dist <= target_front_dist) break;
                      if (timeout_count > 80) break;
                  }
                  osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
                  osDelay(300);
              }
              
              if (s_shape_finished) {
                  printf("✅ 换轨前进已抵达极限，结束S型循环！\r\n");
                  break;
              }
          }
					// =========================================================
          // 🌟 阶段 6：再次逆时针旋转 90 度 (方向修正为逆时针，与阶段 4 一致！)
          // =========================================================
          printf("\r\n🔄 [平台清扫结束] 阶段 6: 再次逆时针旋转 90 度，准备前往下一层...\r\n");
          
          start_yaw = IMU_Get_Yaw() * 57.29578f;   
          
          // 🌟 重新改回与阶段 4 一致的 +90.0f！实现累计 180 度掉头
          target_yaw = start_yaw + 90.0f;     

          if (target_yaw > 180.0f) target_yaw -= 360.0f;

          while (1) {
              float current_yaw = IMU_Get_Yaw() * 57.29578f;
              float err = target_yaw - current_yaw;

              if (err > 180.0f) err -= 360.0f;
              if (err < -180.0f) err += 360.0f;

              int w_speed = (int)(Kp * err); 

              if (abs((int)err) <= 2) { 
                  printf("✅ 再次逆时针 90 度精密掉头完成！累计调转 180 度！最终角度: %.2f°\r\n", current_yaw);
                  break;
              }

              if (w_speed > 45) w_speed = 45;     
              if (w_speed < -45) w_speed = -45;
              
              if (abs((int)err) > 5) {
                  if (w_speed >= 0 && w_speed < 14) w_speed = 14; 
                  if (w_speed <= 0 && w_speed > -14) w_speed = -14;
              } else {
                  if (w_speed >= 0 && w_speed < 8) w_speed = 8; 
                  if (w_speed <= 0 && w_speed > -8) w_speed = -8;
              }

              // 负号喂给底盘以维持逆时针旋转
              ChassisMsg_t msg_turn = {0, 0, -w_speed};
              osMessageQueuePut(ChassisQueueHandle, &msg_turn, 0, 0);
              
              osDelay(20); 
          }
          
          osMessageQueuePut(ChassisQueueHandle, &msg_stop, 0, 0);
          osDelay(800);
          
          printf("🎉 [平台任务结束] 准备重新寻找下行台阶起点！\r\n");
          return;
      }
					
      // ==========================================================
      // 🌟 分支 B：常规台阶攀爬/对齐逻辑
      // ==========================================================
      printf("\r\n[普通台阶] 开始执行常规台阶攀爬及对齐逻辑...\r\n");
      
      extern void Safe_Stepper_Stop(void);
      Safe_Stepper_Stop();
      
      // ==========================================================
      printf("▶ 动作 2: 履带/前轮开始前进(启用防断电软启动)...\r\n");

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
      osMessageQueuePut(StepperQueueHandle, &stepper_cmd, 0, 0);
      
      while (step_motor_pos > AUTO_TARGET_DOWN) { 
          printf("🔄 [步进监控-下降] 当前位置: %ld / 目标: %d\r\n", step_motor_pos, AUTO_TARGET_DOWN);
          osDelay(100); 
      }
      
      Safe_Stepper_Stop();

      // ==========================================================
      printf("▶ 动作 4.5: 麦轮直行贴近下一层台阶，准备左右对中...\r\n");
      
      osMessageQueuePut(ChassisQueueHandle, &msg_fwd, 0, 0); 
      int post_step_lock_counter = 0; 
      
      while (1) {
          if (huart5.RxState != HAL_UART_STATE_BUSY_RX || huart5.gState != HAL_UART_STATE_READY) {
              HAL_UART_Abort(&huart5); 
              huart5.gState = HAL_UART_STATE_READY;  
              huart5.RxState = HAL_UART_STATE_READY; 
              HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16); 
          }

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
      START1_SWEEPER(); 
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
      START2_SWEEPER(); 
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