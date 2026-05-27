/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile int32_t step_motor_pos = 0; // 必须加 volatile！
volatile uint32_t motor_update_count = 0;

/* USER CODE END PV */

// 在 main.c 的 PRIVATE VARIABLES 区
#if defined ( __ICCARM__ )
#pragma location = 0x30000000 // 或者其他非 Cacheable 的 SRAM 区域
uint8_t rxCmd[128];
#else
// 使用 GCC 定义，确保在 linker script 中这块内存被标记为 Non-Cacheable
uint8_t rxCmd[128] __attribute__((section(".noncacheable"))) __attribute__((aligned(32))); 
#endif



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
// 定义接收缓冲区和标志
uint8_t rx_buffer[11]; // 接收11字节的一帧数据
uint8_t rx_index = 0;
uint8_t data_ready = 0; // 一帧数据接收完成标志
uint8_t aRxBuffer; 


// ================= 新增：蓝牙与模式控制变量 =================
uint8_t bt_rx_buf[64];           // 蓝牙接收缓存
uint32_t last_bt_heartbeat = 0;  // 蓝牙防飞车“看门狗”时间戳
int robot_mode = 0;              // 机器人当前模式：0=手动摇杆， 1=全自动爬楼



// 启动第一次中断接收
//步进电机变量
#define CMD_LEN 128          // 定义串口接收缓冲区长度        
uint8_t rxCmd[CMD_LEN] __attribute__((aligned(32))) = {0}; // 🌟 修复：H7必须加32字节对齐！
bool rxFrameFlag = false;     // 接收完成标志位
//雷达变量
extern uint8_t Lidar_RxBuf[640]; 
extern uint16_t Lidar_WriteIndex;
extern uint16_t Lidar_ReadIndex;
extern void Lidar_ParseSingleFrame(uint8_t *frame);
//编码器数据
int16_t pulse_motor1 = 0; // 对应 TIM3
int16_t pulse_motor2 = 0; // 对应 TIM4
int16_t pulse_motor3 = 0; // 对应 TIM5
int16_t pulse_motor4 = 0; // 对应 TIM8
// 用于存储解析后的目标数据
uint16_t obj_x, obj_y, obj_w, obj_h;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



Tuoluo_Data_t my_robot_imu; // 声明一个结构体变量，用来接数据



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	
	
		// 1. 先用比较慢的波特率发送断开连接的 AT 指令
		char *disconnect_cmd = "AT+DISC\r\n"; // 具体指令请看你的蓝牙模块手册
		HAL_UART_Transmit(&huart4, (uint8_t *)disconnect_cmd, strlen(disconnect_cmd), 100);

		// 2. 等待蓝牙模块处理完断开动作
		HAL_Delay(500); 

		// 3. 然后再开启正常的中断接收
		HAL_UARTEx_ReceiveToIdle_IT(&huart4, bt_rx_buf, 64);
	

		// ... 其他代码 ...
		


	
		HAL_TIM_PWM_Init(&htim1);
		HAL_TIM_PWM_Init(&htim2);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // M1: PE9
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // M1: PE11
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // M1: PE13
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // M2: PE14
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // M2: PA5
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // M2: PB3
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // M2: PA2
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // M2: PA3
		HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1);
		HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
		
    // 2. 【新增】以中断模式启动 TIM6，秒表开始计时！
    HAL_TIM_Base_Start_IT(&htim6);
		
		// 👇 补充 2：开启手机蓝牙 UART4 的空闲中断接收！（没有这句摇杆绝对没反应）
		HAL_UARTEx_ReceiveToIdle_IT(&huart4, bt_rx_buf, 64);
		
		// 👇 补充 1：超声波初始化
		Ultrasonic_UART_Init();	
		
		// 🌟 核心修复：强行开启超声波 UART5 的 IT 空闲中断接收！(防止没开启导致一直是 -1)
		
		// 🌟 核心修复：先打断所有可能在 Init 里卡住的接收状态，清空追尾标志，再重新开启！
		extern uint8_t ultra_rx_buf[];
		HAL_UART_AbortReceive(&huart5);    // 强制打断卡死状态
		__HAL_UART_CLEAR_OREFLAG(&huart5); // 清除追尾标志
		
		if (HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16) != HAL_OK) {
				// 把 ErrorCode 打印出来，万一再错我们直接看代码抓鬼
				printf("🚨 超声波串口监听开启失败！错误码: %d\n", huart5.ErrorCode);
		} else {
				printf("✅ 超声波串口 IT 监听开启成功！\r\n");
		}
				
				
		//雷达
		Lidar_Init();
		
		
		// 这里的指令必须在所有外设初始化完成之后
		HAL_Delay(500);
		Emm_V5_Reset_CurPos_To_Zero(1);
		HAL_Delay(50);

		// 🌟 核心：直接使用 HAL_UART_AbortReceive，强行踢开任何抢占者
		HAL_UART_AbortReceive(&huart3); 
		// 🌟 核心 1：将当前丝杠所在的物理位置，强行定义为“绝对 0 点”
		Emm_V5_Reset_CurPos_To_Zero(1); 
		
		// 🚨 修复 1：必须加一点延时！让 DMA 快递员把第一句话送完，防止连环追尾丢包！
    HAL_Delay(50);
				
		extern uint8_t rxCmd[];
		// 使用 IT 模式代替 DMA 进行初始化，看看是否还会报错
		// 如果 IT 模式能通，那问题 100% 出在 DMA Stream 的分配冲突上
		Emm_V5_Auto_Return_Sys_Params_Timed(1, S_CPOS, 20);
		
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart3,rxCmd,128);
		
		__HAL_UART_CLEAR_OREFLAG(&huart3); // 🌟 核心修复 1：强行清除硬件里可能因为“追尾”产生的 ORE 错误标志！
		
		if (HAL_UARTEx_ReceiveToIdle_IT(&huart3, rxCmd, 128) != HAL_OK) {
				printf("🚨 依然报错，错误码: %d\n", huart3.ErrorCode);
		}else {
    printf("✅ USART3 IT 监听开启成功！\r\n");
		}
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
	
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &c, 1, 1000);
    return ch;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
//    // === 1. 步进电机的处理逻辑===
//    if(huart->Instance == USART3) 
//    {
//        extern uint8_t rxCmd[];
//        extern volatile int32_t step_motor_pos; // 引入全局变量
//        
//        // 强制作废 Cache，让 CPU 去内存读取 DMA 刚送来的最新数据！
//        SCB_InvalidateDCache_by_Addr((uint32_t *)rxCmd, 128);
//            
//			
//				// 🌟 终极手段 2：开启底层抓包！把电机发来的每一滴数据都打印到 Vofa+
////        printf("🔍 [串口3抓包] 长度:%d 数据: ", Size);
////        for(int i = 0; i < Size; i++) {
////            printf("%02X ", rxCmd[i]); // 打印原始 Hex 字节
////        }
////        printf("\r\n");
//				
//				
//        // 🌟 核心修复：你之前漏掉了这个 for 循环！这是防止数据错位的终极武器！
//        for(int i = 0; i <= Size - 7; i++) 
//        {
//            // 只要在数组里找到 0x01(地址) 和 0x36(汇报位置码) 紧紧连在一起，就是真数据！
//            if (rxCmd[i] == 0x01 && rxCmd[i+1] == 0x36) 
//            {
//                step_motor_pos = ((int32_t)rxCmd[i+2] << 24) | 
//                                 ((int32_t)rxCmd[i+3] << 16) | 
//                                 ((int32_t)rxCmd[i+4] << 8)  | 
//                                 (uint8_t)rxCmd[i+5];
//                break; // 找到并解析成功，立刻跳出循环！
//            }
//        }
//                
//        // 重新开启 DMA 接收
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxCmd, 128);
//    }
    
		if(huart->Instance == USART3) 
    {
        extern uint8_t rxCmd[];
        extern volatile int32_t step_motor_pos; 
        extern volatile uint32_t motor_update_count;
        // 🌟 1. 彻底作废 D-Cache，强制 CPU 去物理内存拿最新的原始数据
//        SCB_InvalidateDCache_by_Addr((uint32_t *)rxCmd, 128);
            
//				printf("🔍 [串口3抓包] 长度:%d 数据: ", Size);
				
        // 🌟 2. 这里的逻辑没问题，但我们要确保 i 循环能找对包头
        if (Size >= 6) 
        {
            for(int i = 0; i <= Size - 6; i++) 
            {
                if (rxCmd[i] == 0x01 && rxCmd[i+1] == 0x36) 
                {
                    // 🌟 3. 使用关键字 volatile 强制读取，防止编译器优化掉赋值
                    int32_t new_pos = ((int32_t)rxCmd[i+2] << 24) | 
                                      ((int32_t)rxCmd[i+3] << 16) | 
                                      ((int32_t)rxCmd[i+4] << 8)  | 
                                      (uint8_t)rxCmd[i+5];
                    
                    // 只有当位置确实变了，才更新，防止跳动
										step_motor_pos = new_pos; // 赋值
										motor_update_count++;    // 🌟 加这个计数器，看它会不会动！
										break;
                }
            }
        }
        
        // 4. 重新开启下一次接收
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, rxCmd, 128);
    }
		
    // === 2. 激光雷达 (UART2) -> 敲雷达的门铃 ===
    if(huart->Instance == USART2)
    {
        extern osSemaphoreId_t Sem_SensorRxHandle; // 引入 CubeMX 生成的信号量
        extern uint16_t Lidar_WriteIndex;
        
        SCB_InvalidateDCache_by_Addr((uint32_t *)Lidar_RxBuf, 640);
        Lidar_WriteIndex = Size; 
        
        // 🔔 按门铃，唤醒 SensorTask 去慢慢解析点云
        osSemaphoreRelease(Sem_SensorRxHandle); 
			
			  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Lidar_RxBuf, 640);
    }
		
    // === 3. 手机蓝牙 (UART4) -> 敲蓝牙的门铃 ===
    if(huart->Instance == UART4)
    {
        extern osSemaphoreId_t Sem_BtRxHandle;
        
        bt_rx_buf[Size < 64 ? Size : 63] = '\0'; // 安全封尾
        
        // 🔔 按门铃，唤醒 CommTask 去解析指令
        osSemaphoreRelease(Sem_BtRxHandle);      
        
        HAL_UARTEx_ReceiveToIdle_IT(&huart4, bt_rx_buf, 64); // 重新挂载监听
    }
		
		// === 4. 超声波 (UART5) ===
    if(huart->Instance == UART5)
    {
        extern uint8_t ultra_rx_buf[];
        
        // 🛑 警告：已经删除了这里所有的 printf 打印！
        // 绝对不能在 RTOS 的中断里打印数据，否则必死机！

				
				// 🌟 临时侦察兵：把超声波发给单片机的第一手原始数据打印出来！
        //printf("🦇 [超声波抓包] 长度:%d 数据: %02X %02X %02X %02X\r\n", Size, ultra_rx_buf[0], ultra_rx_buf[1], ultra_rx_buf[2], ultra_rx_buf[3]);
        
			
				// 1. 调用解析函数 (里面只能是纯数学计算，不能有任何 Delay)
        Ultrasonic_Parse_Frame(ultra_rx_buf, Size);
			
        // 2. 重新开启空闲中断接收
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, ultra_rx_buf, 16);
    }
		
}
 
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    if (htim->Instance == TIM6) 
    {
        // 每 10ms，雷打不动地执行一次底盘闭环控制
        extern void Motor_Update_PID(void);
        Motor_Update_PID(); 
    }
		
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
