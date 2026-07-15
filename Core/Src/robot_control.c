#include "Robot_control.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "pid.h"
#include "Emm_V5.h"
#include <stdbool.h>
#include <lidar.h>

//#include "ultrasonic_uart.h"

// 声明外部的定时器和PID结构体
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

extern PID_LocTypeDef Motor1_Speed_PID;
extern PID_LocTypeDef Motor2_Speed_PID;
extern PID_LocTypeDef Motor3_Speed_PID;
extern PID_LocTypeDef Motor4_Speed_PID;

// 定义四个全局的目标转速变量（由你的 Move_Mecanum 函数来改变它们）
float target_rpm_m1 = 0;
float target_rpm_m2 = 0;
float target_rpm_m3 = 0;
float target_rpm_m4 = 0;
float actual_rpm_m1 ;
float actual_rpm_m2 ;
float actual_rpm_m3 ;
float actual_rpm_m4 ;

typedef enum {
    STATE_FIND_WALL = 0,    // 寻找墙壁（向左平移）
    STATE_CLIMB_UP = 1,     // 遇到墙壁，步进电机爬升
    STATE_CLIMB_WAIT = 2,   // 等待步进电机爬升到位
    STATE_MOVE_RIGHT = 3,   // 爬升完毕，向右平移离开墙壁
    // ... 依此类推
} RobotState_t;

RobotState_t current_state = STATE_FIND_WALL;

extern int left_min;
extern int right_min;
extern int front_min;

/* ====================================================================
 * 全局状态变量实体化
 * 初始化为：上次距离9999，计数器0，无嫌疑0
 * ==================================================================== */

DynamicObstacleState left_state  = {9999, 0, 0};
DynamicObstacleState right_state = {9999, 0, 0};

/* ====================================================================
 * 算法 1：雷达动态障碍物(人腿)检测
 * 返回值： 1 -> 是突然闯入的人腿；  0 -> 是墙壁、扶手等固定物，或者没障碍
 * ==================================================================== */

int Check_Sudden_Obstacle(int current_min, DynamicObstacleState *state) {
    int ALERT_DIST = 200;  // 危险距离阈值：200mm (20cm)
    int JUMP_THRES = 200;  // 突变落差阈值：瞬间缩短200mm以上
    
    // 1. 捕捉瞬间突变
    if (state->last_min - current_min > JUMP_THRES) {
        state->is_suspect_leg = 1; 
    }

    // 2. 状态解除 (加50mm迟滞区间)
    if (current_min > ALERT_DIST + 50) { 
        state->is_suspect_leg = 0;
    }

    // 3. 计数器防抖
    if (state->is_suspect_leg == 1 && current_min < ALERT_DIST) {
        if (state->confirm_counter < 5) state->confirm_counter++;
    } else {
        if (state->confirm_counter > 0) state->confirm_counter--;
    }

    // 4. 更新历史距离
    state->last_min = current_min;

    // 5. 最终输出
    if (state->confirm_counter >= 5) {
        return 1; 
    }
    return 0; 
}

/* ====================================================================
 * 算法 2：麦克纳姆轮运动学逆解
 * v_x: 前后平移速度 (正向为前)
 * v_y: 左右平移速度 (正向为左或右，取决于你的电机线序，一般正为左)
 * v_z: 原地旋转速度 (正向为逆时针)
 * ==================================================================== */
void Move_Mecanum(int v_x, int v_y, int v_z) 
{
    // 直接用！因为变量就定义在上面
    target_rpm_m1 = (float)(v_x + v_y + v_z); 
    target_rpm_m2 = (float)(v_x - v_y - v_z); 
    target_rpm_m3 = (float)(v_x - v_y + v_z); 
    target_rpm_m4 = (float)(v_x + v_y - v_z); 
	
	  if(target_rpm_m1 >  192.0f) target_rpm_m1 =  192.0f;
    if(target_rpm_m1 < -192.0f) target_rpm_m1 = -192.0f;
	  if(target_rpm_m2 >  192.0f) target_rpm_m2 =  192.0f;
    if(target_rpm_m2 < -192.0f) target_rpm_m2 = -192.0f;
	  if(target_rpm_m3 >  192.0f) target_rpm_m3 =  192.0f;
    if(target_rpm_m3 < -192.0f) target_rpm_m3 = -192.0f;
	  if(target_rpm_m4 >  192.0f) target_rpm_m4 =  192.0f;
    if(target_rpm_m4 < -192.0f) target_rpm_m4 = -192.0f;	
}




/* ====================================================================
 * 以下为预留的机器人动作执行函数，你可以逐步填入对应的逻辑
 * ==================================================================== */

// 底层电机调速控制 (包含PID和定时器PWM修改等)
void Motor_Contro1(int m1_speed,int m2_speed, int m3_speed,int m4_speed) {
	
	if(m1_speed > 60000) m1_speed = 60000; if(m1_speed < -60000) m1_speed = -60000;
	if(m2_speed > 60000) m2_speed = 60000; if(m2_speed < -60000) m2_speed = -60000;
	if(m3_speed > 60000) m3_speed = 60000; if(m3_speed < -60000) m3_speed = -60000;
	if(m4_speed > 60000) m4_speed = 60000; if(m4_speed < -60000) m4_speed = -60000;
	
	
	 if (m1_speed >= 0) { 
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);   //电机1AIN1 
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //电机1 AIN2
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, m1_speed);
    } else {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -m1_speed);
    }
        //第二个电机

    if (m2_speed >= 0) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);  // 改为 RESET
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);   // 改为 SET
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m2_speed);
    } else { 
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);    // 改为 SET
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // 改为 RESET
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -m2_speed);
    }
        //第三个电机
        if (m3_speed >= 0) {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);   //电机3AIN1
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET); //电机3AIN2
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, m3_speed);
    } else { 
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -m3_speed);
    }
        //第四个电机
        if (m4_speed >= 0) {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // 改为 RESET
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   // 改为 SET
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, m4_speed);
    } else { 
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   // 改为 SET
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); // 改为 RESET
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -m4_speed);
    }
}

void Motor_Contro2(int m1_speed,int m2_speed, int m3_speed,int m4_speed ) {
	
	if(m1_speed > 60000) m1_speed = 60000; if(m1_speed < -60000) m1_speed = -60000;
	if(m2_speed > 60000) m2_speed = 60000; if(m2_speed < -60000) m2_speed = -60000;
	if(m3_speed > 60000) m3_speed = 60000; if(m3_speed < -60000) m3_speed = -60000;
	if(m4_speed > 60000) m4_speed = 60000; if(m4_speed < -60000) m4_speed = -60000;
	//第一个电机
    if (m1_speed >= 0) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   //电机1AIN1 
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //电机1 AIN2
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, m1_speed);
    } else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -m1_speed);
    }
		//第二个电机
    if (m2_speed >= 0) {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);   //电机2BIN1
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // 电机2BIN2·
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, m2_speed);
    } else { 
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -m2_speed);
    }
		//第三个电机
		if (m3_speed >= 0) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   //电机4BIN1
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); //电机4BIN2
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, m3_speed);
    } else { 
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -m3_speed);
    }
		//第四个电机
		if (m4_speed >= 0) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   //电机3AIN1
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET); //电机3AIN2
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, m4_speed);
    } else { 
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, -m4_speed);
    }
		
}

void Motor_Update_PID(void)
{
    // 1. 读取四个电机的脉冲 (瞬间快照)
    int16_t pulse_m1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
    int16_t pulse_m2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t pulse_m3 = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
    int16_t pulse_m4 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    // 2. 读完立刻清零
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim8, 0);

    // 3. 计算真实 RPM (根据你 1:50 减速比和 11 线编码器得出的完美常数 30/11)
     actual_rpm_m1 = (pulse_m1 * 30.0f) / 11.0f;
     actual_rpm_m2 = -(pulse_m2 * 30.0f) / 11.0f;
     actual_rpm_m3 = (pulse_m3 * 30.0f) / 11.0f;
     actual_rpm_m4 = -(pulse_m4 * 30.0f) / 11.0f;

    // 4. 将目标 RPM 和 实际 RPM 喂给增量式 PID，算出需要的 PWM 值
    int final_pwm_m1 = (int)PID_increment(target_rpm_m1, actual_rpm_m1, &Motor1_Speed_PID);
    int final_pwm_m2 = (int)PID_increment(target_rpm_m2, actual_rpm_m2, &Motor2_Speed_PID);
    int final_pwm_m3 = (int)PID_increment(target_rpm_m3, actual_rpm_m3, &Motor3_Speed_PID);
    int final_pwm_m4 = (int)PID_increment(target_rpm_m4, actual_rpm_m4, &Motor4_Speed_PID);

    // 5. 把算出来的 PWM 值，丢给底层的硬件驱动函数
  
    Motor_Contro1(final_pwm_m1,final_pwm_m2,final_pwm_m3,final_pwm_m4);
        //Motor_Contro1(0,0,0,final_pwm_m4);
}

// 遇到人腿：紧急停车等待
void Robot_Stop(void) {
    Move_Mecanum(0, 0, 0); // 速度全部清零
}

// 遇到墙壁：启动爬楼梯的两级步进电机-滚珠丝杠复合机构
void Start_Climbing_Stairs(void) {
    // 1. 调整车身姿态，确保雷达平行于墙面
    // 2. 触发步进电机动作
    // ...
}



void Robot_Workflow_Task(void) 
{
	
		//front_min = Ultrasonic_Get_Distance();

		// 如果返回的不是 -1 且小于 20cm
		if (front_min > 0 && front_min < 200) { 
				Robot_Stop(); 
				current_state = STATE_CLIMB_UP; 
		}
    // 1. 直接用你的 left_min 检测是否有人腿突然闯入
    int is_left_leg = Check_Sudden_Obstacle(left_min, &left_state);

    // 2. 状态机流转
    switch (current_state)
    {
        case STATE_FIND_WALL:
            if (is_left_leg == 1) {
                // 是人腿，立刻停车等待
                Robot_Stop();
            } 
            // 注意这里：加上 left_min > 0 是为了防止雷达刚启动时数据为0导致的误判
            else if (left_min > 0 && left_min < 200) { 
                // 到底了！是平整的墙壁/扶手

                current_state = STATE_CLIMB_UP;
            } 
            else 
						{
                // 还没靠到墙，也没遇到腿，继续向左平移
                Move_Mecanum(0, 100, 0); // V_y给正值向左
            }
            break;

        case STATE_CLIMB_UP:
            // 等待步进电机到达指定位置。
            // 真实情况可以通过读取 Emm_V5_Read_Sys_Params(1, S_FLAG) 来判断是否停转
            // 如果停转了，进入下一步：
						                
            // 触发步进电机动作：地址1，反转(假设为上升)，500RPM，加减速50，发100000个脉冲
						Robot_Stop(); // 麦克纳姆轮刹车
            Emm_V5_Pos_Control(1, 1, 500, 50, 100000, false, false);
						
            // current_state = STATE_MOVE_RIGHT;
				
            break;
				
				
				case STATE_CLIMB_WAIT:
            // 等待步进电机爬升到位
            // 以后可以通过读取电机标志位，决定什么时候切换到 STATE_MOVE_RIGHT
						
            break;
						
						
        case STATE_MOVE_RIGHT:
            // 爬升完后向右移动的逻辑
						
						
						
            Move_Mecanum(0, -100, 0);
            break;
    }
}

