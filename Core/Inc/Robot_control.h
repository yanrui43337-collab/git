#ifndef __ROBOT_CONTROL_H__
#define __ROBOT_CONTROL_H__

#include <stdint.h>

/* ====================================================================
 * 1. 结构体定义 (用结构体代替C++里的类成员变量)
 * ==================================================================== */
typedef struct {
    int last_min;        // 上一次测量的最小距离
    int confirm_counter; // 确认计数器（防抖用）
    int is_suspect_leg;  // 状态标志：是否怀疑有突然闯入的腿 (1:是, 0:否)
} DynamicObstacleState;

/* ====================================================================
 * 2. 外部状态变量声明 (让 main.c 也能看到这两个状态)
 * ==================================================================== */
extern DynamicObstacleState left_state;
extern DynamicObstacleState right_state;

/* ====================================================================
 * 3. 核心算法与控制函数声明
 * ==================================================================== */

// 雷达动态障碍物(人腿)检测滤波算法
int Check_Sudden_Obstacle(int current_min, DynamicObstacleState *state);

// 麦克纳姆轮底盘运动学逆解
void Move_Mecanum(int v_x, int v_y, int v_z);

/* ====================================================================
 * 4. 机器人动作层函数声明 (留空待补)
 * ==================================================================== */
void Motor_Control(int m1, int m2, int m3, int m4); // 硬件底层电机控制
void Robot_Stop(void);                              // 紧急刹车等待
void Start_Climbing_Stairs(void);                   // 爬楼梯序列动作(暂时不用)
void Robot_Workflow_Task(void);                     // 爬楼梯总运行逻辑

#endif /* __ROBOT_CONTROL_H__ */
