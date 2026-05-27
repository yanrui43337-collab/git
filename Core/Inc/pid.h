#ifndef __PID_H
#define __PID_H

#include <math.h>

// 1. 制造 PID 的“模具（图纸）”
typedef struct 
{
  float kp;                       
  float ki;                       
  float kd;                       
  float ek;                       
  float ek1;                      
  float ek2;                      
  float location_sum;             
  float out;   
  // 注意：这里只声明，绝对不能写 = 700 !
  float limit_max;                
  float limit_min; 
} PID_LocTypeDef;

// 2. 函数声明 (把你原来那个拗口的 PSpeedPIDControl_Struct 换成了更清晰的指针)
float PID_location(float setvalue, float actualvalue, PID_LocTypeDef *PID);
float PID_increment(float setvalue, float actualvalue, PID_LocTypeDef *PID);

// 3. 对外宣告：我们有 5 个 PID 实体兵团！让 main.c 也能认识它们
extern PID_LocTypeDef Motor1_Speed_PID;
extern PID_LocTypeDef Motor2_Speed_PID;
extern PID_LocTypeDef Motor3_Speed_PID;
extern PID_LocTypeDef Motor4_Speed_PID;
extern PID_LocTypeDef Yaw_Angle_PID;

#endif
