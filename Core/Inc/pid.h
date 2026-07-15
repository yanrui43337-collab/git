#ifndef __PID_H
#define __PID_H

#include <math.h>

// 1. ���� PID �ġ�ģ�ߣ�ͼֽ����
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
  // ע�⣺����ֻ���������Բ���д = 700 !
  float limit_max;                
  float limit_min; 
} PID_LocTypeDef;

// 2. �������� (����ԭ���Ǹ��ֿڵ� PSpeedPIDControl_Struct �����˸�������ָ��)
float PID_location(float setvalue, float actualvalue, PID_LocTypeDef *PID);
float PID_increment(float setvalue, float actualvalue, PID_LocTypeDef *PID);

// 3. �������棺������ 5 �� PID ʵ����ţ��� main.c Ҳ����ʶ����
extern PID_LocTypeDef Motor1_Speed_PID;
extern PID_LocTypeDef Motor2_Speed_PID;
extern PID_LocTypeDef Motor3_Speed_PID;
extern PID_LocTypeDef Motor4_Speed_PID;
extern PID_LocTypeDef Yaw_Angle_PID;

#endif
