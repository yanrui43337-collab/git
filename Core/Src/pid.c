#include "pid.h"

// ==========================================
// 1. 实体化：造出 4 个控制电机转速的 PID (输出范围 ±1000，用来给 PWM)
// ==========================================
PID_LocTypeDef Motor1_Speed_PID = { .kp=200.0, .ki=20.0, .kd=0.0, .ek=0, .ek1=0, .ek2=0, .location_sum=0, .out=0, .limit_max=60000, .limit_min=-60000};
PID_LocTypeDef Motor2_Speed_PID = { .kp=200.0, .ki=20.0, .kd=0.0, .ek=0, .ek1=0, .ek2=0, .location_sum=0, .out=0, .limit_max=60000, .limit_min=-60000 };
PID_LocTypeDef Motor3_Speed_PID = { .kp=200.0, .ki=20.0, .kd=0.0, .ek=0, .ek1=0, .ek2=0, .location_sum=0, .out=0, .limit_max=60000, .limit_min=-60000 };
PID_LocTypeDef Motor4_Speed_PID = { .kp=200.0, .ki=20.0, .kd=0.0, .ek=0, .ek1=0, .ek2=0, .location_sum=0, .out=0, .limit_max=60000, .limit_min=-60000};

// ==========================================
// 2. 实体化：造出 1 个控制方向的 PID (输出范围 ±300，用来修正偏航角)
// ==========================================
PID_LocTypeDef Yaw_Angle_PID = { .kp=2.0, .ki=0.0, .kd=0.5, .ek=0, .ek1=0, .ek2=0, .location_sum=0, .out=0, .limit_max=300, .limit_min=-300 };


// ==========================================
// 核心算法 1：位置式 PID (给陀螺仪角度控制用)
// ==========================================
float PID_location(float setvalue, float actualvalue, PID_LocTypeDef *PID)
{
    PID->ek = setvalue - actualvalue;
    PID->location_sum += PID->ek;                         
    
    // 积分限幅 (调用属于它自己的 limit_max)
    if((PID->ki!=0) && (PID->location_sum > (PID->limit_max/PID->ki))) 
        PID->location_sum = PID->limit_max/PID->ki;
    if((PID->ki!=0) && (PID->location_sum < (PID->limit_min/PID->ki))) 
        PID->location_sum = PID->limit_min/PID->ki;

    PID->out = PID->kp * PID->ek + (PID->ki * PID->location_sum) + PID->kd * (PID->ek - PID->ek1);
    PID->ek1 = PID->ek;
    
    // 输出限幅 (调用属于它自己的 limit_max)
    if(PID->out < PID->limit_min)  PID->out = PID->limit_min;
    if(PID->out > PID->limit_max)  PID->out = PID->limit_max;

    return PID->out;
}

// ==========================================
// 核心算法 2：增量式 PID (给 4 个电机速度控制用)
// ==========================================
float PID_increment(float setvalue, float actualvalue, PID_LocTypeDef *PID)
{
    PID->ek = setvalue - actualvalue;
    
    PID->out += PID->kp * (PID->ek - PID->ek1) + PID->ki * PID->ek + PID->kd * (PID->ek - 2*PID->ek1 + PID->ek2);
    
    PID->ek2 = PID->ek1;
    PID->ek1 = PID->ek;

    // 输出限幅 (调用属于它自己的 limit_max)
    if(PID->out < PID->limit_min)  PID->out = PID->limit_min;
    if(PID->out > PID->limit_max)  PID->out = PID->limit_max;

    return PID->out;
}