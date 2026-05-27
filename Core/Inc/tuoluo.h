#ifndef __TUOLUO_H__
#define __TUOLUO_H__

#include <stdint.h>

// 专门建一个结构体，存放转换后的真实物理数据，准备直接喂给PID
typedef struct {
    float Acc_X;     // 加速度 (g)
    float Acc_Y;
    float Acc_Z;
    float Gyro_X;    // 角速度 (度/秒)
    float Gyro_Y;
    float Gyro_Z;
} Tuoluo_Data_t;

// 对外暴露的两个极简函数
int8_t Tuoluo_Init(void);
void Tuoluo_Read(Tuoluo_Data_t *data);

#endif /* __TUOLUO_H__ */
