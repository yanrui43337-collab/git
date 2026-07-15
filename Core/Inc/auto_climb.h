#ifndef __AUTO_CLIMB_H
#define __AUTO_CLIMB_H

#include "main.h"
typedef struct {
    int x;
    int y;
    int w;
} ChassisMsg_t;


// 将自动爬楼的整个流程封装成一个函数

void AutoClimb_Process(void);

#endif
