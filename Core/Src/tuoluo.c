#include "tuoluo.h"
#include "bmi270.h"
#include "bmi270_port.h"
#include "stdio.h"

// 加上 static，让这些变量变成这个文件的“私有财产”，绝对不会和 main.c 冲突
static struct bmi2_dev bmi270_dev;
static struct bmi2_sens_data sensor_data;
static uint8_t dev_addr = 0x68; 

// ==========================================
// 陀螺仪初始化模块
// ==========================================
int8_t Tuoluo_Init(void) {
    bmi270_dev.intf = BMI2_I2C_INTF;
    bmi270_dev.read = bmi2_i2c_read;
    bmi270_dev.write = bmi2_i2c_write;
    bmi270_dev.delay_us = bmi2_delay_us;
    bmi270_dev.intf_ptr = &dev_addr;
    bmi270_dev.read_write_len = 32; 
    bmi270_dev.config_file_ptr = NULL;

    int8_t rslt = bmi270_init(&bmi270_dev);
    if (rslt == BMI2_OK) {
        printf("IMU Init OK!\r\n");
        // 只有初始化成功了，才开启传感器
        uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
        bmi2_sensor_enable(sens_list, 2, &bmi270_dev);
    } else {
        printf("IMU Init Fail! Error: %d\r\n", rslt);
    }
    return rslt;
}

// ==========================================
// 陀螺仪读取与物理单位换算模块
// ==========================================
void Tuoluo_Read(Tuoluo_Data_t *data) {
    int8_t rslt = bmi2_get_sensor_data(&sensor_data, &bmi270_dev);
    
    if (rslt == BMI2_OK) {
        // 加速度换算：默认量程+-8g，除以 4096.0f
        data->Acc_X = sensor_data.acc.x / 4096.0f;
        data->Acc_Y = sensor_data.acc.y / 4096.0f;
        data->Acc_Z = sensor_data.acc.z / 4096.0f;
        
        // 角速度换算：默认量程+-2000dps，除以 16.384f
        data->Gyro_X = sensor_data.gyr.x / 16.384f;
        data->Gyro_Y = sensor_data.gyr.y / 16.384f;
        data->Gyro_Z = sensor_data.gyr.z / 16.384f;
    }
}