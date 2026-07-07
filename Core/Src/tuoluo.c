#include "tuoluo.h"
#include "bmi270.h"
#include "bmi270_port.h"
#include "stdio.h"

// 加上 static，让这些变量变成这个文件的“私有财产”，绝对不会和 main.c 冲突
static struct bmi2_dev bmi270_dev;
static struct bmi2_sens_data sensor_data;
static uint8_t dev_addr = 0x68; 

// 声明外部的延时函数（它静静地躺在商家的 bmi270_port.c 里）
extern void bmi2_delay_us(uint32_t period, void *intf_ptr);

// ==========================================
// 陀螺仪初始化模块
// ==========================================
int8_t Tuoluo_Init(void) {
    bmi270_dev.intf = BMI2_I2C_INTF;
    bmi270_dev.read = bmi2_i2c_read;
    bmi270_dev.write = bmi2_i2c_write; 
    
    // 直接挂载外部已经写好的延时函数
    bmi270_dev.delay_us = bmi2_delay_us; 
    
    bmi270_dev.intf_ptr = &dev_addr;
    
    // I2C 单次最大传输长度设为 128
    bmi270_dev.read_write_len = 128; 
    bmi270_dev.config_file_ptr = NULL;

    // 1. 初始化并加载微码
    int8_t rslt = bmi270_init(&bmi270_dev);
    
    if (rslt == BMI2_OK) {
        printf("IMU Init OK!\r\n");
        
        // 2. 开启传感器
        uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
        rslt = bmi2_sensor_enable(sens_list, 2, &bmi270_dev);
        
        // 3. 配置传感器参数
        if (rslt == BMI2_OK) {
            struct bmi2_sens_config config[2];

            // --- 加速度计配置 ---
            config[0].type = BMI2_ACCEL;
            config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;        
            config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;      
            config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
            config[0].cfg.acc.range = BMI2_ACC_RANGE_8G;       // +-8G -> 4096 LSB/g

            // --- 陀螺仪配置 ---
            config[1].type = BMI2_GYRO;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;        
            config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;      
            config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
            config[1].cfg.gyr.ois_range = BMI2_GYR_OIS_250;    
            config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;     // +-2000dps -> 16.384 LSB/dps
            config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE; 

            rslt = bmi270_set_sensor_config(config, 2, &bmi270_dev);
            if (rslt == BMI2_OK) {
                printf("IMU Config OK! Sensor is running.\r\n");
            } else {
                printf("IMU Config Fail! Error: %d\r\n", rslt);
            }
        } else {
            printf("IMU Enable Fail! Error: %d\r\n", rslt);
        }
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
        // 加速度换算 (单位: g)
        data->Acc_X = sensor_data.acc.x / 4096.0f;
        data->Acc_Y = sensor_data.acc.y / 4096.0f;
        data->Acc_Z = sensor_data.acc.z / 4096.0f;
        
        // 角速度换算 (单位: dps 度每秒)
        data->Gyro_X = sensor_data.gyr.x / 16.384f;
        data->Gyro_Y = sensor_data.gyr.y / 16.384f;
        data->Gyro_Z = sensor_data.gyr.z / 16.384f;
    }
		else
		{
			printf("❌ IMU读取失败! 错误码: %d\r\n", rslt);
		}
}