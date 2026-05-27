#include "bmi270.h"
#include "i2c.h" // 引入你的 I2C 句柄 (hi2c1)

// 1. I2C 读函数接口封装
// 注意：STM32 的 HAL 库要求 I2C 地址必须左移 1 位 (<< 1)
int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    
    if (HAL_I2C_Mem_Read(&hi2c1, (dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 100) != HAL_OK) {
        return BMI2_E_COM_FAIL;
    }
    return BMI2_OK;
}

// 2. I2C 写函数接口封装
int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    
    if (HAL_I2C_Mem_Write(&hi2c1, (dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, len, 100) != HAL_OK) {
        return BMI2_E_COM_FAIL;
    }
    return BMI2_OK;
}

// 3. 微秒级延时接口封装
void bmi2_delay_us(uint32_t period, void *intf_ptr) {
    // HAL库只有毫秒级延时，这里做一个粗略换算。加 1 是为了防止 period 小于 1000 时直接变为 0
    HAL_Delay((period / 1000) + 1); 
}
