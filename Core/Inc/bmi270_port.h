#ifndef __BMI270_PORT_H__
#define __BMI270_PORT_H__

/* 引入标准整数类型定义 (如 uint8_t, int8_t 等) */
#include <stdint.h>

/* ========================================================= */
/* 底层桥接函数声明                        */
/* ========================================================= */

/* I2C 读函数接口封装 */
int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/* I2C 写函数接口封装 */
int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/* 微秒级延时接口封装 */
void bmi2_delay_us(uint32_t period, void *intf_ptr);

#endif /* __BMI270_PORT_H__ */