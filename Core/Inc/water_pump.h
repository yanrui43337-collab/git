#ifndef WATER_PUMP_H
#define WATER_PUMP_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Water-pump PWM driver for PE6 / TIM15_CH2.
 * PD10 is kept low for the existing signal-return wiring.
 *
 * Call WaterPump_Init() once after MX_TIM15_Init().
 * duty_percent is limited to 0..100 percent.
 *
 * Examples:
 *   WaterPump_Start(30U);     // start at 30 percent duty
 *   WaterPump_SetDuty(60U);   // change speed to 60 percent
 *   WaterPump_Stop();         // stop PWM output
 *   WaterPump_Start(30U);     // start again
 */
HAL_StatusTypeDef WaterPump_Init(void);
HAL_StatusTypeDef WaterPump_Start(uint8_t duty_percent);
HAL_StatusTypeDef WaterPump_SetDuty(uint8_t duty_percent);
HAL_StatusTypeDef WaterPump_Stop(void);
uint8_t WaterPump_GetDuty(void);
uint8_t WaterPump_IsRunning(void);

#ifdef __cplusplus
}
#endif

#endif /* WATER_PUMP_H */
