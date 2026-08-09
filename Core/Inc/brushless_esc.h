#ifndef BRUSHLESS_ESC_H
#define BRUSHLESS_ESC_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * AM32 bidirectional ESC driver for PE5 / TIM15_CH1.
 * PD10 is driven low as the signal return used by the existing wiring.
 *
 * Call Brushless_ESC_Init() once after MX_TIM15_Init(). It starts 1500 us
 * neutral immediately and returns without blocking the RTOS. Speed commands
 * are ignored during the first five seconds while the ESC safely arms.
 *
 * direction:  1 = forward, -1 = reverse, 0 = stop
 * speed_pct:  0..100 percent
 *
 * Examples:
 *   Brushless_ESC_Control( 1, 70);  // forward at 70 percent
 *   Brushless_ESC_Control(-1, 70);  // reverse at 70 percent
 *   Brushless_ESC_Stop();           // neutral / stop
 *
 * Control() performs a blocking soft ramp. After it returns, hardware PWM
 * continues indefinitely at the requested speed.
 */
HAL_StatusTypeDef Brushless_ESC_Init(void);
void Brushless_ESC_Control(int8_t direction, uint8_t speed_pct);
void Brushless_ESC_Stop(void);
uint16_t Brushless_ESC_GetPulseUs(void);

#ifdef __cplusplus
}
#endif

#endif /* BRUSHLESS_ESC_H */
