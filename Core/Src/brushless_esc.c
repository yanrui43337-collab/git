#include "brushless_esc.h"
#include "tim.h"

#define ESC_NEUTRAL_US          1500U
#define ESC_MAX_OFFSET_US        450U
#define ESC_ARM_TIME_MS         5000U
#define ESC_REVERSE_PAUSE_MS     500U
#define ESC_RAMP_STEP_US           2U
#define ESC_RAMP_DELAY_MS          5U

static uint16_t current_pulse_us = ESC_NEUTRAL_US;
static uint8_t esc_initialized = 0U;
static uint32_t esc_arm_start_tick = 0U;

static void Brushless_ESC_RampTo(uint16_t target_pulse_us)
{
    while (current_pulse_us != target_pulse_us)
    {
        if (current_pulse_us < target_pulse_us)
        {
            uint16_t remaining = target_pulse_us - current_pulse_us;
            current_pulse_us += (remaining > ESC_RAMP_STEP_US) ? ESC_RAMP_STEP_US : remaining;
        }
        else
        {
            uint16_t remaining = current_pulse_us - target_pulse_us;
            current_pulse_us -= (remaining > ESC_RAMP_STEP_US) ? ESC_RAMP_STEP_US : remaining;
        }

        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, current_pulse_us);
        HAL_Delay(ESC_RAMP_DELAY_MS);
    }
}

HAL_StatusTypeDef Brushless_ESC_Init(void)
{
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
    current_pulse_us = ESC_NEUTRAL_US;
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ESC_NEUTRAL_US);

    status = HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
    if (status != HAL_OK)
    {
        esc_initialized = 0U;
        return status;
    }

    esc_initialized = 1U;
    esc_arm_start_tick = HAL_GetTick();
    return HAL_OK;
}

void Brushless_ESC_Control(int8_t direction, uint8_t speed_pct)
{
    uint16_t offset_us;
    uint16_t target_pulse_us;
    uint8_t crosses_neutral;

    if (esc_initialized == 0U)
    {
        return;
    }

    /* Keep sending neutral while the ESC arms, but never hold up the RTOS
       scheduler. Commands received during this safety window are ignored. */
    if ((HAL_GetTick() - esc_arm_start_tick) < ESC_ARM_TIME_MS)
    {
        current_pulse_us = ESC_NEUTRAL_US;
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ESC_NEUTRAL_US);
        return;
    }

    if (speed_pct > 100U)
    {
        speed_pct = 100U;
    }

    offset_us = (uint16_t)(((uint32_t)ESC_MAX_OFFSET_US * speed_pct) / 100U);

    if ((direction > 0) && (speed_pct > 0U))
    {
        target_pulse_us = ESC_NEUTRAL_US + offset_us;
    }
    else if ((direction < 0) && (speed_pct > 0U))
    {
        target_pulse_us = ESC_NEUTRAL_US - offset_us;
    }
    else
    {
        target_pulse_us = ESC_NEUTRAL_US;
    }

    crosses_neutral = ((current_pulse_us < ESC_NEUTRAL_US) &&
                       (target_pulse_us > ESC_NEUTRAL_US)) ||
                      ((current_pulse_us > ESC_NEUTRAL_US) &&
                       (target_pulse_us < ESC_NEUTRAL_US));

    if (crosses_neutral != 0U)
    {
        Brushless_ESC_RampTo(ESC_NEUTRAL_US);
        HAL_Delay(ESC_REVERSE_PAUSE_MS);
    }

    Brushless_ESC_RampTo(target_pulse_us);
}

void Brushless_ESC_Stop(void)
{
    Brushless_ESC_Control(0, 0U);
}

uint16_t Brushless_ESC_GetPulseUs(void)
{
    return current_pulse_us;
}
