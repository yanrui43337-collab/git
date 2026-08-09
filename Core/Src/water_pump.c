#include "water_pump.h"
#include "tim.h"

#define WATER_PUMP_CHANNEL TIM_CHANNEL_2

static uint8_t water_pump_duty_percent = 0U;
static uint8_t water_pump_running = 0U;

static uint8_t WaterPump_ClampDuty(uint8_t duty_percent)
{
    return (duty_percent > 100U) ? 100U : duty_percent;
}

static uint32_t WaterPump_DutyToCompare(uint8_t duty_percent)
{
    uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(&htim15) + 1U;

    return (period_counts * duty_percent) / 100U;
}

HAL_StatusTypeDef WaterPump_Init(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim15, WATER_PUMP_CHANNEL, 0U);

    water_pump_duty_percent = 0U;
    water_pump_running = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef WaterPump_Start(uint8_t duty_percent)
{
    HAL_StatusTypeDef status;

    duty_percent = WaterPump_ClampDuty(duty_percent);
    if (duty_percent == 0U)
    {
        return WaterPump_Stop();
    }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim15,
                          WATER_PUMP_CHANNEL,
                          WaterPump_DutyToCompare(duty_percent));

    if (water_pump_running == 0U)
    {
        status = HAL_TIM_PWM_Start(&htim15, WATER_PUMP_CHANNEL);
        if (status != HAL_OK)
        {
            __HAL_TIM_SET_COMPARE(&htim15, WATER_PUMP_CHANNEL, 0U);
            return status;
        }
        water_pump_running = 1U;
    }

    water_pump_duty_percent = duty_percent;
    return HAL_OK;
}

HAL_StatusTypeDef WaterPump_SetDuty(uint8_t duty_percent)
{
    return WaterPump_Start(duty_percent);
}

HAL_StatusTypeDef WaterPump_Stop(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    __HAL_TIM_SET_COMPARE(&htim15, WATER_PUMP_CHANNEL, 0U);

    if (water_pump_running != 0U)
    {
        status = HAL_TIM_PWM_Stop(&htim15, WATER_PUMP_CHANNEL);
    }

    if (status == HAL_OK)
    {
        water_pump_duty_percent = 0U;
        water_pump_running = 0U;
    }

    return status;
}

uint8_t WaterPump_GetDuty(void)
{
    return water_pump_duty_percent;
}

uint8_t WaterPump_IsRunning(void)
{
    return water_pump_running;
}
