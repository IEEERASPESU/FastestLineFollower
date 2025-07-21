// Core/Src/n20_motor.c

#include "n20_motor.h"
#include <stdio.h>
#include <string.h>

// Extern the UART handle if you want to print from here
extern UART_HandleTypeDef huart1;

// Motor constants
#define ENCODER_PPR 7.0f
#define GEAR_RATIO  30.0f
#define PWM_PERIOD  999 // The ARR value of your PWM timer

/**
 * @brief Initializes the motor structure and starts the peripherals.
 */
void Motor_Init(N20_Motor_t* motor, TIM_HandleTypeDef* pwm_timer, uint32_t pwm_channel,
                TIM_HandleTypeDef* encoder_timer, GPIO_TypeDef* dir_port, uint16_t dir_pin)
{
    // Link hardware to the struct
    motor->pwm_timer = pwm_timer;
    motor->pwm_channel = pwm_channel;
    motor->encoder_timer = encoder_timer;
    motor->direction_port = dir_port;
    motor->direction_pin = dir_pin;

    // Set parameters
    motor->gear_ratio = GEAR_RATIO;
    motor->encoder_ppr = ENCODER_PPR;

    // Reset state variables
    motor->last_encoder_count = 0;
    motor->last_time = 0;
    motor->rpm = 0.0f;

    // Start hardware
    HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_channel);
    HAL_TIM_Encoder_Start(motor->encoder_timer, TIM_CHANNEL_ALL);
}

/**
 * @brief Sets the motor speed and direction.
 * @param speed: -999 (full reverse) to +999 (full forward)
 */
void Motor_Set_Speed(N20_Motor_t* motor, int16_t speed)
{
    if (speed > 0)
    {
        // Forward
        HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, speed);
    }
    else
    {
        // Reverse
        HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, -speed);
    }
}

/**
 * @brief Stops the motor.
 */
void Motor_Stop(N20_Motor_t* motor)
{
    Motor_Set_Speed(motor, 0);
}

/**
 * @brief Calculates RPM and prints it. Should be called periodically.
 */
void Motor_Process_RPM(N20_Motor_t* motor)
{
    // Check if 100ms has passed
    if (HAL_GetTick() - motor->last_time >= 100)
    {
        int32_t encoder_count = __HAL_TIM_GET_COUNTER(motor->encoder_timer);
        float delta_time = (float)(HAL_GetTick() - motor->last_time) / 1000.0f;
        int32_t delta_counts = encoder_count - motor->last_encoder_count;

        motor->rpm = ((float)delta_counts / (motor->encoder_ppr * motor->gear_ratio)) * (60.0f / delta_time);

        // Print RPM to serial monitor
        char uart_buf[50];
        sprintf(uart_buf, "RPM: %.2f | Encoder: %ld\r\n", motor->rpm, encoder_count);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

        motor->last_encoder_count = encoder_count;
        motor->last_time = HAL_GetTick();
    }
}
