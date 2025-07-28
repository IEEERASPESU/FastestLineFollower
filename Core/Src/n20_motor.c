// Core/Src/n20_motor.c
// Core/Src/n20_motor.c

#include "n20_motor.h"
#include <stdio.h>
#include <string.h>

// Extern the UART handle if you want to print from here
extern UART_HandleTypeDef huart1;

// Motor constants
#define ENCODER_PPR 7.0f
#define GEAR_RATIO  50.0f
#define PWM_PERIOD  999 // The ARR value of your PWM timer
#define MAX_MOTOR_RPM 600.0f
/**
 * @brief Initializes the motor structure and starts the peripherals.
 */
void Motor_Init(N20_Motor_t* motor, TIM_HandleTypeDef* pwm_timer, uint32_t pwm_channel,
                TIM_HandleTypeDef* encoder_timer, GPIO_TypeDef* dir1_port, uint16_t dir1_pin,
                GPIO_TypeDef* dir2_port, uint16_t dir2_pin)
{
    // Link hardware to the struct
    motor->pwm_timer = pwm_timer;
    motor->pwm_channel = pwm_channel;
    motor->encoder_timer = encoder_timer;
    motor->dir1_port = dir1_port;
    motor->dir1_pin = dir1_pin;
    motor->dir2_port = dir2_port;
    motor->dir2_pin = dir2_pin;

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
	 // Clamp the input speed for safety
	    if (speed > PWM_PERIOD) speed = PWM_PERIOD;
	    if (speed < -PWM_PERIOD) speed = -PWM_PERIOD;

	    if (speed > 0)
	    {
	        // Forward: IN1 = HIGH, IN2 = LOW
	        HAL_GPIO_WritePin(motor->dir1_port, motor->dir1_pin, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(motor->dir2_port, motor->dir2_pin, GPIO_PIN_RESET);
	    }
	    else if (speed < 0)
	    {
	        // Reverse: IN1 = LOW, IN2 = HIGH
	        HAL_GPIO_WritePin(motor->dir1_port, motor->dir1_pin, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(motor->dir2_port, motor->dir2_pin, GPIO_PIN_SET);
	    }
	    else
	    {
	        // Stop (Brake): IN1 = LOW, IN2 = LOW
	        HAL_GPIO_WritePin(motor->dir1_port, motor->dir1_pin, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(motor->dir2_port, motor->dir2_pin, GPIO_PIN_RESET);
	    }

	    // Set the speed on the EN pin using the absolute value
	    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, abs(speed));
}

/**
 * @brief Stops the motor.
 */
void Motor_Stop(N20_Motor_t* motor)
{
    Motor_Set_Speed(motor, 0);
}
static int32_t scale_value(float value, float in_min, float in_max, int32_t out_min, int32_t out_max)
{
    // Clamp the value to the input range
    if (value < in_min) value = in_min;
    if (value > in_max) value = in_max;

    return (int32_t)((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

/**
 * @brief Calculates RPM and prints it. Should be called periodically.
 */
int32_t Motor_Get_Scaled_RPM(N20_Motor_t* motor)
{
    // --- Step 1: Calculate the actual RPM as a float ---
    uint32_t current_time = HAL_GetTick();
    uint16_t current_encoder_count = __HAL_TIM_GET_COUNTER(motor->encoder_timer);
    float delta_time = (float)(current_time - motor->last_time) / 1000.0f;

    if (delta_time < 0.001f)
    {
        // Not enough time passed, return a scaled version of the last known RPM
        //return scale_value(fabsf(motor->rpm), 0.0f, MAX_MOTOR_RPM, 0, 999);
	int32_t scaled_last_rpm = scale_value(fabsf(motor->rpm), 0.0f, MAX_MOTOR_RPM, 0, 999);
        return (motor->rpm < 0) ? -scaled_last_rpm : scaled_last_rpm;
    }

    int16_t delta_counts = (int16_t)(current_encoder_count - motor->last_encoder_count);
    float calculated_rpm = ((float)delta_counts / (motor->encoder_ppr * motor->gear_ratio)) * (60.0f / delta_time);

    // Update state for the next call
    motor->last_encoder_count = current_encoder_count;
    motor->last_time = current_time;
    motor->rpm = calculated_rpm;

    // Scale the absolute RPM to the 0-999 range ---
    
    //Old section: We use fabsf() to get the absolute speed, this returns only a positive value
    //float absolute_rpm = fabsf(calculated_rpm);
    //int32_t scaled_rpm = scale_value(absolute_rpm, 0.0f, MAX_MOTOR_RPM, 0, 999);
	//return scaled_rpm;
    int32_t scaled_rpm = scale_value(fabsf(calculated_rpm), 0.0f, MAX_MOTOR_RPM, 0, 999);
	return (calculated_rpm < 0) ? -scaled_rpm : scaled_rpm;

}

