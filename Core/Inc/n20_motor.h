/*
 * n20_motor.h
 *
 *  Created on: Jul 18, 2025
 *      Author: adithya
 */

#ifndef INC_N20_MOTOR_H_
#define INC_N20_MOTOR_H_
// Core/Inc/n20_motor.h

#include "stm32f4xx_hal.h"

// A structure to hold all data related to a single motor
typedef struct {
    // Hardware Handles
    TIM_HandleTypeDef* pwm_timer;
    uint32_t pwm_channel;
    TIM_HandleTypeDef* encoder_timer;
    GPIO_TypeDef* dir1_port;
    uint16_t      dir1_pin;
    GPIO_TypeDef* dir2_port;
    uint16_t      dir2_pin;


    // Motor Parameters
    float gear_ratio;
    float encoder_ppr;

    // RPM Calculation State
    int32_t last_encoder_count;
    uint32_t last_time;
    float rpm;

} N20_Motor_t;


// --- Public Function Prototypes ---

void Motor_Init(N20_Motor_t* motor, TIM_HandleTypeDef* pwm_timer, uint32_t pwm_channel,
                TIM_HandleTypeDef* encoder_timer, GPIO_TypeDef* dir1_port, uint16_t dir1_pin,
                GPIO_TypeDef* dir2_port, uint16_t dir2_pin);

void Motor_Set_Speed(N20_Motor_t* motor, int16_t speed);


int Motor_Get_Scaled_RPM(N20_Motor_t* motor);

void Motor_Stop(N20_Motor_t* motor);


#endif /* INC_N20_MOTOR_H_ */


