/*
 * motor.h
 *
 *  Created on: Mar 6, 2023
 *      Author: krjar
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f4xx_hal.h"

typedef enum
{
	CH1 = TIM_CHANNEL_1,
	CH2 = TIM_CHANNEL_2,
	CH3 = TIM_CHANNEL_3,
	CH4 = TIM_CHANNEL_4
}pwm_timer_channel;

typedef struct
{
	// external variables
	TIM_HandleTypeDef *enc_timer_handle;					// Encoder timer handle
	TIM_HandleTypeDef *pwm_timer_handle;					// PWM timer handle
	pwm_timer_channel pwm_timer_channel;					// PWM timer channel

	//internal variables
	uint16_t resolution;									// Motor encoder resolution
	uint32_t pulse_count;									// Actual encoder pulses
	int32_t speed;											// Calculated motor speed
}motor;


void motorInit(motor *m, TIM_HandleTypeDef *enc_tim, TIM_HandleTypeDef *pwm_tim, pwm_timer_channel pwm_ch);

#endif /* INC_MOTOR_H_ */
