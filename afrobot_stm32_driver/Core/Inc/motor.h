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

typedef enum
{
	FR_DIR_PIN = GPIO_PIN_11,
	FL_DIR_PIN = GPIO_PIN_12,
	BR_DIR_PIN = GPIO_PIN_13,
	BL_DIR_PIN = GPIO_PIN_14
}motor_dir_pin;

typedef enum
{
	FW = 0,
	BW = 1
}motor_dir;

typedef struct
{
	// external variables
	TIM_HandleTypeDef *enc_timer_handle;					// Encoder timer handle
	TIM_HandleTypeDef *pwm_timer_handle;					// PWM timer handle
	pwm_timer_channel pwm_timer_ch;							// PWM timer channel
	motor_dir_pin dir_pin;									// Motor direction set pin

	//internal variables
	uint16_t resolution;									// Motor encoder resolution
	int16_t pulse_count;									// Actual encoder pulses
	double speed;											// Calculated motor speed (RPM = 1/min)

}motor;


void motorInit(motor *, TIM_HandleTypeDef *, TIM_HandleTypeDef *, pwm_timer_channel, motor_dir_pin);
void motorSetDirection(motor *, motor_dir);
void motorSetSpeed(motor *, int);
void motorUpdatePulse(motor *);
void motorCalculateSpeed(motor *, int);

#endif /* INC_MOTOR_H_ */
