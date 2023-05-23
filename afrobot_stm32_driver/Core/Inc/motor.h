/*
 * motor.h
 *
 *  Created on: Mar 6, 2023
 *      Author: krjar
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f4xx_hal.h"
#include <pid.h>
#include <time.h>
#include <mafilter.h>

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
	/* external variables */
	TIM_HandleTypeDef *enc_timer_handle;					// Encoder timer handle
	TIM_HandleTypeDef *pwm_timer_handle;					// PWM timer handle
	pwm_timer_channel pwm_timer_ch;							// PWM timer channel
	motor_dir_pin dir_pin;									// Motor direction set pin
	pid *controller ;										// PID controller structure
	filterType *filter ; 									// Filter for odometry
	/* internal variables */
	uint16_t resolution;									// Motor encoder resolution
	int16_t pulse_count;									// Actual encoder pulses
	float speed;											// Calculated wheel angular velocity (rad/s)
	float speed_cmd;										// Set wheel angular velocity (rad/s)
	int pwm_value ;											// Actual PWM signal value (0-1000)
	uint32_t time ;											// Last speed measure time (cpu ticks)

}motor;


void motorInit(motor *, pid *, filterType *, TIM_HandleTypeDef *, TIM_HandleTypeDef *, pwm_timer_channel, motor_dir_pin, uint16_t);
void motorSetDirection(motor *, motor_dir);
void motorSetPWM(motor *, int);
void motorUpdatePulse(motor *);
void motorCalculateSpeed(motor *);
void motorRegulateSpeed(motor *);
void motorSetSpeed(motor *, double);

#endif /* INC_MOTOR_H_ */
