/*
 * motor.c
 *
 *  Created on: Mar 6, 2023
 *      Author: krjar
 */

#include <main.hpp>

// Constants
#define ENCODER_RESOLUTION		64
#define TIMER_ENCODER_TI1TI2	4
#define MOTOR_GEAR				30

// Functions definitions


void motorInit(motor *m, TIM_HandleTypeDef *enc_tim, TIM_HandleTypeDef *pwm_tim, pwm_timer_channel pwm_ch, motor_dir_pin direction_pin)
{

	// Assign external variables to motor structure
	m->enc_timer_handle = enc_tim ;
	m->pwm_timer_handle = pwm_tim ;
	m->pwm_timer_ch = pwm_ch ;
	m->dir_pin = direction_pin ;

	// Initialize internal variables
	m->resolution = ENCODER_RESOLUTION * TIMER_ENCODER_TI1TI2 * MOTOR_GEAR ;
	m->pulse_count = 0 ;
	m->speed = 0 ;

	// Initialize timers
	HAL_TIM_Encoder_Start(enc_tim, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(pwm_tim, pwm_ch);
	HAL_Delay(10);

	// Check if PWM works
	motorSetDirection(m, FW);
	motorSetSpeed(m, 500);
	HAL_Delay(2000);
	motorSetSpeed(m, 0);
	HAL_Delay(500);
	motorSetDirection(m, BW);
	motorSetSpeed(m, 500);
	HAL_Delay(2000);
	motorSetSpeed(m, 0);
	HAL_Delay(500);
}

void motorSetDirection(motor *m, motor_dir dir){

	if (dir == 1)
		HAL_GPIO_WritePin(GPIOG, m->dir_pin, GPIO_PIN_SET);
	else if (dir == 0)
		HAL_GPIO_WritePin(GPIOG, m->dir_pin, GPIO_PIN_RESET);

}

void motorSetSpeed(motor *m, int duty_cycle){

	if (duty_cycle > 900)
		__HAL_TIM_SET_COMPARE(m->pwm_timer_handle, m->pwm_timer_ch, 900);
	else if(duty_cycle < 200 && duty_cycle > 0)
		__HAL_TIM_SET_COMPARE(m->pwm_timer_handle, m->pwm_timer_ch, 200);
	else
		__HAL_TIM_SET_COMPARE(m->pwm_timer_handle, m->pwm_timer_ch, duty_cycle);
}

