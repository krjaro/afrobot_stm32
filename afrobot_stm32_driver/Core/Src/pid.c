/*
 * pid.c
 *
 *  Created on: Mar 29, 2023
 *      Author: krjar
 */


#include <pid.h>


void pidInit(pid *p, float kp_init, float ki_init, float kd_init, int anti_windup_limit)
{
	p->previous_error = 0 ;
	p->total_error = 0 ;

	p->k_p = kp_init;
	p->k_i = ki_init;
	p->k_d = kd_init;

	p->anti_windup = anti_windup_limit;
}

void pidReset(pid *p)
{
	p->previous_error = 0 ;
	p->total_error = 0 ;
}

int pidCalculate(pid *p, float setpoint, float actual)
{
	float error, pTerm, iTerm, dTerm ;

	//Calculate errors
	error = setpoint - actual ;
	p->total_error += error ;

	//Calculate regulators components values
	pTerm = p->k_p * error ;
	iTerm = p->k_i * p->total_error ;
	dTerm = p->k_d * (error - p->previous_error) ;

	//Apply anti-windup to integral
	if (iTerm >= p->anti_windup)
		iTerm = p->anti_windup;
	else if (iTerm <= -p->anti_windup)
		iTerm = -p->anti_windup;

	//Save error for next iteration
	p->previous_error = error ;

	//Return regulator output as int
	return (int)(pTerm + iTerm + dTerm) ;
}
