/*
 * pid.h
 *
 *  Created on: Mar 29, 2023
 *      Author: krjar
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct
{
	float previous_error;
	float total_error;
	float k_p;
	float k_i;
	float k_d;
	int anti_windup;
}pid;

void pidInit(pid *, float, float, float, int);
void pidReset(pid *);
int pidCalculate(pid *, float, float);


#endif /* INC_PID_H_ */
