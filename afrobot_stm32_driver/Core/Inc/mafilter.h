/*
 * mafilter.h
 *
 *  Created on: May 3, 2023
 *      Author: krjar
 */

#ifndef INC_MAFILTER_H_
#define INC_MAFILTER_H_

#define windowLength 30

#include "stm32f4xx_hal.h"

typedef struct
{
	int16_t window[windowLength];
}filterType;


void filterInit(filterType *);
int16_t filterCalculate(filterType *, int16_t);

#endif /* INC_MAFILTER_H_ */
