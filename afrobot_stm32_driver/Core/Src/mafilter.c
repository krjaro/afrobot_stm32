/*
 * mafilter.c
 *
 *  Created on: May 3, 2023
 *      Author: krjar
 */

#include <mafilter.h>


void filterInit(filterType *f)
{
	for(int i = 0; i < windowLength; i++)
	{
		f->window[i] = 0 ;
	}
}

int16_t filterCalculate(filterType *f, int16_t value)
{
	int16_t sum = 0 ;
	int16_t filterVal = 0 ;

	// Shift window values to right
	for(int i = windowLength - 1; i > 0; i--)
	{
		f->window[i] = f->window[i - 1];
	}

	// Add new value to window
	f->window[0] = value ;

	// Calculate sum
	for(int i = 0; i < windowLength; i++)
	{
		sum = sum + f->window[i];
	}

	// Calculate average
	filterVal = (int16_t)(sum/windowLength);

	return filterVal;

}
