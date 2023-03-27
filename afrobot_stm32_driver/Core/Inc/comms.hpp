/*
 * comms.hpp
 *
 *  Created on: Mar 27, 2023
 *      Author: krjar
 */

#ifndef INC_COMMS_HPP_
#define INC_COMMS_HPP_

#include <main.hpp>
#include <ros.h>
#include <FreeRTOS.h>
#include <queue.h>


typedef struct
{
	QueueHandle_t *cmdQHandle ;
}comms;


void commsInit();
void commsSetup(void);
void commsLoop(void);


#endif /* INC_COMMS_HPP_ */
