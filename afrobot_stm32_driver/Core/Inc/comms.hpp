/*
 * comms.hpp
 *
 *  Created on: Mar 27, 2023
 *      Author: krjar
 */

#ifndef INC_COMMS_HPP_
#define INC_COMMS_HPP_

#include <ros.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <geometry_msgs/TwistStamped.h>
#include <main.hpp>
#include <std_msgs/String.h>

typedef struct
{
	QueueHandle_t *cmdQHandle ;
	QueueHandle_t *odomQHandle ;
	QueueHandle_t *imuQHandle ;
}comms;


void commsInit(comms *, QueueHandle_t *, QueueHandle_t *, QueueHandle_t *);
void commsSetup(void);
void commsLoop(comms *);
void commandCallback(const geometry_msgs::TwistStamped &);
void lcdCallback(const std_msgs::String &);


#endif /* INC_COMMS_HPP_ */
