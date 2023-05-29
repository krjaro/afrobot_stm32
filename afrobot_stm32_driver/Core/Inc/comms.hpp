/*
 * comms.hpp
 *
 *  Created on: Mar 27, 2023
 *      Author: krjar
 */

#ifndef INC_COMMS_HPP_
#define INC_COMMS_HPP_

#include "stm32f4xx_hal.h"
#include <ros.h>
#include <cmsis_os.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <main.hpp>


void commsInit(void);
void commsSetup(void);
void commsLoop(void);
void odomPublish(geometry_msgs::Twist &);
void commandCallback(const geometry_msgs::Twist &);
void lcdCallback(const std_msgs::String &);
void commsGetTwist(geometry_msgs::Twist *);


#endif /* INC_COMMS_HPP_ */
