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
#include <cmsis_os.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

void commsInit(void);
void commsSetup(void);
void commsLoop(void);
void commandCallback(const geometry_msgs::TwistStamped &);
void lcdCallback(const std_msgs::String &);


#endif /* INC_COMMS_HPP_ */
