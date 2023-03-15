/*
 * kinematic.hpp
 *
 *  Created on: Mar 15, 2023
 *      Author: krjar
 */

#ifndef INC_KINEMATIC_HPP_
#define INC_KINEMATIC_HPP_


#include "stm32f4xx_hal.h"
#include <geometry_msgs/TwistStamped.h>


double * getForwardKinematics(geometry_msgs::TwistStamped *);


#endif /* INC_KINEMATIC_HPP_ */
