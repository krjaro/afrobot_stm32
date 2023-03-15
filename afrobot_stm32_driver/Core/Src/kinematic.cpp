/*
 * kinematic.cpp
 *
 *  Created on: Mar 15, 2023
 *      Author: krjar
 */




#include <main.hpp>
#include <kinematic.hpp>


#define WHEEL_DIAMETER						97		// in mm
#define WHEEL_SEPARATION_HORIZONTAL			180		// robot center to wheel axis in mm   ---
#define WHEEL_SEPARATION_VERTICAL			150		// robot center to wheel axis in mm   |


double * getForwardKinematics(geometry_msgs::TwistStamped *cmd)
{
	static double w[4]; // w[0] = FR, w[1] = FL, w[2] = BR, w[3] = BL
	double vx, vy, wz;

	vx = cmd->twist.linear.x ;
	vy = cmd->twist.linear.y ;
	wz = cmd->twist.angular.z ;

	w[0] = (1/WHEEL_DIAMETER) * (vx + vy + (WHEEL_SEPARATION_HORIZONTAL + WHEEL_SEPARATION_VERTICAL) * wz);
	w[1] = (1/WHEEL_DIAMETER) * (vx - vy - (WHEEL_SEPARATION_HORIZONTAL + WHEEL_SEPARATION_VERTICAL) * wz);
	w[2] = (1/WHEEL_DIAMETER) * (vx - vy + (WHEEL_SEPARATION_HORIZONTAL + WHEEL_SEPARATION_VERTICAL) * wz);
	w[3] = (1/WHEEL_DIAMETER) * (vx + vy - (WHEEL_SEPARATION_HORIZONTAL + WHEEL_SEPARATION_VERTICAL) * wz);

	return w ;
}
