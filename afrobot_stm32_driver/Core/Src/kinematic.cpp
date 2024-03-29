/*
 * kinematic.cpp
 *
 *  Created on: Mar 15, 2023
 *      Author: krjar
 */


#include <kinematic.hpp>

//Defines for AFrobot dimensions
#define WHEEL_RADIUS						0.0485		// in m (0.097 diameter)
#define WHEEL_SEPARATION_HORIZONTAL			0.18		// robot center to wheel axis in m   ---
#define WHEEL_SEPARATION_VERTICAL			0.15		// robot center to wheel axis in m   |


double * getForwardKinematics(geometry_msgs::Twist *cmd)
{
	static double w[4]; // w[0] = FR, w[1] = FL, w[2] = BR, w[3] = BL
	double vx, vy, wz;

	//Get velocity command vx, vy, wz
	vx = cmd->linear.x ;
	vy = cmd->linear.y ;
	wz = cmd->angular.z ;

	//Calculate wheels angular velocity based on forward kinematics equation
	w[0] = (1/WHEEL_RADIUS) * (vx + vy + (WHEEL_SEPARATION_HORIZONTAL + WHEEL_SEPARATION_VERTICAL) * wz); //FR rad/s
	w[1] = (1/WHEEL_RADIUS) * (vx - vy - (WHEEL_SEPARATION_HORIZONTAL + WHEEL_SEPARATION_VERTICAL) * wz); //FL rad/s
	w[2] = (1/WHEEL_RADIUS) * (vx - vy + (WHEEL_SEPARATION_HORIZONTAL + WHEEL_SEPARATION_VERTICAL) * wz); //BR rad/s
	w[3] = (1/WHEEL_RADIUS) * (vx + vy - (WHEEL_SEPARATION_HORIZONTAL + WHEEL_SEPARATION_VERTICAL) * wz); //BL rad/s

	return w ;
}

double * getInverseKinematics(double *w)
{
	static double twist[3]; //twist[0] = vx, twist[1] = vy, twist[2] = wz
	double fr, fl, br, bl;

	//Get wheels angular velocity
	fr = w[0];
	fl = w[1];
	br = w[2];
	bl = w[3];

	//Calculate vx, vy, wz based on inverse kinematics equation
	twist[0] = (WHEEL_RADIUS/4) * (fr + fl + br + bl); // m/s
	twist[1] = (WHEEL_RADIUS/4) * (fr - fl - br + bl); // m/s
	twist[2] = (WHEEL_RADIUS/((WHEEL_SEPARATION_HORIZONTAL + WHEEL_SEPARATION_VERTICAL)*4)) * (fr - fl + br - bl); // rad/s

	return twist ;
}
