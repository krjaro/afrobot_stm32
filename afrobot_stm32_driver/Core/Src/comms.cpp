/*
 * comms.cpp
 *
 *  Created on: Mar 27, 2023
 *      Author: krjar
 */


#include <comms.hpp>



ros::NodeHandle nh ;

int watchdog = 0 ;

geometry_msgs::Twist odom ;
sensor_msgs::Imu imu ;

// Definitions for publishers and subscribers
ros::Publisher odometryPub("odom_stm32", &odom) ;
ros::Publisher imuPub("imu", &imu) ;
ros::Subscriber<geometry_msgs::Twist> commandSub("cmd_vel", &commandCallback);
ros::Subscriber<std_msgs::String> lcdSub("lcd", &lcdCallback);


uint8_t m_u8_uartBuffer = 43 ;

static geometry_msgs::Twist cmdVelHolder;

void commsInit(void)
{


}

void commsSetup(void)
{
	nh.initNode();
	nh.advertise(odometryPub);
	nh.advertise(imuPub);
	nh.subscribe(commandSub);
	nh.subscribe(lcdSub);
}

void commsLoop()
{
	watchdog += 1 ;

	if (watchdog > 3000)
		NVIC_SystemReset();

	nh.spinOnce();
}

void odomPublish(geometry_msgs::Twist &msg)
{
	odometryPub.publish(&msg);
}

void commandCallback(const geometry_msgs::Twist &msg)
{
	cmdVelHolder.linear.x = msg.linear.x ;
	cmdVelHolder.linear.y = msg.linear.y ;
	cmdVelHolder.angular.z = msg.angular.z ;
}

void lcdCallback(const std_msgs::String &msg)
{

}


void commsGetTwist(geometry_msgs::Twist *twist)
{
	twist->linear.x = cmdVelHolder.linear.x ;
	twist->linear.y = cmdVelHolder.linear.y ;
	twist->angular.z = cmdVelHolder.angular.z ;
}


// ------------------------- USART handling to ROSSerial
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	nh.getHardware()->flush();
	watchdog = 0 ;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	nh.getHardware()->reset_rbuf();
}

