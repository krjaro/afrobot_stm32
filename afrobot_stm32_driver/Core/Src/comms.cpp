/*
 * comms.cpp
 *
 *  Created on: Mar 27, 2023
 *      Author: krjar
 */


#include <comms.hpp>



ros::NodeHandle nh ;


geometry_msgs::Twist odom ;
sensor_msgs::Imu imu ;

// Definitions for publishers and subscribers
ros::Publisher odometryPub("odom", &odom) ;
ros::Publisher imuPub("imu", &imu) ;
ros::Subscriber<geometry_msgs::Twist> commandSub("cmd_vel", &commandCallback);
ros::Subscriber<std_msgs::String> lcdSub("lcd", &lcdCallback);


uint8_t m_u8_uartBuffer = 43 ;

static geometry_msgs::Twist cmdVelHolder;

void commsInit(void)
{

//	  /* Create the queue(s) */

//	  MotorCommandQueueHandle = osMessageQueueNew (10, 96, &MotorCommandQueue_attributes);
//	  MotorPool = osMemoryPoolNew(1, 96, NULL);
//	  IMUDataQueueHandle = osMessageQueueNew (10, 344, &IMUDataQueue_attributes);
//	  LCDDataQueueHandle = osMessageQueueNew (10, 8, &LCDDataQueue_attributes);
//	  OdomDataQueueHandle = osMessageQueueNew (10, 96, &OdomDataQueue_attributes);
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
//	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // signalization that comms loop is executing


//	geometry_msgs::Twist testPub ;
////	testPub.header.stamp.sec = ros::Time::now().sec ;
//	testPub.angular.x = 666.0 ;
//
//	odometryPub.publish(&testPub);


	nh.spinOnce();
}

void commandCallback(const geometry_msgs::Twist &msg)
{
	cmdVelHolder.linear.x = msg.linear.x ;
	cmdVelHolder.linear.y = msg.linear.y ;
	cmdVelHolder.angular.z = msg.angular.z ;
}

void lcdCallback(const std_msgs::String &msg)
{
//	osMessageQueuePut(LCDDataQueueHandle, &msg, 0U, 0U);
}


void commsGetTwist(geometry_msgs::Twist *twist)
{

	twist->linear.x = cmdVelHolder.linear.x ;
	twist->linear.y = cmdVelHolder.linear.y ;
	twist->angular.z = cmdVelHolder.angular.z ;

}


// ------------------------- Change USART handling to ROSSerial
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	nh.getHardware()->reset_rbuf();
}

