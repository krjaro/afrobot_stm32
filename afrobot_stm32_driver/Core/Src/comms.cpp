/*
 * comms.cpp
 *
 *  Created on: Mar 27, 2023
 *      Author: krjar
 */


#include <comms.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>


ros::NodeHandle nh ;


nav_msgs::Odometry odom ;
sensor_msgs::Imu imu ;


ros::Publisher odometryPub("odom", &odom) ;
ros::Publisher imuPub("imu", &imu) ;
ros::Subscriber<geometry_msgs::TwistStamped> commandSub("cmd_vel", &commandCallback);
ros::Subscriber<std_msgs::String> lcdSub("lcd", &lcdCallback);

uint8_t m_u8_uartBuffer = 43 ;

void commsInit(comms *c, QueueHandle_t *cmd, QueueHandle_t *odo, QueueHandle_t *imu)
{
	c->cmdQHandle = cmd ;
	c->odomQHandle = odo ;
	c->imuQHandle = imu ;
}


void commsSetup(void)
{
	nh.initNode();
	nh.advertise(odometryPub);
	nh.advertise(imuPub);
	nh.subscribe(commandSub);
	nh.subscribe(lcdSub);
}

void commsLoop(comms *c)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // signalization that comms loop is executing





	nh.spinOnce();
}

void commandCallback(const geometry_msgs::TwistStamped &msg)
{

}

void lcdCallback(const std_msgs::String &msg)
{

}


// ------------------------- Change USART handling to ROSSerial
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->reset_rbuf();
}

