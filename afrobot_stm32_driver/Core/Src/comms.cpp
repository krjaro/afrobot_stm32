/*
 * comms.cpp
 *
 *  Created on: Mar 27, 2023
 *      Author: krjar
 */


#include <comms.hpp>
#include <nav_msgs/Odometry.h>


ros::NodeHandle nh ;


nav_msgs::Odometry odom ;


ros::Publisher odometry("odom", &odom) ;

ros::Subscriber<geometry_msgs::TwistStamped> command("cmd_vel", &commandCallback);

void commsInit(comms *c, QueueHandle_t *cmd, QueueHandle_t *odo, QueueHandle_t *imu)
{
	c->cmdQHandle = cmd ;
	c->odomQHandle = odo ;
	c->imuQHandle = imu ;
}


void commsSetup(void)
{
	nh.initNode();
	nh.advertise(odometry);
}

void commsLoop(comms *c)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // signalization that comms loop is executing





	nh.spinOnce();
}

void commandCallback(const geometry_msgs::TwistStamped &msg)
{

}



// ------------------------- Change USART handling to ROSSerial
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->reset_rbuf();
}

