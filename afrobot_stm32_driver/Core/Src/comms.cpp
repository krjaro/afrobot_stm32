/*
 * comms.cpp
 *
 *  Created on: Mar 27, 2023
 *      Author: krjar
 */


#include <comms.hpp>



ros::NodeHandle nh ;


nav_msgs::Odometry odom ;
sensor_msgs::Imu imu ;

// Definitions for publishers and subscribers
ros::Publisher odometryPub("odom", &odom) ;
ros::Publisher imuPub("imu", &imu) ;
ros::Subscriber<geometry_msgs::TwistStamped> commandSub("cmd_vel", &commandCallback);
ros::Subscriber<std_msgs::String> lcdSub("lcd", &lcdCallback);

/* Definitions for Queues */
/* MotorCommandQueue */
osMessageQueueId_t MotorCommandQueueHandle;
const osMessageQueueAttr_t MotorCommandQueue_attributes = {
  .name = "MotorCommandQueue"
};
/* IMUDataQueue */
osMessageQueueId_t IMUDataQueueHandle;
const osMessageQueueAttr_t IMUDataQueue_attributes = {
  .name = "IMUDataQueue"
};
/* LCDDataQueue */
osMessageQueueId_t LCDDataQueueHandle;
const osMessageQueueAttr_t LCDDataQueue_attributes = {
  .name = "LCDDataQueue"
};
/* OdomDataQueue */
osMessageQueueId_t OdomDataQueueHandle;
const osMessageQueueAttr_t OdomDataQueue_attributes = {
  .name = "OdomDataQueue"
};

uint8_t m_u8_uartBuffer = 43 ;

void commsInit(void)
{
	  /* Create the queue(s) */
	  MotorCommandQueueHandle = osMessageQueueNew (10, 96, &MotorCommandQueue_attributes);
	  IMUDataQueueHandle = osMessageQueueNew (10, 344, &IMUDataQueue_attributes);
	  LCDDataQueueHandle = osMessageQueueNew (10, 8, &LCDDataQueue_attributes);
	  OdomDataQueueHandle = osMessageQueueNew (10, 776, &OdomDataQueue_attributes);
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
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // signalization that comms loop is executing





	nh.spinOnce();
}

void commandCallback(const geometry_msgs::TwistStamped &msg)
{
	osMessageQueuePut(MotorCommandQueueHandle, &msg, 0U, 0U);
}

void lcdCallback(const std_msgs::String &msg)
{
	osMessageQueuePut(LCDDataQueueHandle, &msg, 0U, 0U);
}


// ------------------------- Change USART handling to ROSSerial
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	nh.getHardware()->reset_rbuf();
}

