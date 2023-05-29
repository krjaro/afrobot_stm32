/*
 * imu.h
 *
 *  Created on: May 29, 2023
 *      Author: krjar
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_


/* Includes */
#include "stm32f4xx_hal.h"
#include "lsm6ds3tr-c_reg.h"

/* Defines */
#define SENSOR_BUS hi2c1
#define BOOT_TIME 15

static int32_t platform_write(I2C_HandleTypeDef *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(I2C_HandleTypeDef *handle, uint8_t reg, uint8_t *bufp, uint16_t len);




#endif /* INC_IMU_H_ */
