# afrobot_stm32
This repository contains STM32 CubeIDE project with hardware driver for AF Robot which was built for research purpose as a part of my Master's Thesis.

It is implemented for STM32 NUCLEO-F429ZI board.

# Components

* UART communication with Jetson Nano using rosserial_stm32
* Rotary encoder handling
* PWM signal generation
* PID regulator algorithm
* Inverse and Forward Kinematics of 4 mecanum wheel robot
* Moving Average Filter algorithm
* IMU sensor readings through I2C (not finished yet)
