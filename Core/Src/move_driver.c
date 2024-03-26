
/*
 * software_timer.c
 *
 *  Created on: Sep 21, 2023
 *      Author: HaHuyen
 */
#include "move_driver.h"

uint8_t speed_duty_cycle = 0;

void setSpeed(uint8_t dc, uint8_t duty_cycle) {
	speed_duty_cycle = duty_cycle;
	switch (dc){
	case 1:
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed_duty_cycle);
		break;
	case 2:
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, speed_duty_cycle);
		break;
	case 3:
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed_duty_cycle);
		break;
	case 4:
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, speed_duty_cycle);
		break;
	}
}

void dc1Forward(uint8_t duty_cycle){ //tiến
  HAL_GPIO_WritePin(IN1_DC1_GPIO_Port, IN1_DC1_Pin, 1);
  HAL_GPIO_WritePin(IN2_DC1_GPIO_Port, IN2_DC1_Pin, 0);
  setSpeed(1, duty_cycle);
}

void dc1Backwards(uint8_t duty_cycle){ //lùi
  HAL_GPIO_WritePin(IN1_DC1_GPIO_Port, IN1_DC1_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC1_GPIO_Port, IN2_DC1_Pin, 1);
  setSpeed(1, duty_cycle);
}

void dc1Stop(){
  HAL_GPIO_WritePin(IN1_DC1_GPIO_Port, IN1_DC1_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC1_GPIO_Port, IN2_DC1_Pin, 0);
}

void dc2Forward(uint8_t duty_cycle){ //tiến
  HAL_GPIO_WritePin(IN1_DC2_GPIO_Port, IN1_DC2_Pin, 1);
  HAL_GPIO_WritePin(IN2_DC2_GPIO_Port, IN2_DC2_Pin, 0);
  setSpeed(2, duty_cycle);
}

void dc2Backwards(uint8_t duty_cycle){ //lùi
  HAL_GPIO_WritePin(IN1_DC2_GPIO_Port, IN1_DC2_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC2_GPIO_Port, IN2_DC2_Pin, 1);
  setSpeed(2, duty_cycle);
}

void dc2Stop(){
  HAL_GPIO_WritePin(IN1_DC2_GPIO_Port, IN1_DC2_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC2_GPIO_Port, IN2_DC2_Pin, 0);
}

void dc3Forward(uint8_t duty_cycle){ //tiến
  HAL_GPIO_WritePin(IN1_DC3_GPIO_Port, IN1_DC3_Pin, 1);
  HAL_GPIO_WritePin(IN2_DC3_GPIO_Port, IN2_DC3_Pin, 0);
  setSpeed(3, duty_cycle);
}

void dc3Backwards(uint8_t duty_cycle){ //lùi
  HAL_GPIO_WritePin(IN1_DC3_GPIO_Port, IN1_DC3_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC3_GPIO_Port, IN2_DC3_Pin, 1);
  setSpeed(3, duty_cycle);
}

void dc3Stop(){
  HAL_GPIO_WritePin(IN1_DC3_GPIO_Port, IN1_DC3_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC3_GPIO_Port, IN2_DC3_Pin, 0);
}

void dc4Forward(uint8_t duty_cycle){ //tiến
  HAL_GPIO_WritePin(IN1_DC4_GPIO_Port, IN1_DC4_Pin, 1);
  HAL_GPIO_WritePin(IN2_DC4_GPIO_Port, IN2_DC4_Pin, 0);
  setSpeed(4, duty_cycle);
}

void dc4Backwards(uint8_t duty_cycle){ //lùi
  HAL_GPIO_WritePin(IN1_DC4_GPIO_Port, IN1_DC4_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC4_GPIO_Port, IN2_DC4_Pin, 1);
  setSpeed(4, duty_cycle);
}

void dc4Stop(){
  HAL_GPIO_WritePin(IN1_DC4_GPIO_Port, IN1_DC4_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC4_GPIO_Port, IN2_DC4_Pin, 0);
}

void stop(){
	dc1Stop();
	dc2Stop();
	dc3Stop();
	dc4Stop();
}

void forward(){
	dc1Forward(60);
	dc2Forward(60);
	dc3Forward(60);
	dc4Forward(60);
}

void backwards(){
	dc1Backwards(60);
	dc2Backwards(60);
	dc3Backwards(60);
	dc4Backwards(60);
}

void frontLeft(){
	dc2Forward(60);
	dc3Forward(60);
	dc1Stop();
	dc4Stop();
//	stop();
}

void frontRight(){
	dc1Forward(60);
	dc4Forward(60);
	dc2Stop();
	dc3Stop();
//	stop();
}
void backLeft(){
	dc2Backwards(60);
	dc3Backwards(60);
	dc1Stop();
	dc4Stop();
//	stop();
}

void backRight(){
	dc1Backwards(60);
	dc4Backwards(60);
	dc2Stop();
	dc3Stop();
//	stop();
}

void right(){
	dc2Backwards(60);
	dc3Backwards(60);
	dc1Forward(60);
	dc4Forward(60);
//	stop();
}

void left(){
	dc1Backwards(60);
	dc4Backwards(60);
	dc2Forward(60);
	dc3Forward(60);
//	stop();
}

void rotateLeft(){
	dc1Backwards(60);
	dc3Backwards(60);
	dc2Forward(60);
	dc4Forward(60);
//	stop();
}

void rotateRight(){
	dc2Backwards(60);
	dc4Backwards(60);
	dc1Forward(60);
	dc3Forward(60);
//	stop();
}
