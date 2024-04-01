/*
 * move_driver.h
 *
 *  Created on: Mar 26, 2024
 *      Author: HaHuyen
 */

#ifndef INC_MOVE_DRIVER_H_
#define INC_MOVE_DRIVER_H_

#include "main.h"

void setSpeed(uint8_t dc, uint8_t duty_cycle);

void dc1Forward(uint8_t duty_cycle);//tiến
void dc1Backwards(uint8_t duty_cycle);//lùi
void dc1Stop();

void dc2Forward(uint8_t duty_cycle);
void dc2Backwards(uint8_t duty_cycle);
void dc2Stop();

void dc3Forward(uint8_t duty_cycle);
void dc3Backwards(uint8_t duty_cycle);
void dc3Stop();

void dc4Forward(uint8_t duty_cycle);
void dc4Backwards(uint8_t duty_cycle);
void dc4Stop();

//                   forward
//           frontLeft   |  frontRight          ↺: rotateLeft
//         left		  ---+----     right		↻: rotateRight
//           backLeft    |  backRight
//                  backwards
//

void stop();
void forward();
void backwards();
void frontLeft();
void frontRight();
void backLeft();
void backRight();
void right();
void left();
void rotateLeft();
void rotateRight();

#endif /* INC_MOVE_DRIVER_H_ */
