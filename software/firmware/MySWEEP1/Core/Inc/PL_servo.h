/*
 * PL_servo.h
 *
 *  Created on: Aug 27, 2026
 *      Author: kodamakotaro
 */

#ifndef INC_PL_SERVO_H_
#define INC_PL_SERVO_H_


/*
 * Pl_servo.c
 *
 *  Created on: Aug 27, 2026
 *      Author: kodamakotaro
 */


#include "usart.h"
#include "gpio.h"
#include "PL_servo.h"


void servo_On();

void Torque (unsigned char ID, unsigned char data);/* 名前   : Move                                   */
/* 引数   : ID (Servo ID)                          */
/*        : Angle (Present Posion L&H)             */
/*        : Speed (Present Time L&H)               */
/* 戻り�?� : 無�?                                   */
/*-------------------------------------------------*/
void Move (unsigned char ID, int Angle, int Speed);


#endif /* INC_PL_SERVO_H_ */
