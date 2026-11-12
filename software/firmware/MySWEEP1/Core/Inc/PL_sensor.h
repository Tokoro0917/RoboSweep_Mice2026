/*
 * PL_sensor.h
 *
 *  Created on: Jun 1, 2022
 *      Author: sf199
 */

#ifndef INC_PL_SENSOR_H_
#define INC_PL_SENSOR_H_

#include "stm32f4xx_hal.h"

#define LIPO_LIMIT 11.3
#define CENTER_L 2500
#define CENTER_R 2500
#define CENTOR_F  200//250
#define CENTOR_F4 200//250
#define CENTOR_F2 2500
#define CENTOR_F3 2500
#define CENTOR_F1D 450//450
#define CENTOR_F4D 450//450
#define SENSOR_GAIN 0.02 //0.02
#define SENSOR_GAINF 0.01//0.01
#define ADDGAIN 3.5
#define FPOWER_GAIN 0.006
#define THRESHOLD_L 600
#define THRESHOLD_R 600
#define THRESHOLD_L2 1500
#define THRESHOLD_R2 1500
#define THRESHOLD_L3 600
#define THRESHOLD_R3 600
#define THRESHOLD_L_DIFF 100
#define THRESHOLD_R_DIFF 100
#define SWITCH_1 3500
#define SWITCH_4 3500
#define SWITCH_1l 100
#define SWITCH_4l 100
#define S_OVERFLOW 4500
#define S_COUNT 5




extern uint16_t g_ADCBuffer[5];
extern uint16_t g_sensor[4][4];




extern float g_V_batt;
extern int wallstatus[2];
extern int prewallstatus[2];
extern float pastl;
extern float pastr;
extern int walls;
extern float nowl;
extern float nowr;
extern float nowf1;
extern float nowf2;
extern float listl[S_COUNT];
extern float listr[S_COUNT];
extern float listf1[S_COUNT];
extern float listf2[S_COUNT];
extern int avecount;


float pl_getbatt();

void pl_callback_getSensor();

void pl_interupt_getSensor();

float wallcontrol(float l,float r);

int S_switch();

int S_switch2();


#endif /* INC_PL_SENSOR_H_ */
