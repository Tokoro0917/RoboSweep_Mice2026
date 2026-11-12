/*
 * PL_sensor.c
 *
 *  Created on: Jun 1, 2022
 *      Author: sf199
 */


#include <PL_sensor.h>
#include "adc.h"
#include "dma.h"
#include "gpio.h"//sensorLED用
#include "PL_Motor.h"
//#include "Maze.h"



uint16_t g_ADCBuffer[5];
char AD_step;

uint16_t g_sensor_on[4];
uint16_t g_sensor_off[4];
uint16_t g_sensor[4][4];


float g_V_batt;
int wallstatus[2]={0,0};
int prewallstatus[2] = {0,0};
int walls = 0;
float nowl = 0;
float nowr = 0;
float nowf1 = 0;
float nowf2 = 0;
float pastl = 0;
float pastr = 0;
float listl[S_COUNT];
float listr[S_COUNT];
float listf1[S_COUNT];
float listf2[S_COUNT];
int avecount = 0;




/*******************************************************************/
/*	電圧の取得			(pl_getbatt)	*/
/*******************************************************************/
/*	戻り値に電圧を返す．(AD変換単体)						*/
/*******************************************************************/
float pl_getbatt(void){
	 float batt;
	 uint16_t battAD;


	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 50);
	battAD = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	batt = 3.3  * (float) battAD / 4095 * (104.0 + 22.0) / 22.0;


return batt;
}

/*******************************************************************/
/*	callback用関数			(pl_callback_getSensor)	*/
/*******************************************************************/
/*	DMAがスタートしたら実行するコード(AD変換複数)					*/
/*******************************************************************/
void pl_callback_getSensor(void) {
uint16_t V_battAD;

int i,j;
HAL_ADC_Stop_DMA(&hadc1);
switch (AD_step) {
case 0:
HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin,GPIO_PIN_RESET);
for (j = 0; j <= 500; j++) {
}
break;
case 1:
g_sensor_on[0] = g_ADCBuffer[1];
g_sensor_on[1] = g_ADCBuffer[2];
g_sensor_off[2] = g_ADCBuffer[3];
g_sensor_off[3] = g_ADCBuffer[4];
HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin,GPIO_PIN_SET);
for (j = 0; j <= 500; j++) {
}
break;
case 2:
g_sensor_off[0] = g_ADCBuffer[1];
g_sensor_off[1] = g_ADCBuffer[2];
g_sensor_on[2] = g_ADCBuffer[3];
g_sensor_on[3] = g_ADCBuffer[4];
HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin,GPIO_PIN_RESET);
for (j = 0; j <= 10; j++) {
}
break;
}

V_battAD = g_ADCBuffer[0];
g_V_batt = 3.3 * (float) V_battAD / 1023 * (100.0 + 47.0) / 47.0;
AD_step++;

if (AD_step != 3) {
HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,
sizeof(g_ADCBuffer) / sizeof(uint16_t));

} else {
AD_step = 0;
for(i=0;i<4;i++){
	g_sensor[i][0]=g_sensor_on[i]-g_sensor_off[i];
}
}

}

float wallcontrol(float l,float r){
	float re = 0;
	float diffl = l - pastl;
	float diffr = r - pastr;

	if(l < THRESHOLD_L2 || diffl > THRESHOLD_L_DIFF){
		//printf("l \n\r");
		wallstatus[0] = 1;
	}
	else{
		wallstatus[0] = 0;
	}

	if(r < THRESHOLD_R2 || diffr > THRESHOLD_R_DIFF){
		//printf("r \n\r");
		wallstatus[1] = 1;

	}
	else{
		wallstatus[1] = 0;
	}


//遠心力補正
	if((isslalom && isdash > 1) ){
		float eR = 0;
		float eL = 0;
		if(direc == ROTATE_LEFT){
			eR = 0;
			eL = FPOWER_GAIN*Gv*Gv;
		}
		else{
			eR = FPOWER_GAIN*Gv*Gv;
			eL = 0;

		}
	if( wallstatus[0] == 0 && wallstatus[1] == 0){
		walls = 0;
		re = SENSOR_GAIN * ((l - CENTER_L+eL) - (r-CENTER_R+eR));
	}
	else if(wallstatus[0] == 1 && wallstatus[1] == 0){
		walls = 1;
		re = SENSOR_GAIN * (-2*(r - CENTER_R+eR));

	}
	else if(wallstatus[0] == 0 && wallstatus[1] == 1){
		walls = 2;
		re = SENSOR_GAIN * (2*(l - CENTER_L+eL));
		}
	else{
		walls = 3;
		re = 0;
	}
	}

	/*else if(iscircuit){
		walls = 2;
		re = SENSOR_GAIN * (2*(l - CENTER_L));
		}*/







	//通常壁制御
	else{
	if( wallstatus[0] == 0 && wallstatus[1] == 0){
			walls = 0;
			re = SENSOR_GAIN * ((l - CENTER_L) - (r-CENTER_R));
		}
		else if(wallstatus[0] == 1 && wallstatus[1] == 0){
			walls = 1;
			re = SENSOR_GAIN * (-2*(r - CENTER_R));

		}
		else if(wallstatus[0] == 0 && wallstatus[1] == 1){
			walls = 2;
			re = SENSOR_GAIN * (2*(l - CENTER_L));
			}
		else{
			walls = 3;
			re = 0;
		}
	}



















	prewallstatus[0] = wallstatus[0];
	prewallstatus[1] = wallstatus[1];
	//wallstatus[0] = 0;
	//wallstatus[1] = 0;

	pastl = l;
	pastr = r;

	//printf("%d",walls);
	walls = 0;
	return  re * (Gv/300);
}





int S_switch(){
	return ((nowf1 < SWITCH_1 && nowf2 > SWITCH_4) && (nowf1 < S_OVERFLOW && nowf2 < S_OVERFLOW));
}

int S_switch2(){
	return ((nowf1 < SWITCH_1l && nowf2 < SWITCH_4l));
}








/*******************************************************************/
/*	割り込み用動作関数(センサー取得)			(interupt_calSensor)	*/
/*******************************************************************/
/*	センサーの情報を取得する割り込み関数．						*/
/*******************************************************************/

void pl_interupt_getSensor(void){

		HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,
				sizeof(g_ADCBuffer) / sizeof(uint16_t));

}
