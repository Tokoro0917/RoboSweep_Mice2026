/*
 * PL_Motor.c
 *
 *  Created on: Jun 3, 2025
 *      Author: kodamakotaro
 */

#include "main.h"
#include "tim.h"
#include "math.h"


#define MOTOR_L_FORWARD 0
#define MOTOR_L_BACKWARD 1
#define MOTOR_R_FORWARD 1
#define MOTOR_R_BACKWARD 0
#define ROTATE_LEFT 0
#define ROTATE_RIGHT 1
#define TRUE 1
#define FALSE 0






float Gvl = 0;
float Gvr = 0;
float Gv = 0;
float Gacc = 0;
float Gx = 0;
float Grv = 0;
float Gracc = 0;
float Grx = 0;
float per = 0.001;
float tirediameter = 53;
float tirebetween = 85;
int direc = ROTATE_LEFT;
int isrotate = FALSE;
int isdaikei = FALSE;
int iswallcontrol = FALSE;
int isback = FALSE;
int isad = FALSE;
int isslalom = FALSE;
int isbigslalom = FALSE;
int isdash = FALSE;
int sv = 500;
int svlist[] = {600,700,800};
int bsvlist[] = {800,900,1000};

//割り込み 速度→パルス変換
uint16_t calPWMCount(float velocity) {
	//mm s
	float rawCount;
	uint16_t PWMCount;

	if (velocity == 0) {
		return UINT16_MAX - 1;
	}

	rawCount = (10000000 * 0.9 * M_PI / 180 * tirediameter / 2 / fabs(velocity)) + 0.5;
	if (rawCount >= (float)UINT16_MAX || rawCount < 1) {
		PWMCount = UINT16_MAX - 1;
	} else {
		PWMCount = (uint16_t)rawCount - 1;
	}

	return PWMCount;
}

//モーター励磁
void motor_start_excitation(void){
	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_SET);
	//励磁
	HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_SET);
	HAL_Delay(3);
	HAL_GPIO_WritePin(MD_RESET_GPIO_Port,MD_RESET_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_CW_CCW_L_GPIO_Port,MOTOR_CW_CCW_L_Pin,MOTOR_L_FORWARD);
	HAL_GPIO_WritePin(MOTOR_CW_CCW_R_GPIO_Port,MOTOR_CW_CCW_R_Pin,MOTOR_R_FORWARD);
}


//motor回転スタート
void motor_start(void){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);//MOTOR回転
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}
//motor回転ストップ
void motor_stop(void){
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port,MOTOR_ENABLE_Pin,GPIO_PIN_RESET);
}

void motor_brake(int t){
	motor_stop();
	//Maze_output(&Mazedata);
	//Maze_output2(&Mazedata);
	motor_start_excitation();
	wait_ms(t);
	motor_start();
}


void motor_set_speedl(){//c1 L c2 R
	float c1 = Gvl;
	uint16_t s1 = 0;
	if(!isdaikei && isad){
			c1 = 0;
		}
	if (c1<0 || isback){
		s1 = calPWMCount(-c1);
		HAL_GPIO_WritePin(MOTOR_CW_CCW_L_GPIO_Port,MOTOR_CW_CCW_L_Pin,MOTOR_L_BACKWARD);
	}
	else{
		s1 = calPWMCount(c1);
		HAL_GPIO_WritePin(MOTOR_CW_CCW_L_GPIO_Port,MOTOR_CW_CCW_L_Pin,MOTOR_L_FORWARD);
	}

	__HAL_TIM_SET_AUTORELOAD(&htim3,s1);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, s1 / 2);

}


void motor_set_speedr(){
	float c2 = Gvr;
	uint16_t s2 = 0;
	if(!isdaikei && isad){
		c2 = 0;
	}
	if (c2<0 || isback){
				s2 = calPWMCount(-c2);
				HAL_GPIO_WritePin(MOTOR_CW_CCW_R_GPIO_Port,MOTOR_CW_CCW_R_Pin,MOTOR_R_BACKWARD);
			}
	else{
				s2 = calPWMCount(c2);
				HAL_GPIO_WritePin(MOTOR_CW_CCW_R_GPIO_Port,MOTOR_CW_CCW_R_Pin,MOTOR_R_FORWARD);
			}
	__HAL_TIM_SET_AUTORELOAD(&htim8,s2);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4, s2 / 2);

}


void daikei(float fv,float maxv,float finv,float acc,float x) {//デフォルト
	isdaikei = TRUE;
	iswallcontrol = TRUE;
    Gv = fv;
    Gacc = acc;
    Gx = 0;
    float t1 = (maxv-fv)/acc;
    float s1 = (maxv+fv)*t1/2;
    float t3 = (maxv-finv)/acc;
    float s3 = (maxv+finv)*t3/2;
    float s2 = x - s1- s3;
    //printf("%f,%f\n",t1,t3);
    //printf("%f,%f,%f\n",s1,s3,s2);

    if (s1 + s3 > x) {
        maxv = sqrt((pow(fv,2)+pow(finv,2) + 2*acc*x)/2);
        t1 = (maxv-fv)/acc;
        s1 = (maxv+fv)*t1/2;
        t3 = (maxv-finv)/acc;
        s3 = (maxv+finv)*t3/2;
        s2 = 0;

        //printf("%f,%f\n",t1,t3);
        //printf("%f,%f,%f\n",s1,s3,s2);
    }

    //printf("%f\n\r",maxv);



    while (maxv >= Gv) {
    	//printf("A%f\n\r",Gv);
    }
    //printf("A%f\n\r",Gv);
    Gv = maxv;
    Gacc = 0;
    Gx = 0;

    while (s2 >= Gx){
    }
    //printf("B%f\n\r",Gv);

    Gacc = -acc;

    while (finv <= Gv) {
    	//printf("B%f\n\r",Gv);
    }
    //printf("C%f\n\r",Gv);
    Gv = finv;
    Gacc = 0;
    isdaikei = FALSE;
    iswallcontrol = FALSE;
    }


void daikei2(float fv,float maxv,float finv,float acc,float x) {//壁制御なし
	isdaikei = TRUE;
	//iswallcontrol = TRUE;
    Gv = fv;
    Gacc = acc;
    Gx = 0;
    float t1 = (maxv-Gv)/acc;
    float s1 = (maxv+Gv)*t1/2;
    float t3 = (maxv-finv)/acc;
    float s3 = (maxv+finv)*t3/2;
    float s2 = x - s1- s3;
    //printf("%f,%f\n",t1,t3);
    //printf("%f,%f,%f\n",s1,s3,s2);

    if (s1 + s3 > x) {
        maxv = sqrt((pow(fv,2)+pow(finv,2) + 2*acc*x)/2);
        t1 = (maxv-Gv)/acc;
        s1 = (maxv+Gv)*t1/2;
        t3 = (maxv-finv)/acc;
        s3 = (maxv+finv)*t3/2;
        s2 = 0;

        //printf("%f,%f\n",t1,t3);
        //printf("%f,%f,%f\n",s1,s3,s2);
    }

    //printf("%f\n\r",maxv);



    while (maxv > Gv) {
    	//printf("A%f\n\r",Gv);
    }
    //printf("A%f\n\r",Gv);

    Gacc = 0;
    Gv = maxv;
    Gx = 0;


    while (s2 > Gx){
    }
    //printf("B%f\n\r",Gv);

    Gacc = -acc;

    while (finv < Gv) {
    	//printf("B%f\n\r",Gv);
    }
    //printf("C%f\n\r",Gv);
    Gv = finv;
    Gacc = 0;
    isdaikei = FALSE;
    iswallcontrol = FALSE;
    }

/*


void daikei2_5(float fv,float maxv,float finv,float acc,float x) { //前壁制御　壁制御なし
	isdaikei = TRUE;
	//iswallcontrol = TRUE;
    Gv = fv;
    Gacc = acc;
    Gx = 0;
    float t1 = (maxv-Gv)/acc;
    float s1 = (maxv+Gv)*t1/2;
    float t3 = (maxv-finv)/acc;
    float s3 = (maxv+finv)*t3/2;
    float s2 = x - s1 - s3;
    //printf("%f,%f\n",t1,t3);
    //printf("%f,%f,%f\n",s1,s3,s2);

    if (s1 + s3 > x) {
        maxv = sqrt((pow(fv,2)+pow(finv,2) + 2*acc*x)/2);
        t1 = (maxv-Gv)/acc;
        s1 = (maxv+Gv)*t1/2;
        t3 = (maxv-finv)/acc;
        s3 = (maxv+finv)*t3/2;
        s2 = 0;

        //printf("%f,%f\n",t1,t3);
        //printf("%f,%f,%f\n",s1,s3,s2);
    }

    //printf("%f\n\r",maxv);



    while (maxv > Gv) {
    	//printf("A%f\n\r",Gv);
    	//Maze_generatemap(&Mazedata,&Mazeq);
    	//Maze_updateA(&Mazedata);
    }
    //printf("A%f\n\r",Gv);

    Gacc = 0;
    Gv = maxv;
    Gx = 0;

    while ((s2 > Gx && (nowf1 < CENTOR_F1D || nowf2 < CENTOR_F4D)) || s2*0.70 > Gx){
    	//Maze_generatemap(&Mazedata,&Mazeq);
    	//Maze_updateA(&Mazedata);
    }
    //printf("B%f\n\r",Gv);

    Gacc = -acc;

    while (finv < Gv && (nowf1 < CENTOR_F1D || nowf2 < CENTOR_F4D)) {
    	//printf("B%f\n\r",Gv);
    	//Maze_generatemap(&Mazedata,&Mazeq);
    	//Maze_updateA(&Mazedata);
    }
    //printf("C%f\n\r",Gv);
    Gv = finv;
    Gacc = 0;
    isdaikei = FALSE;
    //iswallcontrol = FALSE;
    }




void daikei3(float fv,float maxv,float finv,float acc,float x) { //前壁制御のみ
	isdaikei = TRUE;
	iswallcontrol = TRUE;
    Gv = fv;
    Gacc = acc;
    Gx = 0;
    float t1 = (maxv-Gv)/acc;
    float s1 = (maxv+Gv)*t1/2;
    float t3 = (maxv-finv)/acc;
    float s3 = (maxv+finv)*t3/2;
    float s2 = x - s1 - s3;
    //printf("%f,%f\n",t1,t3);
    //printf("%f,%f,%f\n",s1,s3,s2);

    if (s1 + s3 > x) {
        maxv = sqrt((pow(fv,2)+pow(finv,2) + 2*acc*x)/2);
        t1 = (maxv-Gv)/acc;
        s1 = (maxv+Gv)*t1/2;
        t3 = (maxv-finv)/acc;
        s3 = (maxv+finv)*t3/2;
        s2 = 0;

        //printf("%f,%f\n",t1,t3);
        //printf("%f,%f,%f\n",s1,s3,s2);
    }

    //printf("%f\n\r",maxv);



    while (maxv > Gv) {
    	//printf("A%f\n\r",Gv);
    	//Maze_generatemap(&Mazedata,&Mazeq);
    	//Maze_updateA(&Mazedata);
    }
    //printf("A%f\n\r",Gv);

    Gacc = 0;
    Gv = maxv;
    Gx = 0;

    while ((s2 > Gx && (nowf1 < CENTOR_F1D || nowf2 < CENTOR_F4D)) || s2*0.70 > Gx){
    	//Maze_generatemap(&Mazedata,&Mazeq);
    	//Maze_updateA(&Mazedata);
    }
    //printf("B%f\n\r",Gv);

    Gacc = -acc;

    while (finv < Gv && (nowf1 < CENTOR_F1D || nowf2 < CENTOR_F4D)) {
    	//printf("B%f\n\r",Gv);
    	//Maze_generatemap(&Mazedata,&Mazeq);
    	//Maze_updateA(&Mazedata);
    }
    //printf("C%f\n\r",Gv);
    Gv = finv;
    Gacc = 0;
    isdaikei = FALSE;
    iswallcontrol = FALSE;
    }


void nutral(float v,float x){
	Gv = v;
	Gx = 0;
	Gacc = 0;
	isdaikei = TRUE;
	iswallcontrol = TRUE;
	 while (x > Gx){
	    	Maze_generatemap(&Mazedata,&Mazeq,M_GOALX,M_GOALY);
	    }
	Gv = v;
	isdaikei = FALSE;
	iswallcontrol = FALSE;
}




void daikei4(float fv,float maxv,float finv,float acc,float x) {//前壁制御,壁切れ検知あり

	isdaikei = TRUE;
	iswallcontrol = TRUE;
    Gv = fv;
    Gacc = acc;
    Gx = 0;
    float t1 = (maxv-Gv)/acc;
    float s1 = (maxv+Gv)*t1/2;
    float t3 = (maxv-finv)/acc;
    float s3 = (maxv+finv)*t3/2;
    float s2 = x - s1 - s3;
    //printf("%f,%f\n",t1,t3);
    //printf("%f,%f,%f\n",s1,s3,s2);

    if (s1 + s3 > x) {
        maxv = sqrt((pow(fv,2)+pow(finv,2) + 2*acc*x)/2);
        t1 = (maxv-Gv)/acc;
        s1 = (maxv+Gv)*t1/2;
        t3 = (maxv-finv)/acc;
        s3 = (maxv+finv)*t3/2;
        s2 = 0;

        //printf("%f,%f\n",t1,t3);
        //printf("%f,%f,%f\n",s1,s3,s2);
    }

    //printf("%f\n\r",maxv);
    //int nowwl = wallstatus[0];
    //int nowwr = wallstatus[1];

    int nowwl = (nowl < THRESHOLD_L3);
    int nowwr = (nowr < THRESHOLD_R3);
    int nowwln = nowwl;
    int nowwrn = nowwr;


    while (maxv > Gv) {
    	//printf("A%f\n\r",Gv);
    }
    //printf("A%f\n\r",Gv);

    Gacc = 0;
    Gv = maxv;
        Gx = 0;

    while ((s2 > Gx && (nowf1 < CENTOR_F1D || nowf2 < CENTOR_F4D)) || s2*0.70 > Gx){
    	  nowwln = (nowl < THRESHOLD_L3);
    	  nowwrn = (nowr < THRESHOLD_R3);
    	if((nowwl == 0 && nowwln == 1) || (nowwr == 0 && nowwrn == 1)){ //wallstatus 0 1
    		daikei(Gv,500,500,2500,62);
    		blink(1,10);
    		break;
    	}
    }
    //printf("B%f\n\r",Gv);

    Gacc = -acc;

    while (finv < Gv && (nowf1 < CENTOR_F1D || nowf2 < CENTOR_F4D)) {
    	 nowwln = (nowl < THRESHOLD_L3);
    	 nowwrn = (nowr < THRESHOLD_R3);
    	//printf("B%f\n\r",Gv);
    	if((nowwl == 0 && nowwln == 1) || (nowwr == 0 && nowwrn == 1)){
    		daikei(Gv,500,500,2500,62);
    		blink(1,10);
    	    		break;
    	    	}
    }
    //printf("C%f\n\r",Gv);
    Gv = finv;
    Gacc = 0;
    isdaikei = FALSE;
    iswallcontrol = FALSE;
    }



void daikei5(float fv,float maxv,float finv,float acc,float x) {//??
	int nowwl = wallstatus[0];
	int nowwr = wallstatus[1];

	int ischeckwall = FALSE;

	isdaikei = TRUE;
	iswallcontrol = TRUE;
    Gv = fv;
    Gacc = acc;
    Gx = 0;
    float t1 = (maxv-Gv)/acc;
    float s1 = (maxv+Gv)*t1/2;
    float t3 = (maxv-finv)/acc;
    float s3 = (maxv+finv)*t3/2;
    float s2 = x - s1- s3;
    //printf("%f,%f\n",t1,t3);
    //printf("%f,%f,%f\n",s1,s3,s2);

    if (s1 + s3 > x) {
        maxv = sqrt((pow(fv,2)+pow(finv,2) + 2*acc*x)/2);
        t1 = (maxv-Gv)/acc;
        s1 = (maxv+Gv)*t1/2;
        t3 = (maxv-finv)/acc;
        s3 = (maxv+finv)*t3/2;
        s2 = 0;

        //printf("%f,%f\n",t1,t3);
        //printf("%f,%f,%f\n",s1,s3,s2);
    }

    //printf("%f\n\r",maxv);

    float xc = x - s1 - 100;
    if(xc < 0){
    	xc = 0;
    }



    while (maxv > Gv) {
    	//printf("A%f\n\r",Gv);
    }
    //printf("A%f\n\r",Gv);

    Gacc = 0;
    Gv = maxv;
        Gx = 0;

    while ((s2 > Gx && (nowf1 < CENTOR_F1D || nowf2 < CENTOR_F4D)) || s2*0.70 > Gx){
    	if(Gx > xc && ischeckwall == FALSE){
    		nowwl = wallstatus[0];
    		nowwr = wallstatus[1];
    		ischeckwall == TRUE;
    		blink(1,10);
    	}
    	if(ischeckwall && (nowwl != wallstatus[0] || nowwr !=  wallstatus[1])){
    		//daikei(Gv,300,300,500,90);

    		//daikei(Gv,Gv,Gv,acc,20);
    		break;
    	}
    }
    //printf("B%f\n\r",Gv);

    Gacc = -acc;

    while (finv < Gv && (nowf1 < CENTOR_F1D || nowf2 < CENTOR_F4D)) {
    	//printf("B%f\n\r",Gv);
    	if(Gx > xc && ischeckwall == FALSE){
    	    		nowwl = wallstatus[0];
    	    		nowwr = wallstatus[1];
    	    		ischeckwall == TRUE;
    	    		blink(1,10);
    	    	}
    	    	if(ischeckwall && (nowwl != wallstatus[0] || nowwr !=  wallstatus[1])){
    	    		//daikei(Gv,300,300,500,90);
    	    		//daikei(Gv,Gv,Gv,2500,62);
    	    		//blink(1,10);
    	    		//daikei(Gv,Gv,Gv,acc,20);
    	    		break;
    	    	}
    }
    //printf("C%f\n\r",Gv);
    Gv = finv;
    Gacc = 0;
    isdaikei = FALSE;
    iswallcontrol = FALSE;
    ischeckwall = FALSE;
    }


void S_kasoku(float fv,float maxv,float finv,float x,float facc,float maxacc,float jerk){

}

*/






float cal_turnV(float W) {
    float TurnV = ((float) tirebetween * M_PI / 360) * W;
    return TurnV;
}

//frv 150 max 300 frv 150


void daikeiR(float frv,float maxrv,float finrv,float racc,float rx,int direc1){//ターン
	    isdaikei = TRUE;
        isrotate = TRUE;
        iswallcontrol = FALSE;
	    direc = direc1;
	    Gv = cal_turnV(frv);
	    Gacc = cal_turnV(racc);
	    Gx = 0;
	    float fv = cal_turnV(frv);
		float maxv = cal_turnV(maxrv);
		float finv = cal_turnV(finrv);
		float acc  = cal_turnV(racc);
		float x = cal_turnV(rx);
	    float t1 = (maxv-Gv)/acc;
	    float s1 = (maxv+Gv)*t1/2;
	    float t3 = (maxv-finv)/acc;
	    float s3 = (maxv+finv)*t3/2;
	    float s2 = x - s1- s3;
	    //printf("%f,%f\n",t1,t3);
	    //printf("%f,%f,%f\n",s1,s3,s2);

	    if (s1 + s3 > x) {
	        maxv = sqrt((pow(fv,2)+pow(finv,2) + 2*acc*x)/2);
	        t1 = (maxv-Gv)/acc;
	        s1 = (maxv+Gv)*t1/2;
	        t3 = (maxv-finv)/acc;
	        s3 = (maxv+finv)*t3/2;
	        s2 = 0;

	        //printf("%f,%f\n",t1,t3);
	        //printf("%f,%f,%f\n",s1,s3,s2);
	    }

	    //printf("%f\n\r",maxv);



	    while (maxv > Gv) {
	    	//printf("A%f\n\r",Grv);
	    }
	    //printf("A%f\n\r",Gv);

	    Gacc = 0;
	    Gv = maxv;
	        Gx = 0;

	    while (s2 > Gx){
	    }
	    //printf("B%f\n\r",Gv);

	    Gacc = -acc;

	    while (finv < Gv) {
	    	//printf("B%f\n\r",Grv);
	    }
	    //printf("C%f\n\r",Gv);

	    Gv = finv;
	    Gacc = 0;
	    isrotate = FALSE;
	    isdaikei = FALSE;

}


void daikeiR2(float frv,float maxrv,float finrv,float racc,float rx,int direc1){//スラローム
	    isad = TRUE; //無視して　ようわからんフラグ
	    isdaikei = TRUE; //台形加速フラグ
        isrotate = TRUE; //旋回フラグ
        isslalom = TRUE; //スラロームフラグ
        iswallcontrol = FALSE; //壁制御フラグ
	    direc = direc1;
	    //sv = Gv;
	    Gv = cal_turnV(frv);
	    Gacc = cal_turnV(racc);
	    Gx = 0;
	    float fv = cal_turnV(frv);
		float maxv = cal_turnV(maxrv);
		float finv = cal_turnV(finrv);
		float acc  = cal_turnV(racc);
		float x = cal_turnV(rx);
	    float t1 = (maxv-Gv)/acc;
	    float s1 = (maxv+Gv)*t1/2;
	    float t3 = (maxv-finv)/acc;
	    float s3 = (maxv+finv)*t3/2;
	    float s2 = x - s1- s3;
	    //printf("%f,%f\n",t1,t3);
	    //printf("%f,%f,%f\n",s1,s3,s2);

	    if (s1 + s3 > x) {
	        maxv = sqrt((pow(fv,2)+pow(finv,2) + 2*acc*x)/2);
	        t1 = (maxv-Gv)/acc;
	        s1 = (maxv+Gv)*t1/2;
	        t3 = (maxv-finv)/acc;
	        s3 = (maxv+finv)*t3/2;
	        s2 = 0;

	        //printf("%f,%f\n",t1,t3);
	        //printf("%f,%f,%f\n",s1,s3,s2);
	    }

	    //printf("%f\n\r",maxv);



	    while (maxv > Gv) {
	    	//printf("A%f\n\r",Grv);
	    }
	    //printf("A%f\n\r",Gv);

	    Gacc = 0;
	    Gv = maxv;
	        Gx = 0;

	    while (s2 > Gx){
	    }
	    //printf("B%f\n\r",Gv);

	    Gacc = -acc;

	    while (finv < Gv) {
	    	//printf("B%f\n\r",Grv);
	    }
	    //printf("C%f\n\r",Gv);

	    Gv = finv;
	    Gacc = 0;
	    isrotate = FALSE;
	    isdaikei = FALSE;
	    isslalom = FALSE;
	    isad = FALSE;

}

void daikeiR2d(float frv,float maxrv,float finrv,float racc,float rx,int direc1,int spd){//スラロームdash

	    isad = TRUE;
	    isdaikei = TRUE;
        isrotate = TRUE;
        isslalom = TRUE;
        isdash = spd;
        iswallcontrol = FALSE;
	    direc = direc1;
	    //sv = Gv;
	    Gv = cal_turnV(frv);
	    Gacc = cal_turnV(racc);
	    Gx = 0;
	    float fv = cal_turnV(frv);
		float maxv = cal_turnV(maxrv);
		float finv = cal_turnV(finrv);
		float acc  = cal_turnV(racc);
		float x = cal_turnV(rx);
	    float t1 = (maxv-Gv)/acc;
	    float s1 = (maxv+Gv)*t1/2;
	    float t3 = (maxv-finv)/acc;
	    float s3 = (maxv+finv)*t3/2;
	    float s2 = x - s1- s3;
	    //printf("%f,%f\n",t1,t3);
	    //printf("%f,%f,%f\n",s1,s3,s2);

	    if (s1 + s3 > x) {
	        maxv = sqrt((pow(fv,2)+pow(finv,2) + 2*acc*x)/2);
	        t1 = (maxv-Gv)/acc;
	        s1 = (maxv+Gv)*t1/2;
	        t3 = (maxv-finv)/acc;
	        s3 = (maxv+finv)*t3/2;
	        s2 = 0;

	        //printf("%f,%f\n",t1,t3);
	        //printf("%f,%f,%f\n",s1,s3,s2);
	    }

	    //printf("%f\n\r",maxv);



	    while (maxv > Gv) {
	    	//printf("A%f\n\r",Grv);
	    }
	    //printf("A%f\n\r",Gv);

	    Gacc = 0;
	    Gv = maxv;
	        Gx = 0;

	    while (s2 > Gx){
	    }
	    //printf("B%f\n\r",Gv);

	    Gacc = -acc;

	    while (finv < Gv) {
	    	//printf("B%f\n\r",Grv);
	    }
	    //printf("C%f\n\r",Gv);

	    Gv = finv;
	    Gacc = 0;
	    isrotate = FALSE;
	    isdaikei = FALSE;
	    isslalom = FALSE;
	    isad = FALSE;
	    isdash = FALSE;

}


void daikeiR3(float frv,float maxrv,float finrv,float racc,float rx,int direc1,int spd){//大回りスラローム
	    isad = TRUE;
	    isdaikei = TRUE;
        isrotate = TRUE;
        isslalom = TRUE;
        isbigslalom = TRUE;
        iswallcontrol = FALSE;
        isdash = spd;
	    direc = direc1;
	    //sv = Gv;
	    Gv = cal_turnV(frv);
	    Gacc = cal_turnV(racc);
	    Gx = 0;
	    float fv = cal_turnV(frv);
		float maxv = cal_turnV(maxrv);
		float finv = cal_turnV(finrv);
		float acc  = cal_turnV(racc);
		float x = cal_turnV(rx);
	    float t1 = (maxv-Gv)/acc;
	    float s1 = (maxv+Gv)*t1/2;
	    float t3 = (maxv-finv)/acc;
	    float s3 = (maxv+finv)*t3/2;
	    float s2 = x - s1- s3;
	    //printf("%f,%f\n",t1,t3);
	    //printf("%f,%f,%f\n",s1,s3,s2);

	    if (s1 + s3 > x) {
	        maxv = sqrt((pow(fv,2)+pow(finv,2) + 2*acc*x)/2);
	        t1 = (maxv-Gv)/acc;
	        s1 = (maxv+Gv)*t1/2;
	        t3 = (maxv-finv)/acc;
	        s3 = (maxv+finv)*t3/2;
	        s2 = 0;

	        //printf("%f,%f\n",t1,t3);
	        //printf("%f,%f,%f\n",s1,s3,s2);
	    }

	    //printf("%f\n\r",maxv);



	    while (maxv > Gv) {
	    	//printf("A%f\n\r",Grv);
	    }
	    //printf("A%f\n\r",Gv);

	    Gacc = 0;
	    Gv = maxv;
	        Gx = 0;

	    while (s2 > Gx){
	    }
	    //printf("B%f\n\r",Gv);

	    Gacc = -acc;

	    while (finv < Gv) {
	    	//printf("B%f\n\r",Grv);
	    }
	    //printf("C%f\n\r",Gv);

	    Gv = finv;
	    Gacc = 0;
	    isrotate = FALSE;
	    isdaikei = FALSE;
	    isslalom = FALSE;
	    isbigslalom = FALSE;
	    isad = FALSE;
	    isdash = FALSE;

}

void modify_speed(){
	if(isrotate){
		if(isslalom){
			if(isbigslalom){
				sv = bsvlist[isdash-1];
			}
			else if(isdash){
				sv = svlist[isdash-1];
			}
			else{
				sv = 500;
			}
		//sv = (isbigslalom) ? 800 : 500;
		if(direc){
			Gvl = -Gv + sv;
			Gvr = Gv + sv;
		}
		else{
			Gvl = Gv + sv;
			Gvr = -Gv + sv;
		}
		}

		else{
			if(direc){
					Gvl = -Gv;
					Gvr = Gv;
				}
				else{
					Gvl = Gv;
					Gvr = -Gv;
				}
		}

	}

	else{
		Gvr = Gv;
		Gvl = Gv;
	}

}

