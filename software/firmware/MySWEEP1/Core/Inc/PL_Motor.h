/*
 * PL_Motor.h
 *
 *  Created on: Jun 3, 2025
 *      Author: kodamakotaro
 */

#ifndef INC_PL_MOTOR_H_
#define INC_PL_MOTOR_H_

#define ROTATE_LEFT 0
#define ROTATE_RIGHT 1
#define SPD_1 1
#define SPD_2 2
#define SPD_3 3

extern float Gvl;
extern float Gvr;
extern float Gv;
extern float Gacc;
extern float Gx;
extern float per;
extern float Grv;
extern float Gracc;
extern float Grx;
extern int direc;
extern int isrotate;
extern int isdaikei;
extern int iswallcontrol;
extern int isback;
extern int isad;
extern int isslalom;
extern int isbigslalom;
extern int sv;
extern int isdash;
extern int svlist[];
extern int bsvlist[];


uint16_t calPWMCount(float velocity);
void motor_start_excitation(void);

void motor_start(void);

void motor_stop(void);

void motor_set_speedl();

void motor_set_speedr();

void motor_brake(int t);

void daikei(float fv,float maxv,float finv,float acc,float x);
void daikei2(float fv,float maxv,float finv,float acc,float x);
void daikei2_5(float fv,float maxv,float finv,float acc,float x);
void daikei3(float fv,float maxv,float finv,float acc,float x);
void nutral(float v,float x);
void daikei4(float fv,float maxv,float finv,float acc,float x);
void daikei5(float fv,float maxv,float finv,float acc,float x);
float cal_turnV(float W);

void daikeiR(float frv,float maxrv,float finrv,float racc,float rx,int direc1);
void daikeiR2(float frv,float maxrv,float finrv,float racc,float rx,int direc1);
void daikeiR2d(float frv,float maxrv,float finrv,float racc,float rx,int direc1,int spd);
void daikeiR3(float frv,float maxrv,float finrv,float racc,float rx,int direc1,int spd);

void modify_speed();




#endif /* INC_PL_MOTOR_H_ */
