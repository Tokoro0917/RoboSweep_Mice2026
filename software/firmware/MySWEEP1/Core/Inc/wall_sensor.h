/*
 * wall_sensor.h
 *
 * 赤外線の壁センサ 4 個と電池電圧を読む。
 * 1ms ごとに wall_sensor_start() を呼ぶと、LED を点けたとき・消したときの
 * 値を測って、その差（外の光の影響を引いた値）を更新する。
 */

#ifndef INC_WALL_SENSOR_H_
#define INC_WALL_SENSOR_H_

#include <stdint.h>

#define WALL_SENSOR_COUNT 4

/* 測定を始める（1ms の制御割り込みから呼ぶ） */
void wall_sensor_start(void);

/* ADC の DMA 完了割り込みから呼ぶ（adc.c） */
void wall_sensor_on_adc_complete(void);

/* センサの値（壁に近いほど大きい）。index は config.h の WALL_SENSOR_* */
int32_t wall_sensor_get(int index);

/* 電池電圧 [V] */
float wall_sensor_battery(void);

/* 区画の判定用（しきい値は config.h） */
int wall_exists_left(void);
int wall_exists_right(void);
int wall_exists_front(void);

#endif /* INC_WALL_SENSOR_H_ */
