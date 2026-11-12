/*
 * drv_ir.h
 *
 * 赤外線の壁センサ 4 個と電池電圧。
 *
 * 1ms ごとに ir_tick() を呼ぶと、次の 2 つを交互に測る。
 *   A: SENSORLED_1 を点ける → SENSOR_1,2 は「点灯時」、SENSOR_3,4 は「消灯時」
 *   B: SENSORLED_2 を点ける → SENSOR_1,2 は「消灯時」、SENSOR_3,4 は「点灯時」
 * B が終わるたびに「点灯時 - 消灯時」（外の光を引いた値）を更新する（2ms ごと）。
 * LED は ADC の変換が終わったらすぐ消すので、点いている時間は短い。
 */
#ifndef INC_DRV_IR_H_
#define INC_DRV_IR_H_

#include <stdint.h>

#define IR_COUNT 4

void ir_init(void);

/* 1ms の制御割り込みから呼ぶ */
void ir_tick(void);

/* ADC の DMA 完了割り込みから呼ぶ（adc.c） */
void ir_on_adc_complete(void);

/* センサの値（壁に近いほど大きい）。index は config.h の IR_* */
int32_t ir_get(int index);

/* 値を更新した回数（新しい値が来たかを知るのに使う） */
uint32_t ir_update_count(void);

/* 電池電圧 [V] */
float battery_voltage(void);

#endif /* INC_DRV_IR_H_ */
