/*
 * motor.h
 *
 * ステッピングモーター（左右 2 個）を動かすいちばん下の層。
 * 車輪の速度 [mm/s] を指定すると、それに合わせたパルスを出す。
 * 出したパルスの数を数えているので、進んだ距離もここで分かる。
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>

typedef enum {
	WHEEL_LEFT = 0,
	WHEEL_RIGHT = 1,
} wheel_t;

/* タイマーと割り込みを開始する（main で 1 回だけ呼ぶ） */
void motor_init(void);

/* モーターを励磁する（トルクがかかる）。割り込みの中では呼ばない。 */
void motor_enable(void);

/* モーターの励磁を切る（手で回せるようになる） */
void motor_disable(void);

int motor_is_enabled(void);

/* 車輪の速度を設定する。負の値で後ろ向き。 */
void motor_set_speed(float left_mm_s, float right_mm_s);

/* 起動してから出したパルスの合計（前向き +、後ろ向き -） */
int32_t motor_get_steps(wheel_t wheel);

/* 1 パルスで進む距離 [mm] */
float motor_step_mm(void);

/* タイマーの更新割り込みから呼ぶ（stm32f4xx_it.c） */
void motor_on_timer_update(wheel_t wheel);

#endif /* INC_MOTOR_H_ */
