/*
 * servo.h
 *
 * アーム用のサーボ（双葉 RS204MD）を USART1 で動かす。
 * 通信の中身は元のプログラム（Pl_servo.c）と同じ。
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>

/* トルクを ON / OFF する */
void servo_torque(uint8_t id, int on);

/*
 * 角度を指定して動かす。
 *   angle_x10 : 角度 × 10（例: 500 = 50.0 度）
 *   time_x10ms: 動き終わるまでの時間 × 10ms（例: 50 = 0.5 秒）
 */
void servo_move(uint8_t id, int16_t angle_x10, uint16_t time_x10ms);

#endif /* INC_SERVO_H_ */
