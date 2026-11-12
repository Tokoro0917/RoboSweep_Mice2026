/*
 * drv_servo.h
 *
 * 双葉 RS204MD を USART1 で動かす（ショートパケット）。
 *   0xFA 0xAF ID Flags Address Length Count Data... Sum（Sum は ID〜Data の XOR）
 * 送るのに 1ms ほどかかるので、割り込みの中からは呼ばない。
 */
#ifndef INC_DRV_SERVO_H_
#define INC_DRV_SERVO_H_

#include <stdint.h>

void servo_torque(uint8_t id, int on);

/* angle_x10: 角度 x10（500 = 50.0deg）、time_x10ms: 動き終わるまでの時間 x10ms */
void servo_move(uint8_t id, int16_t angle_x10, uint16_t time_x10ms);

#endif /* INC_DRV_SERVO_H_ */
