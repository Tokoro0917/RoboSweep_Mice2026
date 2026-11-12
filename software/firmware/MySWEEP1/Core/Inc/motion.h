/*
 * motion.h
 *
 * 機体の動き（まっすぐ進む・その場で回る・一定の速さで動き続ける）を作る層。
 * 1ms ごとの制御割り込みで速度を少しずつ変え、motor に渡す。
 *
 * 向き: 距離は前が +、角度は左回りが +。
 */

#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#include <stdint.h>

void motion_init(void);

/* 1ms ごとに制御割り込みから呼ぶ */
void motion_tick(void);

/*
 * まっすぐ distance_mm 進む（負なら後ろへ）。
 *   max_speed : いちばん速いときの速さ [mm/s]
 *   end_speed : 進み終わったときの速さ [mm/s]。0 なら止まる。0 でなければその速さで進み続ける
 *   accel     : 加速度・減速度 [mm/s^2]
 *   wall_control : 1 なら横の壁を見てまっすぐ進むように補正する
 * すぐに戻る。終わるのを待つときは motion_wait() を呼ぶ。
 */
void motion_straight(float distance_mm, float max_speed, float end_speed,
		float accel, int wall_control);

/* その場で angle_deg 回る（左回りが +） */
void motion_turn(float angle_deg, float max_speed_deg, float accel_deg);

/*
 * 指定した速さに向けて加速・減速し、そのまま動き続ける（リモコン操作用）。
 *   speed     : 前後の速さ [mm/s]
 *   turn_speed: 回る速さ [deg/s]（左回りが +）
 */
void motion_set_velocity(float speed, float speed_accel,
		float turn_speed, float turn_accel, int wall_control);

/* すぐに止まる */
void motion_stop(void);

/* motion_straight / motion_turn の途中なら 1 */
int motion_is_busy(void);

/* motion_straight / motion_turn が終わるまで待つ */
void motion_wait(void);

/* 止まっていれば 1（速度が 0） */
int motion_is_stopped(void);

/* 起動してから進んだ距離 [mm] と回った角度 [deg]（パルス数から計算） */
float motion_get_distance(void);
float motion_get_angle(void);

#endif /* INC_MOTION_H_ */
