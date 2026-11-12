/*
 * drv_stepper.h
 *
 * ステッピングモーター（左右）のドライバ。
 * 車輪の速さ [mm/s] を指定するとパルスを出し、出したパルスの数を数える。
 *
 * 仕組み
 *   - パルスの周期はタイマーの ARR、デューティは 50%（CCR = ARR/2）。
 *   - ARR と CCR は「予約（プリロード）」で書くので、今のパルスが終わってから切り替わる。
 *   - パルスの周期が終わるたびに入る割り込みでは、パルスを数えるだけ。
 *   - 回転方向を変えるときは、パルスを 1 回休んでから方向ピンを切り替える
 *     （パルスの途中で方向が変わらないようにするため）。
 */
#ifndef INC_DRV_STEPPER_H_
#define INC_DRV_STEPPER_H_

#include <stdint.h>

typedef enum {
	STEPPER_LEFT = 0,
	STEPPER_RIGHT = 1,
	STEPPER_COUNT = 2,
} stepper_id_t;

void stepper_init(void);

/* モータードライバを有効にして励磁する（3ms かかる。割り込みの中では呼ばない） */
void stepper_enable(void);
/* 励磁を切る（手で回せる） */
void stepper_disable(void);
int stepper_is_enabled(void);

/* 速さを設定する。負で後ろ向き。|速さ| < WHEEL_MIN_SPEED なら止まる */
void stepper_set_speed(stepper_id_t id, float mm_per_s);

/* 起動してから出したパルスの合計（前向き +） */
int32_t stepper_get_steps(stepper_id_t id);

/* 1 パルスで進む距離 [mm] */
float stepper_step_mm(void);

/* タイマー（TIM3 / TIM8）の更新割り込みから呼ぶ（stm32f4xx_it.c） */
void stepper_on_timer_update(const void *timer);   /* TIM3 か TIM8 を渡す */

#endif /* INC_DRV_STEPPER_H_ */
