/*
 * app.h
 *
 * いちばん上の層。起動したあとの流れと、1ms ごとの制御をまとめる。
 */

#ifndef INC_APP_H_
#define INC_APP_H_

/* 周辺機能の初期化が終わったあとに 1 回呼ぶ */
void app_init(void);

/* main の while(1) から呼び続ける */
void app_loop(void);

/* 1ms ごとの制御割り込み（TIM6）から呼ぶ */
void app_control_tick(void);

#endif /* INC_APP_H_ */
