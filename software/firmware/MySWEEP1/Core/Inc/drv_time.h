/*
 * drv_time.h
 *
 * 時間まわり。ms は HAL のティック、us は CPU のサイクルカウンタ（DWT）で測る。
 */
#ifndef INC_DRV_TIME_H_
#define INC_DRV_TIME_H_

#include <stdint.h>

void time_init(void);
uint32_t time_ms(void);
/* 短い待ち（割り込みの中で使ってよいのは数十 us まで） */
void time_delay_us(uint32_t us);

#endif /* INC_DRV_TIME_H_ */
