/*
 * remote_link.h
 *
 * 機体に載せた ESP32 から UART4 で届くリモコンのボタン状態を受け取る。
 *
 * 1 回分のデータ: 0xAA, ボタン（1 バイト）, 0x55
 * ボタンのビット: REMOTE_BTN_*（1 回に押されているのは 1 つだけの前提）
 */

#ifndef INC_REMOTE_LINK_H_
#define INC_REMOTE_LINK_H_

#include <stdint.h>

#define REMOTE_BTN_R      0x01   /* アーム: ホームポジション */
#define REMOTE_BTN_B      0x02   /* 吸引ファン ON/OFF */
#define REMOTE_BTN_F      0x04   /* アーム: つかむ */
#define REMOTE_BTN_L      0x08   /* アーム: 投げる */
#define REMOTE_BTN_WEST   0x10   /* 左に回る */
#define REMOTE_BTN_EAST   0x20   /* 右に回る */
#define REMOTE_BTN_NORTH  0x40   /* 前に進む */
#define REMOTE_BTN_SOUTH  0x80   /* 後ろに進む */

/* 受信を始める（main で 1 回だけ呼ぶ） */
void remote_link_init(void);

/* 今のボタン状態。信号が REMOTE_TIMEOUT_MS 届いていなければ 0 */
uint8_t remote_link_buttons(void);

/* UART の割り込みから呼ぶ（main.c の HAL_UART_RxCpltCallback など） */
void remote_link_on_rx(void);
void remote_link_on_error(void);

#endif /* INC_REMOTE_LINK_H_ */
