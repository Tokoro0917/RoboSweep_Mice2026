/*
 * drv_link.h
 *
 * 機体の ESP32（robot_receiver.ino）と UART4 でやりとりする。
 *
 * ESP32 → STM32
 *   リモコンのボタン: 0xAA, ボタン, 0x55
 *   もう 1 台の自律機の位置: 0xA6, 番号, 区画 x, 区画 y, 次の区画 x, 次の区画 y, 状態, チェック
 * STM32 → ESP32（ESP32 が ESP-NOW でもう 1 台へ送る）
 *   自分の位置: 0xA5, 番号, 区画 x, 区画 y, 次の区画 x, 次の区画 y, 状態, チェック
 * チェック = 2〜7 バイト目の XOR
 */
#ifndef INC_DRV_LINK_H_
#define INC_DRV_LINK_H_

#include <stdint.h>

#define LINK_BTN_R      0x01   /* リモコン R    */
#define LINK_BTN_BACK   0x02   /* リモコン Back */
#define LINK_BTN_F      0x04   /* リモコン F    */
#define LINK_BTN_L      0x08   /* リモコン L    */
#define LINK_BTN_LEFT   0x10   /* リモコン Y（左） */
#define LINK_BTN_RIGHT  0x20   /* リモコン A（右） */
#define LINK_BTN_UP     0x40   /* リモコン X（上） */
#define LINK_BTN_DOWN   0x80   /* リモコン B（下） */

void link_init(void);

/* 今のボタン。LINK_TIMEOUT_MS 何も届いていなければ 0 */
uint8_t link_buttons(void);

/* 信号が届いているか */
int link_is_alive(void);

/* 自律機どうしで伝え合う位置 */
typedef struct {
	uint8_t id;       /* AUTO_ROBOT_ID */
	int8_t x, y;      /* 今いる区画 */
	int8_t nx, ny;    /* 次に入る区画（止まっているときは今いる区画） */
	uint8_t status;   /* auto_status_t */
} link_peer_t;

/* もう 1 台の位置。PEER_TIMEOUT_MS 以内に届いていれば 1 を返して out に入れる */
int link_peer(link_peer_t *out);

/* 自分の位置を ESP32 に送る（すぐ戻る。送っている途中なら何もしないで 0） */
int link_send_self(const link_peer_t *self);

/* UART の割り込みから呼ぶ（main.c） */
void link_on_rx(void);
void link_on_error(void);

#endif /* INC_DRV_LINK_H_ */
