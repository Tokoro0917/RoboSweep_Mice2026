/*
 * drv_link.h
 *
 * 受信用 ESP32（robot_receiver.ino）から UART4 で届くボタンの状態。
 * 1 回分: 0xAA, ボタン, 0x55
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

/* UART の割り込みから呼ぶ（main.c） */
void link_on_rx(void);
void link_on_error(void);

#endif /* INC_DRV_LINK_H_ */
