/*
 * drv_link.c（ESP32 とのやりとり）を PC 上で動かすテスト。
 * ボタン（0xAA）、もう 1 台の位置（0xA6）、カメラのボール（0xB0）が混ざって届いても、
 * それぞれ正しく読めるかを確かめる。
 */
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "drv_link.h"
#include "config.h"

UART_HandleTypeDef huart4 = { HAL_UART_STATE_READY };
uint8_t *fake_rx_ptr;
uint32_t fake_tick = 1000;
uint8_t fake_tx[16];
int fake_tx_len;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *buf, uint16_t n) { (void)h; (void)n; fake_rx_ptr = buf; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, const uint8_t *buf, uint16_t n) { (void)h; memcpy(fake_tx, buf, n); fake_tx_len = n; return HAL_OK; }
uint32_t HAL_GetTick(void) { return fake_tick; }

static int failures = 0;
static void expect(const char *name, int cond) {
	printf("%-48s %s\n", name, cond ? "OK" : "NG");
	if (!cond) failures++;
}
static void feed(const uint8_t *b, int n) {
	for (int i = 0; i < n; i++) { *fake_rx_ptr = b[i]; link_on_rx(); }
}
static uint8_t xor_of(const uint8_t *b, int n) { uint8_t s = 0; for (int i = 0; i < n; i++) s ^= b[i]; return s; }

int main(void) {
	link_init();
	link_peer_t p;
	link_ball_t b;

	expect("最初は何も届いていない", !link_is_alive() && !link_peer(&p) && !link_ball(&b));

	uint8_t btn[] = { 0xAA, 0x40, 0x55 };
	feed(btn, 3);
	expect("ボタン: 前進が読める", link_buttons() == 0x40);

	uint8_t pos[8] = { 0xA6, 1, 3, 4, 3, 5, 1, 0 };
	pos[7] = xor_of(pos + 1, 6);
	uint8_t bal[7] = { 0xB0, 1, (uint8_t)(int8_t)-12, 37, 25, 9, 0 };
	bal[6] = xor_of(bal + 1, 5);

	/* 混ぜて送る: ボタン → ボール → 位置 → ボタン */
	uint8_t mix[3 + 7 + 8 + 3];
	memcpy(mix, btn, 3); memcpy(mix + 3, bal, 7); memcpy(mix + 10, pos, 8);
	uint8_t btn2[] = { 0xAA, 0x10, 0x55 };
	memcpy(mix + 18, btn2, 3);
	feed(mix, sizeof(mix));
	expect("混ぜて送っても、ボタンが読める", link_buttons() == 0x10);
	expect("混ぜて送っても、位置が読める", link_peer(&p) && p.id == 1 && p.x == 3 && p.y == 4 && p.nx == 3 && p.ny == 5);
	expect("混ぜて送っても、ボールが読める", link_ball(&b) && b.found == 1 && b.deg == -12 && b.cm == 37 && b.size == 25 && b.seq == 9);

	/* チェックが合わないボールは使わない */
	uint8_t bad[7];
	memcpy(bad, bal, 7);
	bad[3] = 99;   /* 距離を壊す */
	feed(bad, 7);
	expect("壊れたボールのデータは使わない", link_ball(&b) && b.cm == 37);

	/* 古くなったら使わない */
	fake_tick += BALL_TIMEOUT_MS + 1;
	expect("BALL_TIMEOUT_MS より古いボールは使わない", !link_ball(&b));

	/* 自分の位置を送る */
	link_peer_t me = { 0, 1, 2, 1, 3, 1 };
	link_send_self(&me);
	expect("自分の位置: 先頭 0xA5 で 8 バイト", fake_tx_len == 8 && fake_tx[0] == 0xA5);
	expect("自分の位置: チェックが合う", fake_tx[7] == xor_of(fake_tx + 1, 6));

	printf(failures ? "\n%d 件失敗\n" : "\nすべて OK\n", failures);
	return failures ? 1 : 0;
}
