/*
 * remote_link.c
 *
 * 受け取り方は元のプログラム（main.c の HAL_UART_RxCpltCallback）と同じ。
 * それに加えて、信号が途切れたら止まるようにした（元のプログラムでは、
 * 途切れると最後のボタンを押しっぱなしの状態が続いていた）。
 */

#include "remote_link.h"

#include "usart.h"
#include "config.h"

#define START_BYTE 0xAA
#define END_BYTE   0x55

static uint8_t rx_byte;
static uint8_t rx_state = 0;
static uint8_t rx_data = 0;
static volatile uint8_t buttons = 0;
static volatile uint32_t last_rx_tick = 0;

void remote_link_init(void) {
	HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

uint8_t remote_link_buttons(void) {
#if REMOTE_TIMEOUT_MS > 0
	if (HAL_GetTick() - last_rx_tick > REMOTE_TIMEOUT_MS) {
		return 0;
	}
#endif
	return buttons;
}

void remote_link_on_rx(void) {
	switch (rx_state) {
	case 0:   /* 0xAA を待つ */
		if (rx_byte == START_BYTE) {
			rx_state = 1;
		}
		break;
	case 1:   /* ボタン */
		rx_data = rx_byte;
		rx_state = 2;
		break;
	case 2:   /* 0x55 を待つ。形が正しいときだけ使う */
		if (rx_byte == END_BYTE) {
			buttons = rx_data;
			last_rx_tick = HAL_GetTick();
		}
		rx_state = 0;
		break;
	}
	HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

void remote_link_on_error(void) {
	__HAL_UART_CLEAR_OREFLAG(&huart4);
	__HAL_UART_CLEAR_NEFLAG(&huart4);
	__HAL_UART_CLEAR_FEFLAG(&huart4);
	rx_state = 0;
	HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}
