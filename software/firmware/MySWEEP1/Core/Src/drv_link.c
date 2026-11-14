#include "drv_link.h"

#include "usart.h"
#include "config.h"

static uint8_t rx_byte;
static uint8_t state = 0;
static uint8_t data = 0;
static volatile uint8_t buttons = 0;
static volatile uint32_t last_ms = 0;
static volatile int received_once = 0;

/* もう 1 台の位置 */
#define PEER_LEN 7   /* 番号〜状態（6 バイト）+ チェック */
static uint8_t peer_buf[PEER_LEN];
static uint8_t peer_pos = 0;
static volatile link_peer_t peer;
static volatile uint32_t peer_ms = 0;
static volatile int peer_once = 0;
static uint8_t tx_buf[8];

void link_init(void) {
	HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

int link_is_alive(void) {
	if (!received_once) {
		return 0;
	}
#if LINK_TIMEOUT_MS > 0
	return (HAL_GetTick() - last_ms) <= LINK_TIMEOUT_MS;
#else
	return 1;
#endif
}

uint8_t link_buttons(void) {
	return link_is_alive() ? buttons : 0;
}

int link_peer(link_peer_t *out) {
	if (!peer_once || (HAL_GetTick() - peer_ms) > PEER_TIMEOUT_MS) {
		return 0;
	}
	__disable_irq();
	out->id = peer.id;
	out->x = peer.x;
	out->y = peer.y;
	out->nx = peer.nx;
	out->ny = peer.ny;
	out->status = peer.status;
	__enable_irq();
	return 1;
}

int link_send_self(const link_peer_t *self) {
	if (huart4.gState != HAL_UART_STATE_READY) {
		return 0;
	}
	tx_buf[0] = 0xA5;
	tx_buf[1] = self->id;
	tx_buf[2] = (uint8_t)self->x;
	tx_buf[3] = (uint8_t)self->y;
	tx_buf[4] = (uint8_t)self->nx;
	tx_buf[5] = (uint8_t)self->ny;
	tx_buf[6] = self->status;
	tx_buf[7] = tx_buf[1] ^ tx_buf[2] ^ tx_buf[3] ^ tx_buf[4] ^ tx_buf[5] ^ tx_buf[6];
	return HAL_UART_Transmit_IT(&huart4, tx_buf, sizeof(tx_buf)) == HAL_OK;
}

static void peer_done(void) {
	uint8_t sum = 0;
	for (int i = 0; i < PEER_LEN - 1; i++) {
		sum ^= peer_buf[i];
	}
	if (sum != peer_buf[PEER_LEN - 1]) {
		return;   /* 壊れている */
	}
	peer.id = peer_buf[0];
	peer.x = (int8_t)peer_buf[1];
	peer.y = (int8_t)peer_buf[2];
	peer.nx = (int8_t)peer_buf[3];
	peer.ny = (int8_t)peer_buf[4];
	peer.status = peer_buf[5];
	peer_ms = HAL_GetTick();
	peer_once = 1;
}

/* state: 0 = 先頭待ち、1〜2 = ボタン、10 = もう 1 台の位置 */
void link_on_rx(void) {
	switch (state) {
	case 0:
		if (rx_byte == 0xAA) {
			state = 1;
		} else if (rx_byte == 0xA6) {
			peer_pos = 0;
			state = 10;
		}
		break;
	case 1:
		data = rx_byte;
		state = 2;
		break;
	case 2:
		if (rx_byte == 0x55) {
			buttons = data;
			last_ms = HAL_GetTick();
			received_once = 1;
		}
		state = 0;
		break;
	case 10:
		peer_buf[peer_pos++] = rx_byte;
		if (peer_pos >= PEER_LEN) {
			peer_done();
			state = 0;
		}
		break;
	default:
		state = 0;
		break;
	}
	HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

void link_on_error(void) {
	__HAL_UART_CLEAR_OREFLAG(&huart4);
	__HAL_UART_CLEAR_NEFLAG(&huart4);
	__HAL_UART_CLEAR_FEFLAG(&huart4);
	state = 0;
	HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}
