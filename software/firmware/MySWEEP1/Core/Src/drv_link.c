#include "drv_link.h"

#include "usart.h"
#include "config.h"

static uint8_t rx_byte;
static uint8_t state = 0;
static uint8_t data = 0;
static volatile uint8_t buttons = 0;
static volatile uint32_t last_ms = 0;
static volatile int received_once = 0;

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

void link_on_rx(void) {
	switch (state) {
	case 0:
		if (rx_byte == 0xAA) {
			state = 1;
		}
		break;
	case 1:
		data = rx_byte;
		state = 2;
		break;
	default:
		if (rx_byte == 0x55) {
			buttons = data;
			last_ms = HAL_GetTick();
			received_once = 1;
		}
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
