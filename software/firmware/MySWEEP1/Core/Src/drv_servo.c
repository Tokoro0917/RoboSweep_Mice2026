#include "drv_servo.h"

#include "usart.h"

#define ADDR_GOAL_POSITION 0x1E
#define ADDR_TORQUE_ENABLE 0x24

static void send(uint8_t id, uint8_t address, const uint8_t *data, uint8_t length) {
	uint8_t buf[16];
	uint8_t n = 0;
	buf[n++] = 0xFA;
	buf[n++] = 0xAF;
	buf[n++] = id;
	buf[n++] = 0x00;   /* Flags */
	buf[n++] = address;
	buf[n++] = length;
	buf[n++] = 0x01;   /* Count */
	for (uint8_t i = 0; i < length; i++) {
		buf[n++] = data[i];
	}
	uint8_t sum = 0;
	for (uint8_t i = 2; i < n; i++) {
		sum ^= buf[i];
	}
	buf[n++] = sum;
	HAL_UART_Transmit(&huart1, buf, n, 10);
}

void servo_torque(uint8_t id, int on) {
	uint8_t d = on ? 1 : 0;
	send(id, ADDR_TORQUE_ENABLE, &d, 1);
}

void servo_move(uint8_t id, int16_t angle_x10, uint16_t time_x10ms) {
	uint8_t d[4] = {
		(uint8_t)(angle_x10 & 0xFF), (uint8_t)((angle_x10 >> 8) & 0xFF),
		(uint8_t)(time_x10ms & 0xFF), (uint8_t)((time_x10ms >> 8) & 0xFF),
	};
	send(id, ADDR_GOAL_POSITION, d, 4);
}
