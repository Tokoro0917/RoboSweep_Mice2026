#include "drv_led.h"

#include "main.h"

void led_set(led_t led, int on) {
	GPIO_PinState s = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
	switch (led) {
	case LED_1:  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, s); break;
	case LED_2:  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, s); break;
	case LED_T1: HAL_GPIO_WritePin(LED_T1_GPIO_Port, LED_T1_Pin, s); break;
	}
}
