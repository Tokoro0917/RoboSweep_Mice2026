#include "drv_fan.h"

#include "tim.h"
#include "config.h"

static int on_state = 0;

void fan_init(void) {
	__HAL_TIM_SET_AUTORELOAD(&htim4, FAN_PWM_PERIOD);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	fan_set(0);
}

void fan_set(int on) {
	on_state = on ? 1 : 0;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, on_state ? FAN_PWM_ON : FAN_PWM_OFF);
}

int fan_is_on(void) {
	return on_state;
}
