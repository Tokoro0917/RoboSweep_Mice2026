/*
 * arm.c
 *
 * 動きは元のプログラム（main.c のボタン F / R / L の処理）と同じ。
 */

#include "arm.h"

#include "main.h"
#include "tim.h"
#include "config.h"
#include "servo.h"

static int fan_on = 0;

static void torque_on(void) {
	servo_torque(SERVO_ID_LOWER, 1);
	servo_torque(SERVO_ID_UPPER, 1);
}

void fan_set(int on) {
	fan_on = on;
	__HAL_TIM_SET_AUTORELOAD(&htim4, FAN_PWM_PERIOD);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, on ? FAN_PWM_ON : FAN_PWM_OFF);
}

int fan_is_on(void) {
	return fan_on;
}

void arm_init(void) {
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	fan_set(0);
}

void arm_catch(void) {
	torque_on();
	servo_move(SERVO_ID_UPPER, ARM_CATCH_UPPER, ARM_CATCH_TIME);
	servo_move(SERVO_ID_LOWER, ARM_CATCH_LOWER, ARM_CATCH_TIME);
	HAL_Delay(ARM_CATCH_WAIT_MS);
	fan_set(1);
}

void arm_home(void) {
	torque_on();
	servo_move(SERVO_ID_LOWER, ARM_HOME_LOWER_FIRST, ARM_HOME_TIME);
	HAL_Delay(ARM_HOME_FIRST_WAIT_MS);
	servo_move(SERVO_ID_UPPER, ARM_HOME_UPPER, ARM_HOME_TIME);
	servo_move(SERVO_ID_LOWER, ARM_HOME_LOWER, ARM_HOME_TIME);
	fan_set(0);
}

void arm_throw(void) {
	torque_on();
	servo_move(SERVO_ID_UPPER, ARM_THROW_READY_UPPER, ARM_THROW_READY_TIME);
	servo_move(SERVO_ID_LOWER, ARM_THROW_READY_LOWER, ARM_THROW_READY_TIME);
	HAL_Delay(ARM_THROW_READY_WAIT_MS);
	servo_move(SERVO_ID_UPPER, ARM_THROW_UPPER, ARM_THROW_TIME);
	HAL_Delay(ARM_THROW_UPPER_WAIT_MS);
	servo_move(SERVO_ID_LOWER, ARM_THROW_LOWER, ARM_THROW_TIME);
}
