/*
 * app.c
 *
 * 今はリモコン操作モードだけ。自律モードはここに追加していく。
 *
 * リモコン操作（元のプログラムと同じボタン配置）
 *   前 / 後   : 押している間、前進 / 後退（前進は横の壁で補正）
 *   右 / 左   : 押している間、その場で回る
 *   F         : アームを下ろしてつかむ（吸引ファン ON）
 *   R         : ホームポジションに戻す（吸引ファン OFF）
 *   L         : 投げる
 *   B         : 吸引ファン ON / OFF
 */

#include "app.h"

#include <stdio.h>
#include "main.h"
#include "config.h"
#include "motor.h"
#include "motion.h"
#include "wall_sensor.h"
#include "arm.h"
#include "remote_link.h"

static uint8_t prev_buttons = 0;

void app_init(void) {
	motor_init();
	motion_init();
	arm_init();
	remote_link_init();
}

void app_control_tick(void) {
	wall_sensor_start();
	motion_tick();
}

static int is_drive_button(uint8_t buttons) {
	return buttons == REMOTE_BTN_NORTH || buttons == REMOTE_BTN_SOUTH
			|| buttons == REMOTE_BTN_WEST || buttons == REMOTE_BTN_EAST;
}

static void remote_drive(uint8_t buttons) {
	switch (buttons) {
	case REMOTE_BTN_NORTH:
		motion_set_velocity(REMOTE_STRAIGHT_SPEED, REMOTE_STRAIGHT_ACCEL, 0, REMOTE_TURN_ACCEL, 1);
		break;
	case REMOTE_BTN_SOUTH:
		motion_set_velocity(-REMOTE_STRAIGHT_SPEED, REMOTE_STRAIGHT_ACCEL, 0, REMOTE_TURN_ACCEL, 0);
		break;
	case REMOTE_BTN_WEST:
		motion_set_velocity(0, REMOTE_STRAIGHT_ACCEL, REMOTE_TURN_SPEED, REMOTE_TURN_ACCEL, 0);
		break;
	case REMOTE_BTN_EAST:
		motion_set_velocity(0, REMOTE_STRAIGHT_ACCEL, -REMOTE_TURN_SPEED, REMOTE_TURN_ACCEL, 0);
		break;
	default:
		/* 走行のボタンを離したら止まる */
		motion_set_velocity(0, REMOTE_STRAIGHT_ACCEL, 0, REMOTE_TURN_ACCEL, 0);
		break;
	}
}

static void remote_action(uint8_t buttons) {
	/* 押した瞬間だけ実行する */
	if (buttons == prev_buttons) {
		return;
	}
	switch (buttons) {
	case REMOTE_BTN_F:
		arm_catch();
		break;
	case REMOTE_BTN_R:
		arm_home();
		break;
	case REMOTE_BTN_L:
		arm_throw();
		break;
	case REMOTE_BTN_B:
		fan_set(!fan_is_on());
		break;
	default:
		break;
	}
}

void app_loop(void) {
	uint8_t buttons = remote_link_buttons();

	remote_drive(buttons);
	remote_action(buttons);
	prev_buttons = buttons;

	/* 走行のボタンを離して止まったら、励磁を切る（元のプログラムと同じ） */
	if (!is_drive_button(buttons) && motion_is_stopped() && motor_is_enabled()) {
		motor_disable();
	}
}
