/*
 * app.c
 *
 * リモコン操作（受信用 ESP32 の robot_receiver.ino と組み合わせて使う）
 *   X / B   : 押している間、前進 / 後退（前進は横の壁で補正）
 *   Y / A   : 押している間、左 / 右にその場で回る
 *   F       : アームを下ろしてつかむ（ファン ON）
 *   R       : ホームポジション（ファン OFF）
 *   L       : 投げる
 *   Back    : ファン ON / OFF
 * リモコンの信号が LINK_TIMEOUT_MS 途切れたら止まる。
 *
 * LED
 *   LED_1  : 動作中は 1 秒ごとに点滅。電池が少ないと速く点滅
 *   LED_2  : リモコンの信号が届いている間は点灯
 *   LED_T1 : モーターを励磁している間は点灯
 *
 * ログ（USART2、115200bps、LOG_INTERVAL_MS ごと）
 *   ms,電池[mV],前右,右,左,前左,x[mm],y[mm],角度[0.1deg],速さ[mm/s],回転[deg/s],ボタン
 *   （CubeIDE の初期設定では printf で小数を出せないので、すべて整数で出す）
 */

#include "app.h"

#include <stdio.h>
#include "main.h"
#include "config.h"
#include "drv_time.h"
#include "drv_stepper.h"
#include "drv_ir.h"
#include "drv_fan.h"
#include "drv_link.h"
#include "drv_led.h"
#include "ctl_motion.h"
#include "app_arm.h"

static uint8_t prev_buttons = 0;
static uint32_t battery_low_ms = 0;
static int battery_low = 0;

void app_init(void) {
	time_init();
	ir_init();
	stepper_init();
	motion_init();
	arm_init();
	link_init();
#if LOG_ENABLE
	printf("\r\nMySWEEP1 start\r\n");
	printf("ms,batt_mv,ir_fr,ir_r,ir_l,ir_fl,x_mm,y_mm,theta_x10,v,w,btn\r\n");
#endif
}

void app_control_tick(void) {
	ir_tick();
	motion_tick();
}

/* ------------------------------------------------------------------ */
/* リモコン                                                             */
/* ------------------------------------------------------------------ */
static int is_drive(uint8_t b) {
	return b == LINK_BTN_UP || b == LINK_BTN_DOWN || b == LINK_BTN_LEFT || b == LINK_BTN_RIGHT;
}

static void remote_control(uint8_t b) {
	switch (b) {
	case LINK_BTN_UP:
		motion_drive(REMOTE_SPEED, REMOTE_ACCEL, 0, REMOTE_TURN_ACCEL, REMOTE_WALL_CONTROL);
		break;
	case LINK_BTN_DOWN:
		motion_drive(-REMOTE_SPEED, REMOTE_ACCEL, 0, REMOTE_TURN_ACCEL, 0);
		break;
	case LINK_BTN_LEFT:
		motion_drive(0, REMOTE_ACCEL, REMOTE_TURN_SPEED, REMOTE_TURN_ACCEL, 0);
		break;
	case LINK_BTN_RIGHT:
		motion_drive(0, REMOTE_ACCEL, -REMOTE_TURN_SPEED, REMOTE_TURN_ACCEL, 0);
		break;
	default:
		motion_drive(0, REMOTE_ACCEL, 0, REMOTE_TURN_ACCEL, 0);
		break;
	}

	/* アームとファンは押した瞬間だけ */
	if (b != prev_buttons) {
		switch (b) {
		case LINK_BTN_F:    arm_catch(); break;
		case LINK_BTN_R:    arm_home(); break;
		case LINK_BTN_L:    arm_throw(); break;
		case LINK_BTN_BACK: fan_set(!fan_is_on()); break;
		default: break;
		}
	}
	prev_buttons = b;

	/* 走行のボタンを離して止まったら、励磁を切る */
	if (!is_drive(b) && motion_is_stopped() && stepper_is_enabled()) {
		stepper_disable();
	}
}

/* ------------------------------------------------------------------ */
/* 状態の表示                                                           */
/* ------------------------------------------------------------------ */
static void update_battery(uint32_t now) {
	static uint32_t last = 0;
	uint32_t dt = now - last;
	last = now;
	if (battery_voltage() > 1.0f && battery_voltage() < BATTERY_LOW_V) {
		battery_low_ms += dt;
	} else {
		battery_low_ms = 0;
	}
	battery_low = battery_low_ms >= 1000;
}

static void update_leds(uint32_t now) {
	uint32_t period = battery_low ? 200 : 1000;
	led_set(LED_1, (now % period) < period / 2);
	led_set(LED_2, link_is_alive());
	led_set(LED_T1, stepper_is_enabled());
}

static void log_status(uint32_t now) {
#if LOG_ENABLE
	static uint32_t last = 0;
	if (now - last < LOG_INTERVAL_MS) {
		return;
	}
	last = now;
	pose_t p = motion_pose();
	printf("%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%02X\r\n",
			(unsigned long)now, (long)(battery_voltage() * 1000.0f),
			(long)ir_get(IR_FRONT_RIGHT), (long)ir_get(IR_RIGHT),
			(long)ir_get(IR_LEFT), (long)ir_get(IR_FRONT_LEFT),
			(long)p.x, (long)p.y, (long)(p.theta * 10.0f),
			(long)motion_speed(), (long)motion_turn_speed(),
			link_buttons());
#else
	(void)now;
#endif
}

void app_loop(void) {
	uint32_t now = time_ms();

	remote_control(link_buttons());
	arm_update();

	update_battery(now);
	update_leds(now);
	log_status(now);
}
