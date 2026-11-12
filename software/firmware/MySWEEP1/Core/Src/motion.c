/*
 * motion.c
 *
 * 台形加速（加速 → 一定 → 減速）で速度を作る。
 * 進んだ距離・回った角度は、モーターに出したパルスの数から求めるので、
 * 時間で積分するよりずれにくい。
 */

#include "motion.h"

#include <math.h>
#include "main.h"
#include "config.h"
#include "motor.h"
#include "wall_sensor.h"

typedef enum {
	MODE_IDLE,
	MODE_STRAIGHT,
	MODE_TURN,
	MODE_VELOCITY,
} motion_mode_t;

typedef struct {
	float target;      /* 進む距離 / 回る角度（符号なし） */
	float start_pos;   /* 始めたときの位置 */
	float sign;        /* +1 / -1 */
	float max_speed;
	float end_speed;
	float accel;
} profile_t;

static volatile motion_mode_t mode = MODE_IDLE;
static volatile int wall_control_on = 0;

/* 今の速さ（前後 [mm/s] と回転 [deg/s]）。符号付き */
static volatile float speed = 0.0f;
static volatile float turn_speed = 0.0f;

static profile_t profile;

/* MODE_VELOCITY の目標 */
static volatile float target_speed = 0.0f;
static volatile float target_speed_accel = 0.0f;
static volatile float target_turn = 0.0f;
static volatile float target_turn_accel = 0.0f;

/* 壁制御で 1 周期前の値を覚えておく */
static int32_t prev_left = 0;
static int32_t prev_right = 0;

static const float RAD_TO_DEG = 180.0f / (float)M_PI;

/* ------------------------------------------------------------------ */
/* 位置                                                                 */
/* ------------------------------------------------------------------ */

static float wheel_mm(wheel_t w) {
	return (float)motor_get_steps(w) * motor_step_mm();
}

float motion_get_distance(void) {
	return (wheel_mm(WHEEL_LEFT) + wheel_mm(WHEEL_RIGHT)) / 2.0f;
}

float motion_get_angle(void) {
	/* 左回り（+）のとき左車輪が前に回る（下の set_wheels を参照） */
	return (wheel_mm(WHEEL_LEFT) - wheel_mm(WHEEL_RIGHT)) / TREAD_MM * RAD_TO_DEG;
}

/* ------------------------------------------------------------------ */
/* 壁制御（元のプログラムの wallcontrol と同じ計算）                       */
/* ------------------------------------------------------------------ */

static float wall_correction(float v) {
	int32_t l = wall_sensor_get(WALL_SENSOR_LEFT);
	int32_t r = wall_sensor_get(WALL_SENSOR_RIGHT);
	int32_t fa = wall_sensor_get(WALL_SENSOR_FRONT_A);
	int32_t fb = wall_sensor_get(WALL_SENSOR_FRONT_B);

	/* 壁がない・急に値が跳ねたときは、その側を使わない */
	int use_l = !(l < WALL_EXIST_LEFT || (l - prev_left) > WALL_JUMP_LEFT);
	int use_r = !(r < WALL_EXIST_RIGHT || (r - prev_right) > WALL_JUMP_RIGHT);
	prev_left = l;
	prev_right = r;

	float e;
	if (use_l && use_r) {
		e = (float)(l - WALL_CENTER_LEFT) - (float)(r - WALL_CENTER_RIGHT);
	} else if (use_l) {
		e = 2.0f * (float)(l - WALL_CENTER_LEFT);
	} else if (use_r) {
		e = -2.0f * (float)(r - WALL_CENTER_RIGHT);
	} else {
		return 0.0f;
	}

	/* 前の壁に近く、横にも近いときは補正しない */
	if ((fa > WALL_FRONT_NEAR || fb > WALL_FRONT_NEAR)
			&& (r > WALL_CENTER_RIGHT || l > WALL_CENTER_LEFT)) {
		return 0.0f;
	}

	return WALL_CONTROL_GAIN * e * (v / WALL_CONTROL_REF_SPEED);
}

/* ------------------------------------------------------------------ */
/* 速度の更新                                                           */
/* ------------------------------------------------------------------ */

/* 台形加速の 1 周期分。cur は進む向きの速さ（0 以上）を返す */
static float profile_step(profile_t *p, float cur, float pos, float start_speed, int *done) {
	float traveled = (pos - p->start_pos) * p->sign;
	float remaining = p->target - traveled;

	if (remaining <= 0.0f) {
		*done = 1;
		return p->end_speed;
	}
	*done = 0;

	if (cur < start_speed) {
		cur = start_speed;
	}

	float brake = (cur * cur - p->end_speed * p->end_speed) / (2.0f * p->accel);
	if (brake >= remaining) {
		cur -= p->accel * CONTROL_DT;
		float floor = (p->end_speed > start_speed) ? p->end_speed : start_speed;
		if (cur < floor) {
			cur = floor;
		}
	} else if (cur < p->max_speed) {
		cur += p->accel * CONTROL_DT;
		if (cur > p->max_speed) {
			cur = p->max_speed;
		}
	}
	return cur;
}

/* 目標の速さに向けて加速・減速する（MODE_VELOCITY 用） */
static float ramp(float cur, float target, float accel, float start_speed) {
	if (target == 0.0f && fabsf(cur) <= start_speed) {
		return 0.0f;   /* ゆっくりになったら止める */
	}
	if (target != 0.0f && fabsf(cur) < start_speed) {
		cur = (target > 0) ? start_speed : -start_speed;   /* 動き出し */
	}
	float step = accel * CONTROL_DT;
	if (cur < target) {
		cur = (cur + step > target) ? target : cur + step;
	} else if (cur > target) {
		cur = (cur - step < target) ? target : cur - step;
	}
	return cur;
}

static void set_wheels(float v, float w_deg, int wall_control) {
	/*
	 * 元のプログラムに合わせて、左回り（w > 0）で「左」車輪が前に回るようにしている。
	 * 普通の向きとは逆だが、config.h の「左右について」のとおり、元のプログラムの
	 * 左右の名前が実物と逆になっていると考えると、実機では正しく左に回る。
	 */
	float diff = w_deg / RAD_TO_DEG * TREAD_MM / 2.0f;
	float left = v + diff;
	float right = v - diff;

	if (wall_control && v > 0.0f) {
		float c = wall_correction(v);
		left += c;
		right -= c;
	}
	motor_set_speed(left, right);
}

void motion_tick(void) {
	int done = 0;

	switch (mode) {
	case MODE_IDLE:
		speed = 0.0f;
		turn_speed = 0.0f;
		break;

	case MODE_STRAIGHT: {
		float cur = profile_step(&profile, fabsf(speed), motion_get_distance(),
				MOTION_START_SPEED, &done);
		speed = cur * profile.sign;
		turn_speed = 0.0f;
		if (done) {
			if (profile.end_speed > 0.0f) {
				/* 止まらずにその速さで進み続ける */
				target_speed = speed;
				target_speed_accel = profile.accel;
				target_turn = 0.0f;
				mode = MODE_VELOCITY;
			} else {
				speed = 0.0f;
				mode = MODE_IDLE;
			}
		}
		break;
	}

	case MODE_TURN: {
		float cur = profile_step(&profile, fabsf(turn_speed), motion_get_angle(),
				MOTION_TURN_START_SPEED, &done);
		turn_speed = cur * profile.sign;
		speed = 0.0f;
		if (done) {
			turn_speed = 0.0f;
			mode = MODE_IDLE;
		}
		break;
	}

	case MODE_VELOCITY:
		speed = ramp(speed, target_speed, target_speed_accel, MOTION_START_SPEED);
		turn_speed = ramp(turn_speed, target_turn, target_turn_accel, MOTION_TURN_START_SPEED);
		break;
	}

	set_wheels(speed, turn_speed, wall_control_on && mode != MODE_TURN);
}

/* ------------------------------------------------------------------ */
/* 指令                                                                 */
/* ------------------------------------------------------------------ */

void motion_init(void) {
	motion_stop();
}

static void start_profile(motion_mode_t m, float amount, float max_speed, float end_speed,
		float accel, float pos) {
	__disable_irq();
	profile.sign = (amount >= 0) ? 1.0f : -1.0f;
	profile.target = fabsf(amount);
	profile.start_pos = pos;
	profile.max_speed = max_speed;
	profile.end_speed = end_speed;
	profile.accel = accel;
	mode = m;
	__enable_irq();
}

void motion_straight(float distance_mm, float max_speed, float end_speed,
		float accel, int wall_control) {
	motor_enable();
	wall_control_on = wall_control;
	start_profile(MODE_STRAIGHT, distance_mm, max_speed, end_speed, accel,
			motion_get_distance());
}

void motion_turn(float angle_deg, float max_speed_deg, float accel_deg) {
	motor_enable();
	wall_control_on = 0;
	start_profile(MODE_TURN, angle_deg, max_speed_deg, 0.0f, accel_deg,
			motion_get_angle());
}

void motion_set_velocity(float new_speed, float speed_accel,
		float new_turn, float turn_accel, int wall_control) {
	if (new_speed != 0.0f || new_turn != 0.0f) {
		motor_enable();
	}
	__disable_irq();
	target_speed = new_speed;
	target_speed_accel = speed_accel;
	target_turn = new_turn;
	target_turn_accel = turn_accel;
	wall_control_on = wall_control;
	mode = MODE_VELOCITY;
	__enable_irq();
}

void motion_stop(void) {
	__disable_irq();
	mode = MODE_IDLE;
	speed = 0.0f;
	turn_speed = 0.0f;
	target_speed = 0.0f;
	target_turn = 0.0f;
	__enable_irq();
	motor_set_speed(0.0f, 0.0f);
}

int motion_is_busy(void) {
	return mode == MODE_STRAIGHT || mode == MODE_TURN;
}

void motion_wait(void) {
	while (motion_is_busy()) {
	}
}

int motion_is_stopped(void) {
	return speed == 0.0f && turn_speed == 0.0f;
}
