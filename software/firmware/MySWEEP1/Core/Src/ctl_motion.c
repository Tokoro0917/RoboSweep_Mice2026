#include "ctl_motion.h"

#include <math.h>
#include "main.h"
#include "config.h"
#include "drv_stepper.h"
#include "ctl_wall.h"

#define DEG_PER_RAD (180.0f / (float)M_PI)

typedef enum {
	MODE_IDLE,
	MODE_STRAIGHT,
	MODE_TURN,
	MODE_DRIVE,
} motion_mode_t;

/* 台形加速の設定 */
typedef struct {
	float target;     /* 動く量（符号なし） */
	float start;      /* 始めたときの位置 */
	float sign;
	float max_speed;
	float end_speed;
	float accel;
} profile_t;

static volatile motion_mode_t mode = MODE_IDLE;
static volatile int wall_on = 0;
static profile_t profile;

static volatile float speed = 0.0f;        /* [mm/s] */
static volatile float turn_speed = 0.0f;   /* [deg/s] */

/* MODE_DRIVE の目標 */
static volatile float drive_speed = 0.0f;
static volatile float drive_accel = 0.0f;
static volatile float drive_turn = 0.0f;
static volatile float drive_turn_accel = 0.0f;

/* 位置 */
static int32_t prev_steps[STEPPER_COUNT];
static volatile float total_distance = 0.0f;
static volatile float total_angle = 0.0f;
static volatile pose_t pose;

/* ------------------------------------------------------------------ */
/* 位置の更新（パルス数から）                                            */
/* ------------------------------------------------------------------ */
static void update_odometry(void) {
	int32_t sl = stepper_get_steps(STEPPER_LEFT);
	int32_t sr = stepper_get_steps(STEPPER_RIGHT);
	float dl = (float)(sl - prev_steps[STEPPER_LEFT]) * stepper_step_mm();
	float dr = (float)(sr - prev_steps[STEPPER_RIGHT]) * stepper_step_mm();
	prev_steps[STEPPER_LEFT] = sl;
	prev_steps[STEPPER_RIGHT] = sr;

	float d = (dl + dr) / 2.0f;
	float dtheta = (dr - dl) / TREAD_MM * DEG_PER_RAD;
	float mid = (pose.theta + dtheta / 2.0f) / DEG_PER_RAD;

	total_distance += d;
	total_angle += dtheta;
	pose.x += d * cosf(mid);
	pose.y += d * sinf(mid);
	pose.theta += dtheta;
}

/* ------------------------------------------------------------------ */
/* 速度の作り方                                                          */
/* ------------------------------------------------------------------ */

/* 台形加速の 1 周期分。向きに関係ない速さ（0 以上）を返す */
static float profile_step(const profile_t *p, float cur, float pos, float start_speed,
		int *done) {
	float moved = (pos - p->start) * p->sign;
	float remaining = p->target - moved;
	if (remaining <= 0.0f) {
		*done = 1;
		return p->end_speed;
	}
	*done = 0;

	if (cur < start_speed) {
		cur = start_speed;
	}
	float braking = (cur * cur - p->end_speed * p->end_speed) / (2.0f * p->accel);
	if (braking >= remaining) {
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
	} else if (cur > p->max_speed) {
		cur -= p->accel * CONTROL_DT;
	}
	return cur;
}

/* 目標の速さへ近づける（MODE_DRIVE 用） */
static float ramp(float cur, float target, float accel, float start_speed) {
	float step = accel * CONTROL_DT;

	/* 逆向きに動いている、または止めたいときは、まず 0 に向けて減速する */
	int reversing = (cur > 0.0f && target < 0.0f) || (cur < 0.0f && target > 0.0f);
	if (target == 0.0f || reversing) {
		if (fabsf(cur) <= start_speed) {
			return 0.0f;
		}
		return (cur > 0.0f) ? cur - step : cur + step;
	}

	if (fabsf(cur) < start_speed) {
		cur = (target > 0.0f) ? start_speed : -start_speed;   /* 動き出し */
	}
	if (cur < target) {
		cur = (cur + step > target) ? target : cur + step;
	} else if (cur > target) {
		cur = (cur - step < target) ? target : cur - step;
	}
	return cur;
}

static void output(float v, float w_deg, int use_wall) {
	float half = w_deg / DEG_PER_RAD * TREAD_MM / 2.0f;
	float right = v + half;
	float left = v - half;
	if (use_wall) {
		float c = wall_control_update(v);
		right += c;
		left -= c;
	}
	stepper_set_speed(STEPPER_LEFT, left);
	stepper_set_speed(STEPPER_RIGHT, right);
}

void motion_tick(void) {
	int done = 0;
	update_odometry();

	switch (mode) {
	case MODE_IDLE:
		speed = 0.0f;
		turn_speed = 0.0f;
		break;

	case MODE_STRAIGHT: {
		float s = profile_step(&profile, fabsf(speed), total_distance, MOTION_START_SPEED, &done);
		speed = s * profile.sign;
		turn_speed = 0.0f;
		if (done) {
			if (profile.end_speed > 0.0f) {
				drive_speed = speed;           /* そのまま進み続ける */
				drive_accel = profile.accel;
				drive_turn = 0.0f;
				drive_turn_accel = profile.accel;
				mode = MODE_DRIVE;
			} else {
				speed = 0.0f;
				mode = MODE_IDLE;
			}
		}
		break;
	}

	case MODE_TURN: {
		float s = profile_step(&profile, fabsf(turn_speed), total_angle, MOTION_TURN_START_SPEED, &done);
		turn_speed = s * profile.sign;
		speed = 0.0f;
		if (done) {
			turn_speed = 0.0f;
			mode = MODE_IDLE;
		}
		break;
	}

	case MODE_DRIVE:
		speed = ramp(speed, drive_speed, drive_accel, MOTION_START_SPEED);
		turn_speed = ramp(turn_speed, drive_turn, drive_turn_accel, MOTION_TURN_START_SPEED);
		break;
	}

	output(speed, turn_speed, wall_on && speed > 0.0f && turn_speed == 0.0f);
}

/* ------------------------------------------------------------------ */
/* 指令                                                                 */
/* ------------------------------------------------------------------ */
void motion_init(void) {
	for (int i = 0; i < STEPPER_COUNT; i++) {
		prev_steps[i] = stepper_get_steps((stepper_id_t)i);
	}
	motion_reset_pose();
	motion_stop();
}

static void start_profile(motion_mode_t m, float amount, float max_speed, float end_speed,
		float accel, float position) {
	__disable_irq();
	profile.sign = (amount >= 0.0f) ? 1.0f : -1.0f;
	profile.target = fabsf(amount);
	profile.start = position;
	profile.max_speed = max_speed;
	profile.end_speed = end_speed;
	profile.accel = accel;
	mode = m;
	__enable_irq();
}

void motion_straight(float distance, float max_speed, float end_speed, float accel,
		int wall_control) {
	stepper_enable();
	if (wall_control && !wall_on) {
		wall_control_reset();
	}
	wall_on = wall_control;
	start_profile(MODE_STRAIGHT, distance, max_speed, end_speed, accel, total_distance);
}

void motion_turn(float angle, float max_speed, float accel) {
	stepper_enable();
	wall_on = 0;
	start_profile(MODE_TURN, angle, max_speed, 0.0f, accel, total_angle);
}

void motion_drive(float new_speed, float accel, float new_turn, float turn_accel,
		int wall_control) {
	if (new_speed != 0.0f || new_turn != 0.0f) {
		stepper_enable();
	}
	if (wall_control && !wall_on) {
		wall_control_reset();
	}
	__disable_irq();
	drive_speed = new_speed;
	drive_accel = accel;
	drive_turn = new_turn;
	drive_turn_accel = turn_accel;
	wall_on = wall_control;
	mode = MODE_DRIVE;
	__enable_irq();
}

void motion_stop(void) {
	__disable_irq();
	mode = MODE_IDLE;
	speed = 0.0f;
	turn_speed = 0.0f;
	drive_speed = 0.0f;
	drive_turn = 0.0f;
	__enable_irq();
	stepper_set_speed(STEPPER_LEFT, 0.0f);
	stepper_set_speed(STEPPER_RIGHT, 0.0f);
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

float motion_speed(void) {
	return speed;
}

float motion_turn_speed(void) {
	return turn_speed;
}

pose_t motion_pose(void) {
	pose_t p;
	__disable_irq();
	p.x = pose.x;
	p.y = pose.y;
	p.theta = pose.theta;
	__enable_irq();
	return p;
}

void motion_reset_pose(void) {
	__disable_irq();
	pose.x = 0.0f;
	pose.y = 0.0f;
	pose.theta = 0.0f;
	__enable_irq();
}

float motion_distance(void) {
	return total_distance;
}

float motion_angle(void) {
	return total_angle;
}
