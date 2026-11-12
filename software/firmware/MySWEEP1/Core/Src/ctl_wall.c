#include "ctl_wall.h"

#include "config.h"
#include "drv_ir.h"

static int32_t prev_left = 0;
static int32_t prev_right = 0;
static float prev_error = 0.0f;
static uint32_t last_update = 0;
static float last_output = 0.0f;

void wall_control_reset(void) {
	prev_left = ir_get(IR_LEFT);
	prev_right = ir_get(IR_RIGHT);
	prev_error = 0.0f;
	last_output = 0.0f;
	last_update = ir_update_count();
}

int wall_left(void) {
	return ir_get(IR_LEFT) >= IR_WALL_SIDE;
}

int wall_right(void) {
	return ir_get(IR_RIGHT) >= IR_WALL_SIDE;
}

int wall_front(void) {
	return ir_get(IR_FRONT_LEFT) >= IR_WALL_FRONT
			&& ir_get(IR_FRONT_RIGHT) >= IR_WALL_FRONT;
}

float wall_control_update(float speed) {
	if (speed <= 0.0f) {
		return 0.0f;
	}
	/* センサの値は 2ms ごとに変わる。変わっていなければ前の出力のまま */
	uint32_t n = ir_update_count();
	if (n == last_update) {
		return last_output;
	}
	last_update = n;

	int32_t l = ir_get(IR_LEFT);
	int32_t r = ir_get(IR_RIGHT);
	/* 壁がない・値が急に跳ねた側は使わない */
	int use_l = (l >= IR_WALL_SIDE) && (l - prev_left <= IR_JUMP_LIMIT);
	int use_r = (r >= IR_WALL_SIDE) && (r - prev_right <= IR_JUMP_LIMIT);
	prev_left = l;
	prev_right = r;

	int front_near = ir_get(IR_FRONT_LEFT) > IR_FRONT_TOO_NEAR
			|| ir_get(IR_FRONT_RIGHT) > IR_FRONT_TOO_NEAR;

	/* error > 0: 右の壁に近い（左へ寄せたい） */
	float error;
	if (front_near) {
		error = 0.0f;
	} else if (use_l && use_r) {
		error = (float)(r - IR_CENTER_RIGHT) - (float)(l - IR_CENTER_LEFT);
	} else if (use_r) {
		error = 2.0f * (float)(r - IR_CENTER_RIGHT);
	} else if (use_l) {
		error = -2.0f * (float)(l - IR_CENTER_LEFT);
	} else {
		error = 0.0f;
	}

	float out = WALL_KP * error + WALL_KD * (error - prev_error);
	prev_error = error;

	out *= speed / WALL_REF_SPEED;
	if (out > WALL_MAX_CORRECTION) {
		out = WALL_MAX_CORRECTION;
	} else if (out < -WALL_MAX_CORRECTION) {
		out = -WALL_MAX_CORRECTION;
	}
	last_output = out;
	return out;
}
