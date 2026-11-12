#include "app_arm.h"

#include <stddef.h>
#include "config.h"
#include "drv_time.h"
#include "drv_servo.h"
#include "drv_fan.h"

typedef enum {
	ACT_TORQUE_ON,   /* 両方のサーボのトルクを ON */
	ACT_MOVE,        /* サーボを動かす */
	ACT_WAIT,        /* 待つ */
	ACT_FAN,         /* ファン ON / OFF */
	ACT_END,
} action_t;

typedef struct {
	action_t action;
	int a;   /* MOVE: サーボ ID / WAIT: ms / FAN: 0 or 1 */
	int b;   /* MOVE: 角度 x10 */
	int c;   /* MOVE: 時間 x10ms */
} step_t;

static const step_t SEQ_CATCH[] = {
	{ ACT_TORQUE_ON, 0, 0, 0 },
	{ ACT_MOVE, SERVO_ID_UPPER, ARM_CATCH_UPPER, ARM_CATCH_TIME },
	{ ACT_MOVE, SERVO_ID_LOWER, ARM_CATCH_LOWER, ARM_CATCH_TIME },
	{ ACT_WAIT, ARM_CATCH_WAIT_MS, 0, 0 },
	{ ACT_FAN, 1, 0, 0 },
	{ ACT_END, 0, 0, 0 },
};

static const step_t SEQ_HOME[] = {
	{ ACT_TORQUE_ON, 0, 0, 0 },
	{ ACT_MOVE, SERVO_ID_LOWER, ARM_HOME_LOWER_FIRST, ARM_HOME_TIME },
	{ ACT_WAIT, ARM_HOME_FIRST_WAIT_MS, 0, 0 },
	{ ACT_MOVE, SERVO_ID_UPPER, ARM_HOME_UPPER, ARM_HOME_TIME },
	{ ACT_MOVE, SERVO_ID_LOWER, ARM_HOME_LOWER, ARM_HOME_TIME },
	{ ACT_FAN, 0, 0, 0 },
	{ ACT_WAIT, ARM_HOME_WAIT_MS, 0, 0 },
	{ ACT_END, 0, 0, 0 },
};

static const step_t SEQ_THROW[] = {
	{ ACT_TORQUE_ON, 0, 0, 0 },
	{ ACT_MOVE, SERVO_ID_UPPER, ARM_THROW_READY_UPPER, ARM_THROW_READY_TIME },
	{ ACT_MOVE, SERVO_ID_LOWER, ARM_THROW_READY_LOWER, ARM_THROW_READY_TIME },
	{ ACT_WAIT, ARM_THROW_READY_WAIT_MS, 0, 0 },
	{ ACT_MOVE, SERVO_ID_UPPER, ARM_THROW_UPPER, ARM_THROW_TIME },
	{ ACT_WAIT, ARM_THROW_UPPER_WAIT_MS, 0, 0 },
	{ ACT_MOVE, SERVO_ID_LOWER, ARM_THROW_LOWER, ARM_THROW_TIME },
	{ ACT_WAIT, ARM_THROW_END_WAIT_MS, 0, 0 },
	{ ACT_END, 0, 0, 0 },
};

static const step_t *seq = NULL;
static int step_index = 0;
static uint32_t wait_until = 0;
static int waiting = 0;

void arm_init(void) {
	fan_init();
	seq = NULL;
}

static int start(const step_t *s) {
	if (seq != NULL) {
		return 0;
	}
	seq = s;
	step_index = 0;
	waiting = 0;
	return 1;
}

int arm_catch(void) { return start(SEQ_CATCH); }
int arm_home(void)  { return start(SEQ_HOME); }
int arm_throw(void) { return start(SEQ_THROW); }

int arm_is_busy(void) {
	return seq != NULL;
}

void arm_update(void) {
	while (seq != NULL) {
		const step_t *s = &seq[step_index];

		if (waiting) {
			if ((int32_t)(time_ms() - wait_until) < 0) {
				return;   /* まだ待つ */
			}
			waiting = 0;
			step_index++;
			continue;
		}

		switch (s->action) {
		case ACT_TORQUE_ON:
			servo_torque(SERVO_ID_LOWER, 1);
			servo_torque(SERVO_ID_UPPER, 1);
			break;
		case ACT_MOVE:
			servo_move((uint8_t)s->a, (int16_t)s->b, (uint16_t)s->c);
			break;
		case ACT_WAIT:
			wait_until = time_ms() + (uint32_t)s->a;
			waiting = 1;
			return;
		case ACT_FAN:
			fan_set(s->a);
			break;
		case ACT_END:
			seq = NULL;
			return;
		}
		step_index++;
	}
}
