/*
 * motor.c
 *
 * パルスの周期 = タイマーの ARR。デューティ 50% でパルスを出す。
 * 周期が終わるたびに更新割り込みが入るので、そこで
 *   1. 今の周期でパルスを出していたら、パルス数を数える
 *   2. 次の周期の速度と回転方向を反映する
 * をしている（元のプログラムの motor_set_speedl / motor_set_speedr と同じ考え方）。
 */

#include "motor.h"

#include <math.h>
#include "main.h"
#include "tim.h"
#include "config.h"

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;
	GPIO_PinState dir_forward;   /* 前に進むときの回転方向ピンの値 */
	volatile float speed;        /* 指定された速度 [mm/s] */
	volatile int32_t steps;      /* 出したパルスの合計 */
	volatile int8_t pulsing;     /* 今の周期でパルスを出しているときの向き（+1 / -1 / 0） */
} wheel_state_t;

/* ピンの組み合わせは元のプログラムと同じ（config.h の「左右について」を参照） */
static wheel_state_t wheels[2] = {
	[WHEEL_LEFT] = {
		.htim = &htim3, .channel = TIM_CHANNEL_3,           /* PC8 */
		.dir_port = MOTOR_CW_CCW_L_GPIO_Port, .dir_pin = MOTOR_CW_CCW_L_Pin, /* PB15 */
		.dir_forward = GPIO_PIN_RESET,
	},
	[WHEEL_RIGHT] = {
		.htim = &htim8, .channel = TIM_CHANNEL_4,           /* PC9 */
		.dir_port = MOTOR_CW_CCW_R_GPIO_Port, .dir_pin = MOTOR_CW_CCW_R_Pin, /* PB14 */
		.dir_forward = GPIO_PIN_SET,
	},
};

static volatile int enabled = 0;
static float step_mm;

float motor_step_mm(void) {
	return step_mm;
}

void motor_init(void) {
	step_mm = (float)M_PI * TIRE_DIAMETER_MM * STEP_ANGLE_DEG / 360.0f;

	for (int i = 0; i < 2; i++) {
		wheel_state_t *w = &wheels[i];
		__HAL_TIM_SET_COMPARE(w->htim, w->channel, 0);   /* パルスなし */
		HAL_TIM_PWM_Start(w->htim, w->channel);
		HAL_TIM_Base_Start_IT(w->htim);
	}
	motor_disable();
}

void motor_enable(void) {
	if (enabled) {
		return;
	}
	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_SET);
	/* モータードライバをリセットして励磁する */
	HAL_GPIO_WritePin(MD_RESET_GPIO_Port, MD_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(3);
	HAL_GPIO_WritePin(MD_RESET_GPIO_Port, MD_RESET_Pin, GPIO_PIN_RESET);
	enabled = 1;
}

void motor_disable(void) {
	motor_set_speed(0.0f, 0.0f);
	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_RESET);
	enabled = 0;
}

int motor_is_enabled(void) {
	return enabled;
}

void motor_set_speed(float left_mm_s, float right_mm_s) {
	float speeds[2] = { left_mm_s, right_mm_s };
	for (int i = 0; i < 2; i++) {
		wheel_state_t *w = &wheels[i];
		int was_stopped = (w->pulsing == 0);
		w->speed = speeds[i];
		/* 止まっていた車輪は、次の更新を待たずにすぐ動き出させる */
		if (was_stopped && fabsf(speeds[i]) >= WHEEL_MIN_SPEED) {
			w->htim->Instance->EGR = TIM_EGR_UG;  /* 割り込みの中からも呼べるようにレジスタを直接書く */
		}
	}
}

int32_t motor_get_steps(wheel_t wheel) {
	return wheels[wheel].steps;
}

void motor_on_timer_update(wheel_t wheel) {
	wheel_state_t *w = &wheels[wheel];

	/* 1. 終わった周期のパルスを数える */
	w->steps += w->pulsing;

	/* 2. 次の周期を設定する */
	float v = w->speed;
	float speed = fabsf(v);
	if (!enabled || speed < WHEEL_MIN_SPEED) {
		__HAL_TIM_SET_COMPARE(w->htim, w->channel, 0);
		__HAL_TIM_SET_AUTORELOAD(w->htim, UINT16_MAX);
		w->pulsing = 0;
		return;
	}

	float count = STEP_TIMER_HZ * step_mm / speed;
	if (count > (float)UINT16_MAX) {
		count = (float)UINT16_MAX;
	}
	uint16_t arr = (uint16_t)(count + 0.5f) - 1;

	HAL_GPIO_WritePin(w->dir_port, w->dir_pin,
			(v >= 0) ? w->dir_forward :
			(w->dir_forward == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET));
	__HAL_TIM_SET_AUTORELOAD(w->htim, arr);
	__HAL_TIM_SET_COMPARE(w->htim, w->channel, arr / 2);
	w->pulsing = (v >= 0) ? 1 : -1;
}
