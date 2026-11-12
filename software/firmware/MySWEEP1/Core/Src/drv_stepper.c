#include "drv_stepper.h"

#include <math.h>
#include "main.h"
#include "tim.h"
#include "config.h"

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;
	GPIO_PinState dir_forward;

	volatile int32_t steps;
	volatile int8_t running_dir;   /* 今の周期のパルスの向き（+1 / -1 / 0 = パルスなし） */
	volatile int8_t next_dir;      /* 次の周期に予約した向き */
	volatile int8_t pin_dir;       /* 方向ピンが今どちら向きか（+1 / -1） */
	volatile int8_t wanted_dir;    /* 切り替えたい向き */
} stepper_t;

#if STEPPER_SWAP_WHEELS
#define RIGHT_TIM &htim8
#define RIGHT_CH  TIM_CHANNEL_4
#define LEFT_TIM  &htim3
#define LEFT_CH   TIM_CHANNEL_3
#else
#define RIGHT_TIM &htim3   /* PC8 = MOTOR_CLOCK_R */
#define RIGHT_CH  TIM_CHANNEL_3
#define LEFT_TIM  &htim8   /* PC9 = MOTOR_CLOCK_L */
#define LEFT_CH   TIM_CHANNEL_4
#endif

static stepper_t steppers[STEPPER_COUNT] = {
	[STEPPER_LEFT] = {
		.htim = LEFT_TIM, .channel = LEFT_CH,
		.dir_port = MOTOR_CW_CCW_R_GPIO_Port, .dir_pin = MOTOR_CW_CCW_R_Pin,   /* PB14 */
		.dir_forward = STEPPER_LEFT_DIR_FORWARD_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET,
	},
	[STEPPER_RIGHT] = {
		.htim = RIGHT_TIM, .channel = RIGHT_CH,
		.dir_port = MOTOR_CW_CCW_L_GPIO_Port, .dir_pin = MOTOR_CW_CCW_L_Pin,   /* PB15 */
		.dir_forward = STEPPER_RIGHT_DIR_FORWARD_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET,
	},
};

static volatile int enabled = 0;
static float step_mm;

static void write_dir_pin(stepper_t *s, int8_t dir) {
	GPIO_PinState back = (s->dir_forward == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
	HAL_GPIO_WritePin(s->dir_port, s->dir_pin, (dir > 0) ? s->dir_forward : back);
	s->pin_dir = dir;
}

static void stop_output(stepper_t *s) {
	__HAL_TIM_SET_COMPARE(s->htim, s->channel, 0);
	__HAL_TIM_SET_AUTORELOAD(s->htim, UINT16_MAX);
	s->next_dir = 0;
}

float stepper_step_mm(void) {
	return step_mm;
}

void stepper_init(void) {
	step_mm = (float)M_PI * TIRE_DIAMETER_MM * STEP_ANGLE_DEG / 360.0f;

	for (int i = 0; i < STEPPER_COUNT; i++) {
		stepper_t *s = &steppers[i];
		s->htim->Instance->CR1 |= TIM_CR1_ARPE;   /* ARR を予約で書く */
		stop_output(s);
		write_dir_pin(s, +1);
		s->wanted_dir = +1;
		HAL_TIM_PWM_Start(s->htim, s->channel);   /* CCR は HAL の設定で予約になっている */
		HAL_TIM_Base_Start_IT(s->htim);
	}
	stepper_disable();
}

void stepper_enable(void) {
	if (enabled) {
		return;
	}
	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD_RESET_GPIO_Port, MD_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(3);
	HAL_GPIO_WritePin(MD_RESET_GPIO_Port, MD_RESET_Pin, GPIO_PIN_RESET);
	enabled = 1;
}

void stepper_disable(void) {
	enabled = 0;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		__disable_irq();
		stop_output(&steppers[i]);
		__enable_irq();
	}
	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_RESET);
}

int stepper_is_enabled(void) {
	return enabled;
}

void stepper_set_speed(stepper_id_t id, float v) {
	stepper_t *s = &steppers[id];
	float speed = fabsf(v);
	int8_t dir = (v >= 0.0f) ? +1 : -1;

	__disable_irq();
	if (!enabled || speed < WHEEL_MIN_SPEED) {
		stop_output(s);
		__enable_irq();
		return;
	}

	float count = STEP_TIMER_HZ * step_mm / speed;
	if (count > 65536.0f) {
		count = 65536.0f;
	}
	uint32_t arr = (uint32_t)(count + 0.5f) - 1U;

	__HAL_TIM_SET_AUTORELOAD(s->htim, arr);
	if (dir != s->pin_dir) {
		/* 向きを変える: 次の周期はパルスを出さず、その始まりで方向ピンを切り替える */
		s->wanted_dir = dir;
		__HAL_TIM_SET_COMPARE(s->htim, s->channel, 0);
		s->next_dir = 0;
	} else {
		__HAL_TIM_SET_COMPARE(s->htim, s->channel, arr / 2U);
		s->next_dir = dir;
	}

	/* 止まっていた（長い周期の途中）ときは、待たずにすぐ新しい周期を始める */
	if (s->running_dir == 0) {
		s->htim->Instance->EGR = TIM_EGR_UG;
	}
	__enable_irq();
}

int32_t stepper_get_steps(stepper_id_t id) {
	return steppers[id].steps;
}

void stepper_on_timer_update(const void *timer) {
	stepper_t *s = ((const void*)steppers[STEPPER_LEFT].htim->Instance == timer)
			? &steppers[STEPPER_LEFT] : &steppers[STEPPER_RIGHT];

	/* 終わった周期のパルスを数える */
	s->steps += s->running_dir;

	/* ここから始まる周期は、予約していた設定で動く */
	s->running_dir = s->next_dir;

	/* パルスを出さない周期の始まりなら、方向ピンを切り替えてよい */
	if (s->running_dir == 0 && s->wanted_dir != s->pin_dir) {
		write_dir_pin(s, s->wanted_dir);
	}
}
