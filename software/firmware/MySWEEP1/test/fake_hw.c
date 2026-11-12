/*
 * テスト用のにせのハードウェア。
 * stepper: 指定した速さどおりに車輪が回る（WHEEL_MIN_SPEED 未満は止まる）
 * ir:      テストから値を入れる
 */
#include <math.h>
#include "config.h"
#include "drv_stepper.h"
#include "drv_ir.h"
#include "fake_hw.h"

double fake_wheel_mm[STEPPER_COUNT];
float fake_wheel_speed[STEPPER_COUNT];
float fake_max_wheel_speed;
int32_t fake_ir[IR_COUNT];
uint32_t fake_ir_updates;
static int enabled;

float stepper_step_mm(void) { return (float)M_PI * TIRE_DIAMETER_MM * STEP_ANGLE_DEG / 360.0f; }
void stepper_init(void) {}
void stepper_enable(void) { enabled = 1; }
void stepper_disable(void) { enabled = 0; }
int stepper_is_enabled(void) { return enabled; }
void stepper_set_speed(stepper_id_t id, float v) {
	fake_wheel_speed[id] = v;
	if (fabsf(v) > fake_max_wheel_speed) fake_max_wheel_speed = fabsf(v);
}
int32_t stepper_get_steps(stepper_id_t id) { return (int32_t)lround(fake_wheel_mm[id] / stepper_step_mm()); }

int32_t ir_get(int i) { return fake_ir[i]; }
uint32_t ir_update_count(void) { return fake_ir_updates; }

void fake_advance_1ms(void) {
	for (int i = 0; i < STEPPER_COUNT; i++) {
		if (fabsf(fake_wheel_speed[i]) >= WHEEL_MIN_SPEED) {
			fake_wheel_mm[i] += fake_wheel_speed[i] * CONTROL_DT;
		}
	}
	fake_ir_updates++;
}
