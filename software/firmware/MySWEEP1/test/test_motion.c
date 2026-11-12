/*
 * motion.c を PC 上で動かすテスト。
 * モーターは「指定した速さどおりにパルスが出る」として真似する。
 *
 *   make -C test     （software/firmware/MySWEEP1 で実行）
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "config.h"
#include "motor.h"
#include "motion.h"
#include "wall_sensor.h"

/* ---- motor / wall_sensor の真似 ---- */
static double pos_mm[2];
/* 回るときの動き出し（車輪の速さ）がパルスの下限より速いこと */
_Static_assert(MOTION_TURN_START_SPEED * 3.14159f / 180.0f * TREAD_MM / 2.0f > WHEEL_MIN_SPEED, "turn start too slow");
_Static_assert(MOTION_START_SPEED > WHEEL_MIN_SPEED, "start too slow");
static float spd[2];
static int enabled;
static float max_wheel;
float motor_step_mm(void) { return (float)M_PI * TIRE_DIAMETER_MM * STEP_ANGLE_DEG / 360.0f; }
void motor_enable(void) { enabled = 1; }
void motor_disable(void) { enabled = 0; }
int motor_is_enabled(void) { return enabled; }
void motor_set_speed(float l, float r) {
	spd[0] = l; spd[1] = r;
	if (fabsf(l) > max_wheel) max_wheel = fabsf(l);
	if (fabsf(r) > max_wheel) max_wheel = fabsf(r);
}
int32_t motor_get_steps(wheel_t w) { return (int32_t)(pos_mm[w] / motor_step_mm()); }
int32_t wall_sensor_get(int i) { (void)i; return 0; }

static void run_ms(int ms) {
	for (int i = 0; i < ms; i++) {
		motion_tick();
		for (int w = 0; w < 2; w++) {
			if (fabsf(spd[w]) >= WHEEL_MIN_SPEED) pos_mm[w] += spd[w] * CONTROL_DT;
		}
	}
}

static int failures = 0;
static void check(const char *name, double got, double want, double tol) {
	int ok = fabs(got - want) <= tol;
	printf("%-36s %10.2f (期待 %8.2f ±%.1f) %s\n", name, got, want, tol, ok ? "OK" : "NG");
	if (!ok) failures++;
}

int main(void) {
	motion_init();

	/* 1 区画まっすぐ */
	double d0 = motion_get_distance(); int t = 0; max_wheel = 0;
	motion_straight(CELL_MM, 500, 0, 2000, 0);
	while (motion_is_busy() && t < 5000) { run_ms(1); t++; }
	check("1区画 進んだ距離 [mm]", motion_get_distance() - d0, CELL_MM, 1.0);
	check("1区画 最高速度 [mm/s]", max_wheel, 500, 0.1);
	check("1区画 終了後の速度", motion_is_stopped(), 1, 0);
	printf("    かかった時間 %d ms\n", t);

	/* 加速が足りないとき（最高速度まで届かない） */
	d0 = motion_get_distance(); t = 0; max_wheel = 0;
	motion_straight(CELL_MM, 3000, 0, 2000, 0);
	while (motion_is_busy() && t < 5000) { run_ms(1); t++; }
	check("短い距離 進んだ距離 [mm]", motion_get_distance() - d0, CELL_MM, 1.0);
	/* 動き出し 100mm/s から加速・減速する三角形: 最高速度 = sqrt(a*d + v0^2) */
	check("短い距離 最高速度 [mm/s]", max_wheel, sqrt(2000.0 * CELL_MM + 100.0 * 100.0), 10.0);

	/* 後ろに 90mm */
	d0 = motion_get_distance(); t = 0;
	motion_straight(-90, 300, 0, 1000, 0);
	while (motion_is_busy() && t < 5000) { run_ms(1); t++; }
	check("後退 進んだ距離 [mm]", motion_get_distance() - d0, -90, 1.0);

	/* その場で左 90 度、右 180 度 */
	double a0 = motion_get_angle(); t = 0;
	motion_turn(90, 360, 2000);
	while (motion_is_busy() && t < 5000) { run_ms(1); t++; }
	check("左90度 回った角度 [deg]", motion_get_angle() - a0, 90, 1.0);
	a0 = motion_get_angle(); t = 0;
	motion_turn(-180, 360, 2000);
	while (motion_is_busy() && t < 5000) { run_ms(1); t++; }
	check("右180度 回った角度 [deg]", motion_get_angle() - a0, -180, 1.0);

	/* 進み続ける（end_speed > 0）→ そのあと止める */
	d0 = motion_get_distance();
	motion_straight(CELL_MM, 400, 300, 2000, 0);
	while (motion_is_busy()) run_ms(1);
	check("連続走行 1区画後の距離 [mm]", motion_get_distance() - d0, CELL_MM, 2.0);
	run_ms(100);
	check("連続走行 0.1秒後の左車輪 [mm/s]", spd[0], 300, 0.1);

	/* リモコン: 押している間 300mm/s、離すと止まる */
	motion_set_velocity(REMOTE_STRAIGHT_SPEED, REMOTE_STRAIGHT_ACCEL, 0, REMOTE_TURN_ACCEL, 0);
	run_ms(1000);
	check("リモコン前進 速度 [mm/s]", spd[0], REMOTE_STRAIGHT_SPEED, 0.1);
	motion_set_velocity(0, REMOTE_STRAIGHT_ACCEL, 0, REMOTE_TURN_ACCEL, 0);
	run_ms(400);
	check("リモコン離す 止まった", motion_is_stopped(), 1, 0);

	/* リモコン: 左に回る（左車輪が前、右車輪が後ろ：元のプログラムと同じ） */
	motion_set_velocity(0, REMOTE_STRAIGHT_ACCEL, REMOTE_TURN_SPEED, REMOTE_TURN_ACCEL, 0);
	run_ms(1000);
	double wheel = REMOTE_TURN_SPEED * M_PI / 180.0 * TREAD_MM / 2.0;
	check("リモコン左旋回 左車輪 [mm/s]", spd[0], wheel, 0.1);
	check("リモコン左旋回 右車輪 [mm/s]", spd[1], -wheel, 0.1);

	printf(failures ? "\n%d 件失敗\n" : "\nすべて OK\n", failures);
	return failures ? 1 : 0;
}
