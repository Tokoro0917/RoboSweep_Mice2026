/*
 * ctl_motion.c / ctl_wall.c を PC 上で動かすテスト。
 *   make -C test     （software/firmware/MySWEEP1 で実行）
 */
#include <stdio.h>
#include <math.h>
#include "config.h"
#include "ctl_motion.h"
#include "ctl_wall.h"
#include "fake_hw.h"

_Static_assert(MOTION_START_SPEED > WHEEL_MIN_SPEED, "動き出しが遅すぎる");
_Static_assert(MOTION_TURN_START_SPEED * 3.14159f / 180.0f * TREAD_MM / 2.0f > WHEEL_MIN_SPEED,
		"回り始めが遅すぎる");

static int failures = 0;

static void check(const char *name, double got, double want, double tol) {
	int ok = fabs(got - want) <= tol;
	printf("%-40s %9.2f (期待 %8.2f ±%.1f) %s\n", name, got, want, tol, ok ? "OK" : "NG");
	if (!ok) failures++;
}

static void run(int ms) {
	for (int i = 0; i < ms; i++) {
		motion_tick();
		fake_advance_1ms();
	}
}

static int run_until_done(int limit_ms) {
	int t = 0;
	while (motion_is_busy() && t < limit_ms) {
		run(1);
		t++;
	}
	return t;
}

int main(void) {
	motion_init();

	/* --- まっすぐ --- */
	float d0 = motion_distance();
	fake_max_wheel_speed = 0;
	motion_straight(CELL_MM, 500, 0, 2000, 0);
	int t = run_until_done(5000);
	check("1区画: 進んだ距離 [mm]", motion_distance() - d0, CELL_MM, 1.0);
	check("1区画: 最高速度 [mm/s]", fake_max_wheel_speed, 500, 0.1);
	check("1区画: 止まった", motion_is_stopped(), 1, 0);
	check("1区画: 向きは変わらない [deg]", motion_pose().theta, 0, 0.1);
	printf("    かかった時間 %d ms\n", t);

	d0 = motion_distance();
	fake_max_wheel_speed = 0;
	motion_straight(CELL_MM, 3000, 0, 2000, 0);
	run_until_done(5000);
	check("短い距離: 進んだ距離 [mm]", motion_distance() - d0, CELL_MM, 1.0);
	check("短い距離: 最高速度 = sqrt(a*d + v0^2)", fake_max_wheel_speed,
			sqrt(2000.0 * CELL_MM + MOTION_START_SPEED * MOTION_START_SPEED), 10.0);

	d0 = motion_distance();
	motion_straight(-90, 300, 0, 1000, 0);
	run_until_done(5000);
	check("後退: 進んだ距離 [mm]", motion_distance() - d0, -90, 1.0);

	/* --- その場で回る --- */
	motion_reset_pose();
	motion_turn(90, 360, 2000);
	run_until_done(5000);
	check("左90度: 角度 [deg]", motion_pose().theta, 90, 1.0);
	check("左90度: 右車輪が前に回った", fake_wheel_mm[STEPPER_RIGHT] > fake_wheel_mm[STEPPER_LEFT], 1, 0);
	check("左90度: 位置はほぼ動かない [mm]", hypot(motion_pose().x, motion_pose().y), 0, 1.0);
	motion_turn(-180, 360, 2000);
	run_until_done(5000);
	check("右180度: 角度 [deg]", motion_pose().theta, -90, 1.5);

	/* --- 位置（左に 90 度向いてから 1 区画） --- */
	motion_reset_pose();
	motion_turn(90, 360, 2000);
	run_until_done(5000);
	motion_straight(CELL_MM, 500, 0, 2000, 0);
	run_until_done(5000);
	check("左を向いて1区画: x [mm]", motion_pose().x, 0, 3.0);
	check("左を向いて1区画: y [mm]", motion_pose().y, CELL_MM, 3.0);

	/* --- 止まらずに次の区画へ --- */
	d0 = motion_distance();
	motion_straight(CELL_MM, 400, 300, 2000, 0);
	run_until_done(5000);
	check("連続: 1区画目の距離 [mm]", motion_distance() - d0, CELL_MM, 2.0);
	run(100);
	check("連続: そのあとも 300mm/s", motion_speed(), 300, 0.1);
	motion_stop();
	run(10);

	/* --- リモコン --- */
	motion_drive(REMOTE_SPEED, REMOTE_ACCEL, 0, REMOTE_TURN_ACCEL, 0);
	run(1000);
	check("リモコン前進: 速さ [mm/s]", motion_speed(), REMOTE_SPEED, 0.1);
	motion_drive(-REMOTE_SPEED, REMOTE_ACCEL, 0, REMOTE_TURN_ACCEL, 0);
	run(150);
	check("いきなり後退: まず減速する", motion_speed() > 0, 1, 0);
	run(1000);
	check("いきなり後退: 後ろ向きの速さ [mm/s]", motion_speed(), -REMOTE_SPEED, 0.1);
	motion_drive(0, REMOTE_ACCEL, 0, REMOTE_TURN_ACCEL, 0);
	run(400);
	check("リモコン離す: 止まった", motion_is_stopped(), 1, 0);

	motion_drive(0, REMOTE_ACCEL, REMOTE_TURN_SPEED, REMOTE_TURN_ACCEL, 0);
	run(1000);
	double wheel = REMOTE_TURN_SPEED * M_PI / 180.0 * TREAD_MM / 2.0;
	check("リモコン左旋回: 右車輪 [mm/s]", fake_wheel_speed[STEPPER_RIGHT], wheel, 0.1);
	check("リモコン左旋回: 左車輪 [mm/s]", fake_wheel_speed[STEPPER_LEFT], -wheel, 0.1);
	motion_stop();
	run(10);

	/* --- 壁制御 --- */
	fake_ir[IR_LEFT] = IR_CENTER_LEFT;
	fake_ir[IR_RIGHT] = IR_CENTER_RIGHT + 500;   /* 右の壁に近い */
	motion_drive(REMOTE_SPEED, REMOTE_ACCEL, 0, REMOTE_TURN_ACCEL, 1);
	run(1000);
	check("右の壁に近い: 右車輪のほうが速い（左へ寄る）",
			fake_wheel_speed[STEPPER_RIGHT] > fake_wheel_speed[STEPPER_LEFT], 1, 0);
	fake_ir[IR_RIGHT] = IR_CENTER_RIGHT;
	fake_ir[IR_LEFT] = IR_CENTER_LEFT + 500;     /* 左の壁に近い */
	run(10);
	check("左の壁に近い: 左車輪のほうが速い（右へ寄る）",
			fake_wheel_speed[STEPPER_LEFT] > fake_wheel_speed[STEPPER_RIGHT], 1, 0);
	fake_ir[IR_LEFT] = 0;
	fake_ir[IR_RIGHT] = 0;                       /* 両側に壁がない */
	run(10);
	check("壁がない: 補正しない",
			fake_wheel_speed[STEPPER_LEFT] - fake_wheel_speed[STEPPER_RIGHT], 0, 0.01);
	fake_ir[IR_RIGHT] = 10000;                   /* とても近い → 上限で止まる */
	fake_ir[IR_LEFT] = IR_CENTER_LEFT;
	run(10);
	check("補正の上限 [mm/s]",
			(fake_wheel_speed[STEPPER_RIGHT] - fake_wheel_speed[STEPPER_LEFT]) / 2, WALL_MAX_CORRECTION, 0.01);

	printf(failures ? "\n%d 件失敗\n" : "\nすべて OK\n", failures);
	return failures ? 1 : 0;
}
