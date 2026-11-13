/*
 * app_auto.c を PC 上で動かすテスト（走行の計算 ctl_motion.c も本物を使う）。
 *   make -C test     （software/firmware/MySWEEP1 で実行）
 *
 * でたらめな迷路の中で、にせの壁センサの値を機体の位置から作り、次を確かめる。
 *   - ゲートが開くまでは動かない
 *   - 壁を通り抜けず、相手陣地にも入らない
 *   - ボールの区画に入らない（ボールを押さない）
 *   - ボールの隣でボールの方を向いてつかみ、相手陣地との壁の 1 マス手前から、その壁に向けて投げる
 *   - config.h のボール（AUTO_BALLS）を全部投げて止まる
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"
#include "app_auto.h"
#include "app_arm.h"
#include "ctl_motion.h"
#include "drv_time.h"
#include "fake_hw.h"
#include "sim_maze.h"

#define TRIALS 300
#define GATE_OPEN_MS 2000
#define TIME_LIMIT_MS (10 * 60 * 1000)

static int failures = 0;
#define FAIL(...) do { printf("NG: " __VA_ARGS__); printf("\n"); failures++; } while (0)

/* ---- にせの時間とアーム ---- */
static uint32_t now_ms;
uint32_t time_ms(void) { return now_ms; }
void time_init(void) {}
void time_delay_us(uint32_t us) { (void)us; }

typedef enum { ARM_NONE, ARM_CATCH, ARM_CARRY, ARM_THROW, ARM_HOME } arm_act_t;
static arm_act_t arm_last;
static uint32_t arm_until;
static int arm_start(arm_act_t a, uint32_t ms) {
	if (now_ms < arm_until) return 0;
	arm_last = a;
	arm_until = now_ms + ms;
	return 1;
}
void arm_init(void) {}
void arm_update(void) {}
int arm_is_busy(void) { return now_ms < arm_until; }
/* 下の 4 つは、呼ばれたときに機体の位置が正しいかを調べる */
static int catch_ok(void);
static int throw_ok(void);
static int catches, bad_catch, bad_throw;
static int pushed;   /* ボールを押してもよいことにした迷路の数 */
static int caught[AUTO_BALL_COUNT];
int arm_catch(void) { if (!arm_start(ARM_CATCH, 1500)) return 0; catches++;  if (!catch_ok()) bad_catch++; return 1; }
int arm_carry(void) { return arm_start(ARM_CARRY, 1000); }
static void report_throw(void);
int arm_throw(void) { if (!arm_start(ARM_THROW, 1500)) return 0; if (!throw_ok()) { bad_throw++; report_throw(); } return 1; }
int arm_home(void) { return arm_start(ARM_HOME, 1500); }

/* ---- 機体の位置（フィールドの座標 [mm]） ---- */
static const int8_t BALLS[AUTO_BALL_COUNT][2] = AUTO_BALLS;
static double fx, fy, fdeg;

static void field_pose(void) {
	pose_t p = motion_pose();
	double a0 = AUTO_START_DIR * 90.0 * M_PI / 180.0;
	double cx = (AUTO_START_X + 0.5) * CELL_MM, cy = (AUTO_START_Y + 0.5) * CELL_MM;
	fx = cx + p.x * cos(a0) - p.y * sin(a0);
	fy = cy + p.x * sin(a0) + p.y * cos(a0);
	fdeg = AUTO_START_DIR * 90.0 + p.theta;
}
static int cur_cell_x(void) { return (int)floor(fx / CELL_MM); }
static int cur_cell_y(void) { return (int)floor(fy / CELL_MM); }
static int cur_dir(void) { return ((int)lround(fdeg / 90.0) % 4 + 4) % 4; }

/* 区画の中央からのずれ（進む向きと横）。つかむ・投げるときに確かめる */
static double off_center(void) {
	double dx = fx - (cur_cell_x() + 0.5) * CELL_MM, dy = fy - (cur_cell_y() + 0.5) * CELL_MM;
	return sqrt(dx * dx + dy * dy);
}

/* つかむとき: ボールの隣の区画で、ボールの方を向き、AUTO_APPROACH_MM 前に出ている */
static int catch_ok(void) {
	field_pose();
	int d = cur_dir();
	double bx = fx + (180.0 - AUTO_APPROACH_MM) * DX[d], by = fy + (180.0 - AUTO_APPROACH_MM) * DY[d];
	for (int i = 0; i < AUTO_BALL_COUNT; i++) {
		double cx = (BALLS[i][0] + 0.5) * CELL_MM, cy = (BALLS[i][1] + 0.5) * CELL_MM;
		if (fabs(bx - cx) < 20 && fabs(by - cy) < 20 && fabs(fdeg - lround(fdeg / 90.0) * 90.0) < 2) {
			if (caught[i]) return 0;   /* 同じボールを 2 回つかもうとした */
			caught[i] = 1;
			return 1;
		}
	}
	return 0;
}

/* 投げるとき: 区画の中央にいて、前の区画とは壁がなく、その先が相手陣地との壁 */
static int throw_ok(void) {
	field_pose();
	int x = cur_cell_x(), y = cur_cell_y(), d = cur_dir();
	if (off_center() > 20 || fabs(fdeg - lround(fdeg / 90.0) * 90.0) > 2) return 0;
	int ax = x + DX[d], ay = y + DY[d], bx = ax + DX[d], by = ay + DY[d];
	if (!in(ax, ay) || !in(bx, by)) return 0;
	return ours[x][y] && ours[ax][ay] && !ours[bx][by] && !twall(x, y, d) && twall(ax, ay, d);
}

static void report_throw(void) {
	if (!getenv("SEED")) return;
	field_pose();
	int x = cur_cell_x(), y = cur_cell_y(), d = cur_dir();
	cell_t c = auto_cell();
	printf("  投げた: 本当 (%d,%d) 向き %d ずれ %.1fmm 角度 %.1f / 推定 (%d,%d) 向き %d\n",
			x, y, d, off_center(), fdeg, c.x, c.y, auto_heading());
	int ax = x + DX[d], ay = y + DY[d], bx = ax + DX[d], by = ay + DY[d];
	printf("  ours here %d, a(%d,%d) ours %d, b(%d,%d) ours %d, wall here→a %d, wall a→b %d\n",
			ours[x][y], ax, ay, in(ax, ay) ? ours[ax][ay] : -1, bx, by, in(bx, by) ? ours[bx][by] : -1,
			twall(x, y, d), in(ax, ay) ? twall(ax, ay, d) : -1);
}

/*
 * 機体の位置から、にせの壁センサの値を作る。
 * 横のセンサは、区画の中央にいるとき IR_CENTER_*、壁に 1mm 近づくごとに IR_PER_MM 増える。
 * 向きが傾くと、片方の壁に近づくのと同じように見える（センサは車軸より前にあるため）。
 */
#define IR_PER_MM 20.0
#define SENSOR_AHEAD_MM 50.0
static void set_ir(void) {
	field_pose();
	int x = cur_cell_x(), y = cur_cell_y(), d = cur_dir();
	if (!in(x, y)) return;
	int l = (d + 1) & 3, r = (d + 3) & 3;
	/* 進む向きに対して左がプラスの横ずれ（センサの位置で） */
	double cx = (x + 0.5) * CELL_MM, cy = (y + 0.5) * CELL_MM;
	double err_deg = fdeg - d * 90.0;
	err_deg -= 360.0 * floor((err_deg + 180.0) / 360.0);
	double lat = -(fx - cx) * DY[d] + (fy - cy) * DX[d] + SENSOR_AHEAD_MM * sin(err_deg * M_PI / 180.0);
	double vl = IR_CENTER_LEFT + IR_PER_MM * lat, vr = IR_CENTER_RIGHT - IR_PER_MM * lat;
	fake_ir[IR_LEFT] = twall(x, y, l) ? (int32_t)(vl < 0 ? 0 : vl) : 0;
	fake_ir[IR_RIGHT] = twall(x, y, r) ? (int32_t)(vr < 0 ? 0 : vr) : 0;
	int front = twall(x, y, d);
	if (now_ms < GATE_OPEN_MS && x == AUTO_START_X && y == AUTO_START_Y && d == AUTO_START_DIR) front = 1;   /* ゲート */
	fake_ir[IR_FRONT_LEFT] = fake_ir[IR_FRONT_RIGHT] = front ? IR_WALL_FRONT + 200 : 0;
}

static void reset_sim(void) {
	for (int i = 0; i < STEPPER_COUNT; i++) { fake_wheel_mm[i] = 0; fake_wheel_speed[i] = 0; }
	now_ms = 0;
	arm_until = 0;
	catches = bad_catch = bad_throw = 0;
	for (int i = 0; i < AUTO_BALL_COUNT; i++) caught[i] = 0;
	motion_init();
	motion_reset_pose();
}

static void run_trial(unsigned seed, int *done, int64_t *total_ms) {
	make_maze(seed, 6);   /* AUTO_BALLS（x < 6）が自分の陣地に入るようにする */
	sim_open(AUTO_START_X, AUTO_START_Y, AUTO_START_DIR);   /* ゲートの先は通れる */
	reset_sim();
	set_ir();
	auto_init();
	int px = AUTO_START_X, py = AUTO_START_Y;
	int moved_before_gate = 0, pushed_noted = 0;
	for (now_ms = 1; now_ms < TIME_LIMIT_MS; now_ms++) {
		set_ir();
		motion_tick();
		fake_advance_1ms();
		auto_loop();
		field_pose();
		int x = cur_cell_x(), y = cur_cell_y();
		if (now_ms < GATE_OPEN_MS && (fabs(motion_speed()) > 0 || fabs(motion_turn_speed()) > 0)) moved_before_gate = 1;
		if (x != px || y != py) {
			int d = -1;
			for (int k = 0; k < 4; k++) if (px + DX[k] == x && py + DY[k] == y) d = k;
			if (d < 0 || !in(x, y)) { FAIL("迷路 %u: 区画を飛ばした (%d,%d)→(%d,%d)", seed, px, py, x, y); return; }
			if (twall(px, py, d)) { FAIL("迷路 %u: 壁を通り抜けた (%d,%d)→(%d,%d)", seed, px, py, x, y); return; }
			if (!ours[x][y]) { FAIL("迷路 %u: 相手陣地に入った", seed); return; }
			for (int i = 0; i < AUTO_BALL_COUNT; i++)
				if (BALLS[i][0] == x && BALLS[i][1] == y && !caught[i] && maze_avoided((cell_t){ BALLS[i][0], BALLS[i][1] }))
					{ FAIL("迷路 %u: ボールの区画 (%d,%d) に入った", seed, x, y); return; }
			px = x; py = y;
		}
		for (int i = 0; i < AUTO_BALL_COUNT; i++)
			if (!caught[i] && !maze_avoided((cell_t){ BALLS[i][0], BALLS[i][1] }) && !pushed_noted) { pushed++; pushed_noted = 1; }
		auto_status_t st = auto_status();
		if (st == AUTO_DONE || st == AUTO_STUCK) break;
	}
	if (moved_before_gate) FAIL("迷路 %u: ゲートが開く前に動いた", seed);
	if (bad_catch) FAIL("迷路 %u: つかむ位置がずれている", seed);
	if (bad_throw) FAIL("迷路 %u: 投げる位置・向きが正しくない", seed);
	if (auto_status() != AUTO_DONE) { FAIL("迷路 %u: 終わらない（状態 %d、%d 回投げた）", seed, auto_status(), auto_throw_count()); return; }
	if (auto_throw_count() != AUTO_BALL_COUNT) { FAIL("迷路 %u: 投げた数が %d", seed, auto_throw_count()); return; }
	(*done)++;
	*total_ms += now_ms - GATE_OPEN_MS;
}

#ifndef DBG
int main(void) {
	int done = 0;
	int64_t total = 0;
	const char *one = getenv("SEED");   /* SEED=5294 ./test_auto で 1 つだけ */
	if (one) { run_trial((unsigned)atoi(one), &done, &total); }
	else for (unsigned t = 0; t < TRIALS; t++) run_trial(5000 + t, &done, &total);
	printf("迷路 %d 個: 全部投げて終わった %d（うち、ボールの区画を通らないと投げる場所に行けなかった %d）", TRIALS, done, pushed);
	if (done) printf("（ゲートが開いてから平均 %.0f 秒）", (double)total / done / 1000.0);
	printf("\n");
	printf(failures ? "\n%d 件失敗\n" : "\nすべて OK\n", failures);
	return failures ? 1 : 0;
}
#endif
