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
static int blocked_ok;   /* もう 1 台が唯一の道をふさいでいて、待ち続けた迷路の数 */
static int caught[AUTO_BALL_COUNT];
int arm_catch(void) { if (!arm_start(ARM_CATCH, 1500)) return 0; catches++;  if (!catch_ok()) bad_catch++; return 1; }
int arm_carry(void) { return arm_start(ARM_CARRY, 1000); }
static void report_throw(void);
int arm_throw(void) { if (!arm_start(ARM_THROW, 1500)) return 0; if (!throw_ok()) { bad_throw++; report_throw(); } return 1; }
static int homed;
static uint32_t arm_until_home;
int arm_home(void) { if (!arm_start(ARM_HOME, 1500)) return 0; if (!homed) arm_until_home = arm_until; homed = 1; return 1; }

/*
 * ---- にせのもう 1 台 ----
 * TEST_PEER のとき、ボールまでの道の途中の区画に、もう 1 台が止まったままいる（故障した、など）。
 * 機体はその区画に入らず、回り道があれば回り道をする。壁センサには、もう 1 台が壁のように見える。
 */
#include "drv_link.h"
static int peer_on;
static int peer_x = -1, peer_y = -1;
static int peer_entered;
int link_peer(link_peer_t *out) {
	if (!peer_on) return 0;
	out->id = AUTO_ROBOT_ID == 0 ? 1 : 0;
	out->x = out->nx = (int8_t)peer_x;
	out->y = out->ny = (int8_t)peer_y;
	out->status = 1;
	return 1;
}
static int sent_count;
int link_send_self(const link_peer_t *self) { (void)self; sent_count++; return 1; }

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
	int is_peer_l = peer_on && x + DX[l] == peer_x && y + DY[l] == peer_y;
	int is_peer_r = peer_on && x + DX[r] == peer_x && y + DY[r] == peer_y;
	int is_peer_f = peer_on && x + DX[d] == peer_x && y + DY[d] == peer_y;
	fake_ir[IR_LEFT] = (twall(x, y, l) || is_peer_l) ? (int32_t)(vl < 0 ? 0 : vl) : 0;
	fake_ir[IR_RIGHT] = (twall(x, y, r) || is_peer_r) ? (int32_t)(vr < 0 ? 0 : vr) : 0;
	int front = twall(x, y, d) || is_peer_f;
	if (now_ms < GATE_OPEN_MS && x == AUTO_START_X && y == AUTO_START_Y && d == AUTO_START_DIR) front = 1;   /* ゲート */
	fake_ir[IR_FRONT_LEFT] = fake_ir[IR_FRONT_RIGHT] = front ? IR_WALL_FRONT + 200 : 0;
}

static void reset_sim(void) {
	for (int i = 0; i < STEPPER_COUNT; i++) { fake_wheel_mm[i] = 0; fake_wheel_speed[i] = 0; }
	now_ms = 0;
	arm_until = 0;
	catches = bad_catch = bad_throw = 0;
	homed = 0;
	arm_until_home = 0;
	for (int i = 0; i < AUTO_BALL_COUNT; i++) caught[i] = 0;
	motion_init();
	motion_reset_pose();
}

/* 本当の迷路で a から b への最短の歩数（区画 skip は通らない）。行けなければ -1 */
static int skip_balls;   /* 1 のとき、行き先以外のボールの区画も通らない */
static int true_dist(int ax, int ay, int bx, int by, int skip_x, int skip_y, int path[][2], int *plen) {
	static int dist[MAZE_SIZE][MAZE_SIZE], prev[MAZE_SIZE][MAZE_SIZE];
	static int q[MAZE_SIZE * MAZE_SIZE][2];
	for (int x = 0; x < MAZE_SIZE; x++) for (int y = 0; y < MAZE_SIZE; y++) dist[x][y] = -1;
	int h = 0, t = 0;
	dist[ax][ay] = 0; q[t][0] = ax; q[t][1] = ay; t++;
	while (h < t) {
		int x = q[h][0], y = q[h][1]; h++;
		for (int d = 0; d < 4; d++) {
			int nx = x + DX[d], ny = y + DY[d];
			if (!in(nx, ny) || twall(x, y, d) || !ours[nx][ny] || dist[nx][ny] >= 0) continue;
			if (nx == skip_x && ny == skip_y) continue;
			if (skip_balls && !(nx == bx && ny == by)) {
				int is_ball = 0;
				for (int i = 0; i < AUTO_BALL_COUNT; i++) if (BALLS[i][0] == nx && BALLS[i][1] == ny) is_ball = 1;
				if (is_ball) continue;
			}
			dist[nx][ny] = dist[x][y] + 1; prev[nx][ny] = d;
			q[t][0] = nx; q[t][1] = ny; t++;
		}
	}
	if (dist[bx][by] < 0) return -1;
	if (path) {
		int n = dist[bx][by], x = bx, y = by;
		*plen = n + 1;
		for (int i = n; i >= 0; i--) {
			path[i][0] = x; path[i][1] = y;
			if (i > 0) { int d = prev[x][y]; x -= DX[d]; y -= DY[d]; }
		}
	}
	return dist[bx][by];
}

static void run_trial(unsigned seed, int *done, int64_t *total_ms) {
	make_maze(seed, 6);   /* AUTO_BALLS（x < 6）が自分の陣地に入るようにする */
	sim_open(AUTO_START_X, AUTO_START_Y, AUTO_START_DIR);   /* ゲートの先は通れる */
	sim_open(AUTO_START_X, AUTO_START_Y, (AUTO_START_DIR + 2) & 3);   /* 後ろが内側なら通れるようにする（見ていない側） */
	reset_sim();
	peer_on = 0;
	peer_entered = 0;
#ifdef TEST_PEER
	{
		/* 自分のボールのうち最初のものへの道の、4 区画目に置く（ボールの区画のすぐ隣は避ける） */
		static const uint8_t owner[AUTO_BALL_COUNT] = AUTO_BALL_OWNER;
		int bi = 0;
		while (bi < AUTO_BALL_COUNT && AUTO_ROBOT_COUNT >= 2 && owner[bi] != AUTO_ROBOT_ID) bi++;
		static int path[MAZE_SIZE * MAZE_SIZE][2];
		int plen = 0;
		if (true_dist(AUTO_START_X, AUTO_START_Y, BALLS[bi][0], BALLS[bi][1], -1, -1, path, &plen) >= 6) {
			peer_x = path[3][0];
			peer_y = path[3][1];
			peer_on = 1;
		}
	}
#endif
	set_ir();
	auto_init();
	int px = AUTO_START_X, py = AUTO_START_Y;
	int moved_before_gate = 0, moved_before_home = 0, pushed_noted = 0;
	for (now_ms = 1; now_ms < TIME_LIMIT_MS; now_ms++) {
		set_ir();
		motion_tick();
		fake_advance_1ms();
		auto_loop();
		field_pose();
		int x = cur_cell_x(), y = cur_cell_y();
		int moving = fabs(motion_speed()) > 0 || fabs(motion_turn_speed()) > 0;
		if (now_ms < GATE_OPEN_MS && moving) moved_before_gate = 1;
		if (moving && (!homed || now_ms < arm_until_home)) moved_before_home = 1;
		if (x != px || y != py) {
			int d = -1;
			for (int k = 0; k < 4; k++) if (px + DX[k] == x && py + DY[k] == y) d = k;
			if (d < 0 || !in(x, y)) { FAIL("迷路 %u: 区画を飛ばした (%d,%d)→(%d,%d)", seed, px, py, x, y); return; }
			if (twall(px, py, d)) { FAIL("迷路 %u: 壁を通り抜けた (%d,%d)→(%d,%d)", seed, px, py, x, y); return; }
			if (!maze_open((cell_t){ (int8_t)px, (int8_t)py }, (dir_t)d)) { FAIL("迷路 %u: 見ていない壁の所を通った (%d,%d)→(%d,%d)", seed, px, py, x, y); return; }
			if (!ours[x][y]) { FAIL("迷路 %u: 相手陣地に入った", seed); return; }
			if (peer_on && x == peer_x && y == peer_y) { FAIL("迷路 %u: もう 1 台がいる区画 (%d,%d) に入った", seed, x, y); return; }
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
	if (moved_before_home) FAIL("迷路 %u: アームをホームにする前に動いた", seed);
	if (bad_catch) FAIL("迷路 %u: つかむ位置がずれている", seed);
	if (bad_throw) FAIL("迷路 %u: 投げる位置・向きが正しくない", seed);
	if (auto_status() != AUTO_DONE && peer_on) {
		/* もう 1 台とボールの区画を通らずに、自分のボールまで行けるか */
		static const uint8_t owner2[AUTO_BALL_COUNT] = AUTO_BALL_OWNER;
		int ok_path = 1;
		skip_balls = 1;
		for (int i = 0; i < AUTO_BALL_COUNT; i++)
			if ((AUTO_ROBOT_COUNT < 2 || owner2[i] == AUTO_ROBOT_ID)
					&& true_dist(AUTO_START_X, AUTO_START_Y, BALLS[i][0], BALLS[i][1], peer_x, peer_y, NULL, NULL) < 0) ok_path = 0;
		skip_balls = 0;
		if (!ok_path) { blocked_ok++; return; }   /* もう 1 台が唯一の道をふさいでいる。待ち続けるのが正しい */
	}
	if (auto_status() != AUTO_DONE) { FAIL("迷路 %u: 終わらない（状態 %d、%d 回投げた）", seed, auto_status(), auto_throw_count()); return; }
	{
		static const uint8_t owner[AUTO_BALL_COUNT] = AUTO_BALL_OWNER;
		int mine = 0;
		for (int i = 0; i < AUTO_BALL_COUNT; i++) if (AUTO_ROBOT_COUNT < 2 || owner[i] == AUTO_ROBOT_ID) mine++;
		if (auto_throw_count() != mine) { FAIL("迷路 %u: 投げた数が %d（%d 個のはず）", seed, auto_throw_count(), mine); return; }
	}
	(*done)++;
	*total_ms += now_ms - GATE_OPEN_MS;
}

#ifndef DBG_MAIN
int main(void) {
	int done = 0;
	int64_t total = 0;
	const char *one = getenv("SEED");   /* SEED=5294 ./test_auto で 1 つだけ */
	if (one) { run_trial((unsigned)atoi(one), &done, &total); }
	else for (unsigned t = 0; t < TRIALS; t++) run_trial(5000 + t, &done, &total);
	printf("スタート (%d,%d) 向き %d%s / ", AUTO_START_X, AUTO_START_Y, AUTO_START_DIR,
#ifdef TEST_PEER
			"、道の途中にもう 1 台が止まっている"
#else
			""
#endif
			);
	printf("迷路 %d 個: 全部投げて終わった %d（うち、ボールの区画を通らないと投げる場所に行けなかった %d）", TRIALS, done, pushed);
	if (done) printf("（ゲートが開いてから平均 %.0f 秒）", (double)total / done / 1000.0);
	if (blocked_ok) printf("、もう 1 台が唯一の道をふさいでいて待ち続けた %d", blocked_ok);
	printf("\n");
	printf(failures ? "\n%d 件失敗\n" : "\nすべて OK\n", failures);
	return failures ? 1 : 0;
}
#endif
