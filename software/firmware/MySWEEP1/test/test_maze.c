/*
 * maze.c を PC 上で動かすテスト。
 *   make -C test     （software/firmware/MySWEEP1 で実行）
 *
 * でたらめな迷路（中心について点対称に 2 つの陣地に分かれる）を作り、
 * にせの機体を走らせて次を確かめる。
 *   - ボールの区画まで、壁を通り抜けずに着く（行き止まりからも戻れる）
 *   - 投げる場所を見つけられる
 *   - 見つけた投げる場所は、本当に相手陣地との間の壁で、外周の壁ではない
 */
#include <stdio.h>
#include <stdlib.h>
#include "maze.h"

#define N MAZE_SIZE
#define TRIALS 2000

static int failures = 0;
#define FAIL(...) do { printf("NG: " __VA_ARGS__); printf("\n"); failures++; } while (0)

/* ---- 本当の迷路 ---- */
static uint8_t truth[N][N];     /* 壁があるビット（向きの番号） */
static uint8_t ours[N][N];      /* 自分の陣地なら 1 */
static const int DX[4] = { 1, 0, -1, 0 }, DY[4] = { 0, 1, 0, -1 };

static int in(int x, int y) { return x >= 0 && x < N && y >= 0 && y < N; }
static int twall(int x, int y, int d) { return (truth[x][y] >> d) & 1; }
static void topen(int x, int y, int d) {
	truth[x][y] &= (uint8_t)~(1u << d);
	truth[x + DX[d]][y + DY[d]] &= (uint8_t)~(1u << ((d + 2) & 3));
}

static void make_maze(unsigned seed) {
	srand(seed);
	/* 陣地: 行 y では x < s[y] が自分。s[15-y] = 16 - s[y] で点対称になる */
	int s[N];
	for (int y = 0; y < N / 2; y++) {
		s[y] = 5 + rand() % 7;          /* 5〜11 */
		s[N - 1 - y] = N - s[y];
	}
	for (int x = 0; x < N; x++)
		for (int y = 0; y < N; y++) {
			truth[x][y] = 0x0F;
			ours[x][y] = x < s[y];
		}
	/* 自分の陣地の中に道を掘る（深さ優先）。反対側にも同じ道を掘る */
	static int stack[N * N][2];
	static uint8_t seen[N][N];
	for (int x = 0; x < N; x++)
		for (int y = 0; y < N; y++) seen[x][y] = 0;
	int sp = 0;
	stack[sp][0] = 0; stack[sp][1] = 0; sp++; seen[0][0] = 1;
	while (sp > 0) {
		int x = stack[sp - 1][0], y = stack[sp - 1][1];
		int cand[4], nc = 0;
		for (int d = 0; d < 4; d++) {
			int nx = x + DX[d], ny = y + DY[d];
			if (in(nx, ny) && ours[nx][ny] && !seen[nx][ny]) cand[nc++] = d;
		}
		if (nc == 0) { sp--; continue; }
		int d = cand[rand() % nc];
		topen(x, y, d);
		topen(N - 1 - x, N - 1 - y, (d + 2) & 3);
		seen[x + DX[d]][y + DY[d]] = 1;
		stack[sp][0] = x + DX[d]; stack[sp][1] = y + DY[d]; sp++;
	}
	/* ところどころ壁を抜いて、回り道もできるようにする */
	for (int x = 0; x < N; x++)
		for (int y = 0; y < N; y++)
			for (int d = 0; d < 2; d++) {
				int nx = x + DX[d], ny = y + DY[d];
				if (in(nx, ny) && ours[x][y] && ours[nx][ny] && rand() % 100 < 12) {
					topen(x, y, d);
					topen(N - 1 - x, N - 1 - y, (d + 2) & 3);
				}
			}
}

/* ---- にせの機体 ---- */
static cell_t pos;
static dir_t heading;
static int steps;

static void sense(void) {
	maze_update(pos, heading,
			twall(pos.x, pos.y, DIR_LEFT(heading)),
			twall(pos.x, pos.y, heading),
			twall(pos.x, pos.y, DIR_RIGHT(heading)));
}

/* goals のどれかへ行く。着いたら 1、行けないと分かったら 0、壁を通り抜けたら -1 */
static int go(const cell_t *goals, int n) {
	for (int guard = 0; guard < 2000; guard++) {
		maze_flood(goals, n);
		if (maze_dist(pos) == 0) return 1;
		int d = maze_next_dir(pos, heading);
		if (d < 0) return 0;
		if (twall(pos.x, pos.y, d)) return -1;     /* 地図では通れるのに、本当は壁がある */
		heading = (dir_t)d;
		pos = maze_neighbor(pos, heading);
		if (!ours[pos.x][pos.y]) return -1;      /* 相手陣地に入った */
		steps++;
		sense();
	}
	return 0;
}

/* 地図の壁が、本当の壁と食い違っていないか */
static int map_matches_truth(void) {
	for (int x = 0; x < N; x++)
		for (int y = 0; y < N; y++)
			for (int d = 0; d < 4; d++) {
				cell_t c = { (int8_t)x, (int8_t)y };
				if (maze_known(c, (dir_t)d) && maze_wall(c, (dir_t)d) != twall(x, y, d)) return 0;
			}
	return 1;
}

/* 相手陣地と判定した区画が、本当に相手陣地か */
static int opponent_matches_truth(void) {
	for (int x = 0; x < N; x++)
		for (int y = 0; y < N; y++) {
			cell_t c = { (int8_t)x, (int8_t)y };
			if (maze_is_opponent(c) && ours[x][y]) return 0;
		}
	return 1;
}

/* 本当の迷路に、投げる場所（1 つ手前から壁なしで、相手陣地との壁に向ける）があるか */
static int true_spot_exists(void) {
	for (int x = 0; x < N; x++)
		for (int y = 0; y < N; y++)
			for (int d = 0; d < 4; d++) {
				int bx = x + DX[d], by = y + DY[d], fx = x - DX[d], fy = y - DY[d];
				if (!in(bx, by) || !in(fx, fy)) continue;
				if (ours[x][y] && !ours[bx][by] && ours[fx][fy] && !twall(fx, fy, d)) return 1;
			}
	return 0;
}

static void test_basics(void) {
	maze_init();
	cell_t c = { 0, 0 };
	if (!maze_wall(c, DIR_S) || !maze_wall(c, DIR_W)) FAIL("外周の壁が入っていない");
	if (maze_known(c, DIR_N)) FAIL("最初は内側の壁は分からない");
	maze_update(c, DIR_N, 1, 0, 1);   /* 北を向いて、左（西）壁あり・前なし・右（東）壁あり */
	if (!maze_open(c, DIR_N)) FAIL("前の壁なしが入っていない");
	if (!maze_wall(c, DIR_E)) FAIL("右の壁が入っていない");
	if (!maze_wall((cell_t){ 1, 0 }, DIR_W)) FAIL("隣の区画の側にも壁を書いていない");
	maze_set_wall(c, DIR_W, 0);
	if (!maze_wall(c, DIR_W)) FAIL("外周の壁が消えた");
	if (!maze_is_opponent((cell_t){ N - 1, N - 1 })) FAIL("通った区画の反対側が相手陣地にならない");
	if (maze_is_opponent((cell_t){ 1, 1 })) FAIL("通っていない区画の反対側が相手陣地になった");
	if (DIR_LEFT(DIR_N) != DIR_W || DIR_RIGHT(DIR_N) != DIR_E || DIR_BACK(DIR_E) != DIR_W)
		FAIL("向きの計算がおかしい");
}

static void test_runs(void) {
	int reached = 0, found = 0, total_steps = 0, search_steps = 0, no_spot = 0;
	for (unsigned t = 0; t < TRIALS; t++) {
		make_maze(1000 + t);
		maze_init();
		pos = (cell_t){ 0, 0 };
		heading = DIR_N;
		steps = 0;
		sense();

		/* 1. ボールの区画（自分の陣地のどこか）へ行く */
		cell_t ball;
		do {
			ball.x = (int8_t)(rand() % N);
			ball.y = (int8_t)(rand() % N);
		} while (!ours[ball.x][ball.y]);
		int r = go(&ball, 1);
		if (r < 0) { FAIL("迷路 %u: 壁を通り抜けた / 相手陣地に入った", t); continue; }
		if (r == 0) { FAIL("迷路 %u: ボールの区画 (%d,%d) に着けない", t, ball.x, ball.y); continue; }
		reached++;
		total_steps += steps;

		/* 2. 投げる場所を探す */
		int s0 = steps;
		throw_spot_t spots[64];
		int ns = 0;
		for (int round = 0; round < 200; round++) {
			ns = maze_find_throw_spots(spots, 64);
			if (ns > 0) break;
			cell_t goals[N * N];
			int ng = maze_throw_search_goals(pos, goals, N * N);
			if (ng == 0) break;
			r = go(goals, ng > N * N ? N * N : ng);
			if (r < 0) { FAIL("迷路 %u: 探す途中で壁を通り抜けた", t); break; }
			if (r == 0) continue;   /* 途中で行けないと分かった。選び直す */
		}
		if (ns == 0) {
			if (true_spot_exists()) FAIL("迷路 %u: 投げる場所があるのに見つからない", t);
			else no_spot++;
			continue;
		}
		found++;

		/* 3. 投げる場所まで行けて、その場所が正しいか */
		cell_t froms[64];
		int nf = ns > 64 ? 64 : ns;
		for (int i = 0; i < nf; i++) froms[i] = spots[i].from;
		r = go(froms, nf);
		if (r != 1) { FAIL("迷路 %u: 投げる場所に行けない", t); continue; }
		search_steps += steps - s0;
		for (int i = 0; i < nf; i++) {
			throw_spot_t sp = spots[i];
			cell_t a = maze_neighbor(sp.from, sp.dir);
			cell_t b = maze_neighbor(a, sp.dir);
			if (!in(b.x, b.y)) FAIL("迷路 %u: 外周の壁に向けて投げる", t);
			else if (ours[b.x][b.y]) FAIL("迷路 %u: 壁の向こうが相手陣地でない", t);
			else if (!twall(a.x, a.y, sp.dir)) FAIL("迷路 %u: 投げる先に壁がない", t);
			else if (twall(sp.from.x, sp.from.y, sp.dir)) FAIL("迷路 %u: 目の前に壁がある", t);
			else if (!ours[sp.from.x][sp.from.y] || !ours[a.x][a.y]) FAIL("迷路 %u: 投げる場所が自分の陣地でない", t);
		}
		if (!map_matches_truth()) FAIL("迷路 %u: 地図が本当の壁と食い違う", t);
		if (!opponent_matches_truth()) FAIL("迷路 %u: 自分の陣地を相手陣地と判定した", t);
	}
	printf("迷路 %d 個: ボールに着いた %d、投げる場所に着いた %d、投げる場所がそもそもない %d\n",
			TRIALS, reached, found, no_spot);
	if (reached) printf("  ボールまで平均 %.1f 区画、そこから投げる場所まで平均 %.1f 区画\n",
			(double)total_steps / reached, found ? (double)search_steps / found : 0.0);
}

int main(void) {
	test_basics();
	test_runs();
	printf(failures ? "\n%d 件失敗\n" : "\nすべて OK\n", failures);
	return failures ? 1 : 0;
}
