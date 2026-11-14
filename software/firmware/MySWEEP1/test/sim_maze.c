/* テスト用: でたらめな点対称の迷路 */
#include <stdlib.h>
#include "sim_maze.h"
#define N MAZE_SIZE

/* ---- 本当の迷路 ---- */
uint8_t truth[N][N];     /* 壁があるビット（向きの番号） */
uint8_t ours[N][N];      /* 自分の陣地なら 1 */
const int DX[4] = { 1, 0, -1, 0 }, DY[4] = { 0, 1, 0, -1 };

int in(int x, int y) { return x >= 0 && x < N && y >= 0 && y < N; }
int twall(int x, int y, int d) { return (truth[x][y] >> d) & 1; }
static void topen(int x, int y, int d) {
	truth[x][y] &= (uint8_t)~(1u << d);
	truth[x + DX[d]][y + DY[d]] &= (uint8_t)~(1u << ((d + 2) & 3));
}

void make_maze(unsigned seed, int min_split) {
	srand(seed);
	/* 陣地: 行 y では x < s[y] が自分。s[15-y] = 16 - s[y] で点対称になる */
	int s[N];
	for (int y = 0; y < N / 2; y++) {
		s[y] = min_split + rand() % (12 - min_split);   /* min_split〜11 */
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


void sim_open(int x, int y, int d) {
	int nx = x + DX[d], ny = y + DY[d];
	if (!in(x, y) || !in(nx, ny)) return;
	topen(x, y, d);
	topen(N - 1 - x, N - 1 - y, (d + 2) & 3);
}
