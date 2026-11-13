/*
 * maze.c
 *
 * 迷路の地図と道順（説明は maze.h）。
 */
#include "maze.h"

/* 下位 4 ビット: 壁がある / 上位 4 ビット: 分かっている（ビットの位置は dir_t） */
static uint8_t walls[MAZE_SIZE][MAZE_SIZE];
static uint8_t visited[MAZE_SIZE][MAZE_SIZE];
static uint16_t dist[MAZE_SIZE][MAZE_SIZE];

static const int8_t DX[4] = { 1, 0, -1, 0 };
static const int8_t DY[4] = { 0, 1, 0, -1 };

int maze_in_field(int x, int y) {
	return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

cell_t maze_neighbor(cell_t c, dir_t d) {
	cell_t n = { (int8_t)(c.x + DX[d]), (int8_t)(c.y + DY[d]) };
	return n;
}

static void set_one(cell_t c, dir_t d, int present) {
	if (!maze_in_field(c.x, c.y)) {
		return;
	}
	uint8_t bit = (uint8_t)(1u << d);
	walls[c.x][c.y] |= (uint8_t)(bit << 4);
	if (present) {
		walls[c.x][c.y] |= bit;
	} else {
		walls[c.x][c.y] &= (uint8_t)~bit;
	}
}

void maze_set_wall(cell_t c, dir_t d, int present) {
	if (!maze_in_field(c.x, c.y)) {
		return;
	}
	/* 外周の壁は消さない */
	cell_t n = maze_neighbor(c, d);
	if (!maze_in_field(n.x, n.y)) {
		present = 1;
	}
	set_one(c, d, present);
	set_one(n, DIR_BACK(d), present);
}

void maze_init(void) {
	for (int x = 0; x < MAZE_SIZE; x++) {
		for (int y = 0; y < MAZE_SIZE; y++) {
			walls[x][y] = 0;
			visited[x][y] = 0;
			dist[x][y] = MAZE_DIST_INF;
		}
	}
	for (int i = 0; i < MAZE_SIZE; i++) {
		set_one((cell_t){ (int8_t)i, 0 }, DIR_S, 1);
		set_one((cell_t){ (int8_t)i, MAZE_SIZE - 1 }, DIR_N, 1);
		set_one((cell_t){ 0, (int8_t)i }, DIR_W, 1);
		set_one((cell_t){ MAZE_SIZE - 1, (int8_t)i }, DIR_E, 1);
	}
}

int maze_wall(cell_t c, dir_t d) {
	if (!maze_in_field(c.x, c.y)) {
		return 1;
	}
	return (walls[c.x][c.y] >> d) & 1;
}

int maze_known(cell_t c, dir_t d) {
	if (!maze_in_field(c.x, c.y)) {
		return 1;
	}
	return (walls[c.x][c.y] >> (d + 4)) & 1;
}

int maze_open(cell_t c, dir_t d) {
	return maze_known(c, d) && !maze_wall(c, d);
}

void maze_mark_visited(cell_t c) {
	if (maze_in_field(c.x, c.y)) {
		visited[c.x][c.y] = 1;
	}
}

int maze_visited(cell_t c) {
	return maze_in_field(c.x, c.y) && visited[c.x][c.y];
}

void maze_update(cell_t c, dir_t heading, int left, int front, int right) {
	maze_set_wall(c, DIR_LEFT(heading), left);
	maze_set_wall(c, heading, front);
	maze_set_wall(c, DIR_RIGHT(heading), right);
	maze_mark_visited(c);
}

int maze_is_opponent(cell_t c) {
	if (!maze_in_field(c.x, c.y)) {
		return 0;
	}
	cell_t m = { (int8_t)(MAZE_SIZE - 1 - c.x), (int8_t)(MAZE_SIZE - 1 - c.y) };
	return maze_visited(m);
}

void maze_flood(const cell_t *goals, int n) {
	static cell_t queue[MAZE_SIZE * MAZE_SIZE];
	int head = 0, tail = 0;

	for (int x = 0; x < MAZE_SIZE; x++) {
		for (int y = 0; y < MAZE_SIZE; y++) {
			dist[x][y] = MAZE_DIST_INF;
		}
	}
	for (int i = 0; i < n; i++) {
		cell_t g = goals[i];
		if (!maze_in_field(g.x, g.y) || dist[g.x][g.y] == 0) {
			continue;
		}
		dist[g.x][g.y] = 0;
		queue[tail++] = g;
	}
	while (head < tail) {
		cell_t c = queue[head++];
		uint16_t next = (uint16_t)(dist[c.x][c.y] + 1);
		for (int d = 0; d < 4; d++) {
			if (maze_wall(c, (dir_t)d)) {
				continue;   /* 壁がある（分かっている）。分かっていない壁は「ない」とみなす */
			}
			cell_t nb = maze_neighbor(c, (dir_t)d);
			if (!maze_in_field(nb.x, nb.y) || maze_is_opponent(nb)) {
				continue;
			}
			if (dist[nb.x][nb.y] > next) {
				dist[nb.x][nb.y] = next;
				queue[tail++] = nb;
			}
		}
	}
}

uint16_t maze_dist(cell_t c) {
	if (!maze_in_field(c.x, c.y)) {
		return MAZE_DIST_INF;
	}
	return dist[c.x][c.y];
}

int maze_next_dir(cell_t c, dir_t heading) {
	uint16_t here = maze_dist(c);
	if (here == 0 || here == MAZE_DIST_INF) {
		return -1;
	}
	const dir_t order[4] = { heading, DIR_LEFT(heading), DIR_RIGHT(heading), DIR_BACK(heading) };
	for (int i = 0; i < 4; i++) {
		dir_t d = order[i];
		if (maze_wall(c, d)) {
			continue;
		}
		if (maze_dist(maze_neighbor(c, d)) == here - 1) {
			return d;
		}
	}
	return -1;
}

int maze_find_throw_spots(throw_spot_t *out, int max) {
	int count = 0;
	for (int x = 0; x < MAZE_SIZE; x++) {
		for (int y = 0; y < MAZE_SIZE; y++) {
			cell_t a = { (int8_t)x, (int8_t)y };
			for (int d = 0; d < 4; d++) {
				cell_t beyond = maze_neighbor(a, (dir_t)d);
				cell_t from = maze_neighbor(a, DIR_BACK((dir_t)d));
				if (!maze_in_field(beyond.x, beyond.y) || !maze_in_field(from.x, from.y)) {
					continue;   /* 外周の壁に向けては投げない */
				}
				if (!maze_known(a, (dir_t)d) || !maze_wall(a, (dir_t)d)) {
					continue;
				}
				if (!maze_is_opponent(beyond) || maze_is_opponent(a)) {
					continue;
				}
				if (!maze_open(from, (dir_t)d) || !maze_visited(from)) {
					continue;
				}
				if (count < max) {
					out[count].from = from;
					out[count].dir = (dir_t)d;
				}
				count++;
			}
		}
	}
	return count;
}

int maze_throw_search_goals(cell_t here, cell_t *out, int max) {
	int count = 0;
	maze_flood(&here, 1);   /* 今いる区画から行ける区画だけを選ぶ */
	for (int x = 0; x < MAZE_SIZE; x++) {
		for (int y = 0; y < MAZE_SIZE; y++) {
			cell_t a = { (int8_t)x, (int8_t)y };
			if (maze_is_opponent(a)) {
				continue;
			}
			for (int d = 0; d < 4; d++) {
				cell_t beyond = maze_neighbor(a, (dir_t)d);
				cell_t from = maze_neighbor(a, DIR_BACK((dir_t)d));
				if (!maze_in_field(beyond.x, beyond.y) || !maze_in_field(from.x, from.y)) {
					continue;
				}
				if (!maze_is_opponent(beyond) || maze_open(a, (dir_t)d)) {
					continue;
				}
				cell_t goal;
				if (!maze_known(a, (dir_t)d)) {
					goal = a;          /* 壁を見に行く */
				} else if (!maze_visited(from) && !maze_wall(from, (dir_t)d) && !maze_is_opponent(from)) {
					goal = from;       /* 壁はあると分かった。1 つ手前に行けるか見に行く */
				} else {
					continue;
				}
				if (maze_dist(goal) == MAZE_DIST_INF) {
					continue;
				}
				if (count < max) {
					out[count] = goal;
				}
				count++;
				break;
			}
		}
	}
	if (count > 0) {
		return count;
	}
	/*
	 * 手がかりがなければ、まだ通っていない区画を見に行く（相手陣地の分かる区画が増える）。
	 * 陣地の境目は点対称なのでフィールドの中心を通る。中心に近い区画から先に見る。
	 */
	for (int band = 4; band <= MAZE_SIZE && count == 0; band += 4) {
		for (int x = 0; x < MAZE_SIZE; x++) {
			for (int y = 0; y < MAZE_SIZE; y++) {
				cell_t c = { (int8_t)x, (int8_t)y };
				int dx2 = 2 * x - (MAZE_SIZE - 1), dy2 = 2 * y - (MAZE_SIZE - 1);   /* 中心からの距離の 2 倍 */
				if ((dx2 < 0 ? -dx2 : dx2) + (dy2 < 0 ? -dy2 : dy2) > 2 * band) {
					continue;
				}
				if (maze_visited(c) || maze_is_opponent(c) || maze_dist(c) == MAZE_DIST_INF) {
					continue;
				}
				if (count < max) {
					out[count] = c;
				}
				count++;
			}
		}
	}
	return count;
}
