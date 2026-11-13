/*
 * maze.h
 *
 * 迷路の地図と、行き先までの道順を決める（Issue #9）。
 * ハードウェアに触らないので、PC 上でテストできる（test/test_maze.c）。
 *
 * 座標
 *   区画 (x, y)。x, y は 0〜MAZE_SIZE-1。
 *   向き: 東 (+x) = 0、北 (+y) = 1、西 = 2、南 = 3。
 *   左回りで 1 増える（ctl_motion.h と同じく左回りが +）。
 *
 * 壁
 *   区画の 4 つの辺ごとに「壁がある / ない」と「分かっているか」を持つ。
 *   外周の壁は最初から分かっている。
 *   分かっていない壁は、道順を決めるときは「ない」とみなす（足立法）。
 *
 * 相手陣地
 *   フィールドは中心について点対称（規程 2-4）。区画 (x, y) の反対側は (15-x, 15-y)。
 *   自分が通った区画の反対側は、必ず相手陣地にある。
 *   投げる場所は、この「相手陣地と分かっている区画」との間の壁だけから選ぶ
 *   （外周の壁や、どちらの陣地か分からない壁に向けて投げると、外へ出て負けになることがある。規程 4-12）。
 */
#ifndef INC_MAZE_H_
#define INC_MAZE_H_

#include <stdint.h>

#define MAZE_SIZE       16
#define MAZE_DIST_INF   0xFFFF

typedef enum {
	DIR_E = 0,
	DIR_N = 1,
	DIR_W = 2,
	DIR_S = 3,
} dir_t;

typedef struct {
	int8_t x;
	int8_t y;
} cell_t;

/* 投げる場所: 区画 from で向き dir を向いて投げる */
typedef struct {
	cell_t from;
	dir_t dir;
} throw_spot_t;

#define DIR_LEFT(d)     ((dir_t)(((d) + 1) & 3))
#define DIR_RIGHT(d)    ((dir_t)(((d) + 3) & 3))
#define DIR_BACK(d)     ((dir_t)(((d) + 2) & 3))

/* 地図を消し、外周の壁だけを入れる */
void maze_init(void);

int maze_in_field(int x, int y);
cell_t maze_neighbor(cell_t c, dir_t d);

/* 壁を書き込む（隣の区画の側にも書く） */
void maze_set_wall(cell_t c, dir_t d, int present);
int maze_wall(cell_t c, dir_t d);         /* 壁がある（分かっている）なら 1 */
int maze_known(cell_t c, dir_t d);        /* 分かっていれば 1 */
int maze_open(cell_t c, dir_t d);         /* 壁がないと分かっていれば 1 */

/*
 * 区画 c の中央で heading を向いているときに見た壁を書き込み、c を「通った」にする。
 * left / front / right は 1 = 壁あり、0 = 壁なし。
 */
void maze_update(cell_t c, dir_t heading, int left, int front, int right);
void maze_mark_visited(cell_t c);
int maze_visited(cell_t c);

/* 区画 c が相手陣地と分かっていれば 1（反対側の区画を通ったことがある） */
int maze_is_opponent(cell_t c);

/*
 * 行き先（goals の n 個のどれか）までの歩数を、すべての区画について求める。
 * 分かっていない壁は「ない」とみなす。相手陣地と分かっている区画は通らない。
 */
void maze_flood(const cell_t *goals, int n);
uint16_t maze_dist(cell_t c);

/*
 * 区画 c で heading を向いているとき、次に進む向きを返す。
 * 歩数が 1 減る隣の区画のうち、まっすぐ → 左 → 右 → 後ろの順に選ぶ。
 * 着いている（歩数 0）か、行けないときは -1。
 */
int maze_next_dir(cell_t c, dir_t heading);

/*
 * 投げる場所を探す。条件（すべて分かっていること）:
 *   - 区画 a の dir の側に壁があり、その向こうの区画が相手陣地
 *   - from は a の 1 つ手前（dir と反対側の隣）で、from と a の間に壁がない
 *   - from は通ったことがある区画
 * 見つかった数を返す（最大 max 個を out に入れる）。
 */
int maze_find_throw_spots(throw_spot_t *out, int max);

/*
 * 投げる場所がまだ見つからないときに、次に見に行く区画を探す。
 * 相手陣地と分かっている区画の隣で、その間の壁がまだ分からない区画と、
 * 投げる場所の候補の 1 つ手前で、まだ通っていない区画。
 * どちらもなければ、まだ通っていない区画（相手陣地と分かっている区画を除く）。
 * 今いる区画 here から行けない区画（分かっている壁で閉じている）は選ばない。
 * 見つかった数を返す（最大 max 個を out に入れる）。maze_flood() の結果は上書きされる。
 */
int maze_throw_search_goals(cell_t here, cell_t *out, int max);

#endif /* INC_MAZE_H_ */
