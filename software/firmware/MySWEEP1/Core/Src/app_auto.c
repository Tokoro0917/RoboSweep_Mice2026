/*
 * app_auto.c
 *
 * 自律走行（説明は app_auto.h）。
 * 待つ処理（motion_wait など）は使わず、auto_loop() を呼ぶたびに少しずつ進める。
 * こうすると、待っている間も arm_update() やログが止まらない。
 */
#include "app_auto.h"

#include "config.h"
#include "drv_time.h"
#include "ctl_motion.h"
#include "ctl_wall.h"
#include "app_arm.h"
#include "drv_link.h"

/* 何をしているか */
typedef enum {
	ST_WAIT_GATE,
	ST_PLAN,        /* 次にすることを決める */
	ST_TURN,        /* その場で回っている */
	ST_MOVE,        /* 1 区画進んでいる */
	ST_SETTLE,      /* 止まってからセンサの値が落ち着くのを待つ */
	ST_WAIT_PEER,   /* 次の区画にもう 1 台がいるので待っている */
	ST_APPROACH,    /* ボールの方へ少し進んでいる */
	ST_CATCH,       /* つかんでいる */
	ST_LIFT,        /* 持ち上げている */
	ST_BACK,        /* 区画の中央へ戻っている */
	ST_THROW,       /* 投げている */
	ST_HOME,        /* アームを戻している */
	ST_DONE,
	ST_STUCK,
} state_t;

/* 何を目指しているか */
typedef enum {
	GOAL_BALL,      /* ボールの区画の隣へ */
	GOAL_THROW,     /* 投げる場所へ */
} goal_t;

#define SETTLE_MS 30

static const int8_t BALLS[AUTO_BALL_COUNT][2] = AUTO_BALLS;
static const uint8_t BALL_OWNER[AUTO_BALL_COUNT] = AUTO_BALL_OWNER;

static state_t state;
static goal_t goal;
static int ball_index;
static uint8_t ball_done[AUTO_BALL_COUNT];
static int throws;
static cell_t pos;
static dir_t heading;
static dir_t turn_target;     /* ST_TURN のあと向く向き */
static state_t after_turn;    /* ST_TURN のあとにすること */
static uint32_t t0;
static cell_t next_cell;      /* 次に入る区画（もう 1 台に伝える） */
static cell_t temp_avoid;     /* もう 1 台をよけるために、しばらく通らない区画 */
static int temp_avoid_on;
static uint32_t retry_at;     /* ST_WAIT_PEER で、次に回り道を試してよい時刻 */
static uint32_t last_send;
static link_peer_t last_sent;
static int gate_seen;

static cell_t ball_cell(int i) {
	cell_t c = { BALLS[i][0], BALLS[i][1] };
	return c;
}

void auto_init(void) {
	maze_init();
	pos.x = AUTO_START_X;
	pos.y = AUTO_START_Y;
	heading = (dir_t)(AUTO_START_DIR & 3);
	ball_index = 0;
	throws = 0;
	for (int i = 0; i < AUTO_BALL_COUNT; i++) {
		/* 2 台のときは、もう 1 台が取るボールは最初から「済み」にしておく */
		ball_done[i] = (AUTO_ROBOT_COUNT >= 2 && BALL_OWNER[i] != AUTO_ROBOT_ID);
	}
	next_cell = pos;
	temp_avoid_on = 0;
	retry_at = 0;
	last_send = 0;
	goal = GOAL_BALL;
	gate_seen = 0;
	t0 = time_ms();
	state = ST_WAIT_GATE;
	arm_home();   /* 電源を入れたあと、サーボのトルクを入れてアームをホームポジションにする */
	for (int i = 0; i < AUTO_BALL_COUNT; i++) {
		maze_set_avoid(ball_cell(i), 1);   /* ボールを押さないように、ボールの区画は通らない */
	}
}

auto_status_t auto_status(void) {
	switch (state) {
	case ST_WAIT_GATE: return AUTO_WAIT_GATE;
	case ST_DONE:      return AUTO_DONE;
	case ST_STUCK:     return AUTO_STUCK;
	default:           return AUTO_RUNNING;
	}
}

cell_t auto_cell(void) { return pos; }
dir_t auto_heading(void) { return heading; }
int auto_throw_count(void) { return throws; }
int auto_ball_index(void) { return ball_index; }

static int same(cell_t a, cell_t b) {
	return a.x == b.x && a.y == b.y;
}

/* もう 1 台が今いる区画を返す（いなければ 0） */
static int peer_cell(cell_t *c, cell_t *next, uint8_t *id) {
	link_peer_t p;
	if (AUTO_ROBOT_COUNT < 2 || !link_peer(&p) || p.id == AUTO_ROBOT_ID) {
		return 0;
	}
	c->x = p.x;
	c->y = p.y;
	next->x = p.nx;
	next->y = p.ny;
	*id = p.id;
	return 1;
}

/*
 * 区画 n に入ってはいけないか。
 *   もう 1 台が n にいる、または n に入ろうとしていて、その機体の番号が小さい（番号が小さいほうが先）
 */
static int peer_blocks(cell_t n) {
	cell_t pc, pn;
	uint8_t id;
	if (!peer_cell(&pc, &pn, &id)) {
		return 0;
	}
	return same(pc, n) || (same(pn, n) && (int)id + 1 <= AUTO_ROBOT_ID);
}

/* 正面同士でにらみ合っている（相手が自分の区画に入ろうとし、自分は相手の区画に入ろうとしている） */
static int peer_head_on(cell_t n) {
	cell_t pc, pn;
	uint8_t id;
	return peer_cell(&pc, &pn, &id) && same(pc, n) && same(pn, pos);
}

/* 壁を見る。もう 1 台がいる隣の区画の側は、機体を壁と見間違えるので書き込まない */
static void sense(void) {
	cell_t pc, pn;
	uint8_t id;
	int have_peer = peer_cell(&pc, &pn, &id);
	const dir_t sides[3] = { DIR_LEFT(heading), heading, DIR_RIGHT(heading) };
	const int walls[3] = { wall_left(), wall_front(), wall_right() };
	for (int i = 0; i < 3; i++) {
		if (have_peer && same(maze_neighbor(pos, sides[i]), pc) && walls[i]) {
			continue;   /* 壁かもしれないし、もう 1 台かもしれない。分からないままにする */
		}
		maze_set_wall(pos, sides[i], walls[i]);
	}
	maze_mark_visited(pos);
}

/* 自分の位置を送る（変わったとき、または AUTO_SEND_INTERVAL_MS ごと） */
static void send_self(uint32_t now) {
	if (AUTO_ROBOT_COUNT < 2) {
		return;
	}
	link_peer_t me = { AUTO_ROBOT_ID, pos.x, pos.y, next_cell.x, next_cell.y, (uint8_t)auto_status() };
	int changed = me.x != last_sent.x || me.y != last_sent.y || me.nx != last_sent.nx
			|| me.ny != last_sent.ny || me.status != last_sent.status;
	if ((changed || now - last_send >= AUTO_SEND_INTERVAL_MS) && link_send_self(&me)) {
		last_sent = me;
		last_send = now;
	}
}

static void start_state(state_t s);

/* 向き d を向いてから next を始める。もう向いていればすぐ始める */
static void face(dir_t d, state_t next) {
	int diff = (d - heading) & 3;
	turn_target = d;
	after_turn = next;
	if (diff == 0) {
		start_state(next);
		return;
	}
	float angle = diff == 1 ? 90.0f : (diff == 3 ? -90.0f : 180.0f);
	motion_turn(angle, AUTO_TURN_SPEED, AUTO_TURN_ACCEL);
	state = ST_TURN;
}

static void start_state(state_t s) {
	switch (s) {
	case ST_MOVE: {
		cell_t n = maze_neighbor(pos, heading);
		if (peer_blocks(n)) {
			t0 = time_ms();   /* もう 1 台がいる。空くまで待つ */
			state = ST_WAIT_PEER;
			return;
		}
		next_cell = n;
		motion_straight(CELL_MM, AUTO_SPEED, 0, AUTO_ACCEL, 1);
		break;
	}
	case ST_APPROACH:
		motion_straight(AUTO_APPROACH_MM, AUTO_SPEED, 0, AUTO_ACCEL, 0);
		break;
	case ST_THROW:
		arm_throw();
		break;
	case ST_SETTLE:
		t0 = time_ms();
		break;
	default:
		break;
	}
	state = s;
}

/*
 * 次にねらうボールを選ぶ。まだ投げていないボールのうち、AUTO_BALLS の順で最初に行けそうなもの。
 * ほかのボールの区画を通らないと行けないボールは後回しになる（そのボールを取ると通れるようになる）。
 */
static int pick_ball(void) {
	for (int i = 0; i < AUTO_BALL_COUNT; i++) {
		if (ball_done[i]) {
			continue;
		}
		cell_t b = ball_cell(i);
		maze_flood(&b, 1);
		if (maze_dist(pos) != MAZE_DIST_INF) {
			ball_index = i;
			return 1;
		}
	}
	return 0;
}

static void next_ball(void) {
	goal = GOAL_BALL;
	int left = 0;
	for (int i = 0; i < AUTO_BALL_COUNT; i++) {
		left += !ball_done[i];
	}
	if (left == 0) {
		state = ST_DONE;
		return;
	}
	if (pick_ball()) {
		state = ST_PLAN;
		return;
	}
	/*
	 * どのボールにも行けない。ほかのボールの区画を通らないと行けないのかもしれないので、
	 * ボールを押してもよいことにして、もう一度選ぶ。それでもだめなら止まる。
	 */
	for (int i = 0; i < AUTO_BALL_COUNT; i++) {
		maze_set_avoid(ball_cell(i), 0);
	}
	if (pick_ball()) {
		maze_set_avoid(ball_cell(ball_index), 0);
		state = ST_PLAN;
		return;
	}
	state = ST_STUCK;   /* 残っているボールに行けない */
}

static void plan_goal(void);

/* もう 1 台に道をゆずるため、来た道などへ 1 区画下がる。下がれる所がなければ 0 */
static int retreat(void) {
	cell_t pc, pn;
	uint8_t id;
	int have_peer = peer_cell(&pc, &pn, &id);
	const dir_t order[4] = { DIR_BACK(heading), DIR_LEFT(heading), DIR_RIGHT(heading), heading };
	for (int i = 0; i < 4; i++) {
		cell_t c = maze_neighbor(pos, order[i]);
		if (!maze_open(pos, order[i]) || maze_avoided(c) || maze_is_opponent(c)) {
			continue;
		}
		if (have_peer && (same(c, pc) || same(c, pn))) {
			continue;
		}
		face(order[i], ST_MOVE);
		return 1;
	}
	return 0;
}

static void plan(void) {
	plan_goal();
	if (temp_avoid_on && state == ST_STUCK) {
		/* もう 1 台をよけると行ける所がない。よけるのをやめる */
		maze_set_avoid(temp_avoid, 0);
		temp_avoid_on = 0;
		cell_t n = maze_neighbor(pos, heading);
		if (peer_head_on(n) && AUTO_ROBOT_ID > 0 && retreat()) {
			return;   /* 番号が大きいほうが下がって道をゆずる */
		}
		state = ST_PLAN;
		plan_goal();   /* 同じ向きに行こうとして、また待つ */
	}
}

static void plan_goal(void) {
	if (goal == GOAL_BALL) {
		cell_t b = ball_cell(ball_index);
		maze_flood(&b, 1);
		int d = maze_next_dir(pos, heading);
		if (d < 0) {
			next_ball();   /* このボールには今は行けない。行けるボールに変える（なければ止まる） */
			return;
		}
		if (!maze_known(pos, (dir_t)d)) {
			face((dir_t)d, ST_SETTLE);   /* まだ見ていない側（スタート区画の後ろなど）。向いて見てから決め直す */
			return;
		}
		cell_t n = maze_neighbor(pos, (dir_t)d);
		if (n.x == b.x && n.y == b.y) {
			face((dir_t)d, ST_APPROACH);   /* 隣に着いた。ボールの方を向いてつかみに行く */
		} else {
			face((dir_t)d, ST_MOVE);
		}
		return;
	}

	/* GOAL_THROW */
	static throw_spot_t spots[16];
	static cell_t goals[MAZE_SIZE * MAZE_SIZE];
	int ns = maze_find_throw_spots(spots, 16);
	if (ns > 16) {
		ns = 16;
	}
	for (int i = 0; i < ns; i++) {
		if (spots[i].from.x == pos.x && spots[i].from.y == pos.y) {
			face(spots[i].dir, ST_THROW);   /* 投げる場所に着いた */
			return;
		}
	}
	int ng;
	if (ns > 0) {
		for (int i = 0; i < ns; i++) {
			goals[i] = spots[i].from;
		}
		ng = ns;
	} else {
		ng = maze_throw_search_goals(pos, goals, MAZE_SIZE * MAZE_SIZE);
		if (ng > MAZE_SIZE * MAZE_SIZE) {
			ng = MAZE_SIZE * MAZE_SIZE;
		}
	}
	int d = -1;
	if (ng > 0) {
		maze_flood(goals, ng);
		d = maze_next_dir(pos, heading);
	}
	if (d < 0) {
		/*
		 * 投げる場所に行けない。ほかのボールの区画を通らないと行けないのかもしれないので、
		 * ボールを押してもよいことにして、もう一度探す。それでもだめなら止まる。
		 */
		int cleared = 0;
		for (int i = 0; i < AUTO_BALL_COUNT; i++) {
			if (maze_avoided(ball_cell(i))) {
				maze_set_avoid(ball_cell(i), 0);
				cleared = 1;
			}
		}
		if (!cleared) {
			state = ST_STUCK;
		}
		return;   /* cleared なら、次の auto_loop() でもう一度 plan() する */
	}
	if (!maze_known(pos, (dir_t)d)) {
		face((dir_t)d, ST_SETTLE);
		return;
	}
	face((dir_t)d, ST_MOVE);
}

void auto_loop(void) {
	uint32_t now = time_ms();
	send_self(now);

	switch (state) {
	case ST_WAIT_GATE:
		/* ゲートが見えている状態が続いたあと、見えなくなって AUTO_GATE_OPEN_MS たったら走り出す */
		if (wall_front()) {
			if (!gate_seen && now - t0 >= AUTO_GATE_SEEN_MS) {
				gate_seen = 1;
			}
			if (!gate_seen) {
				break;
			}
			t0 = now;
		} else if (!gate_seen) {
			t0 = now;   /* まだゲートを見ていない。見えるまで待つ */
		} else if (now - t0 >= AUTO_GATE_OPEN_MS && !arm_is_busy()) {
			sense();
			next_ball();
		}
		break;

	case ST_PLAN:
		plan();
		break;

	case ST_TURN:
		if (!motion_is_busy()) {
			heading = turn_target;
			start_state(after_turn);
		}
		break;

	case ST_MOVE:
		if (!motion_is_busy()) {
			pos = maze_neighbor(pos, heading);
			next_cell = pos;
			if (temp_avoid_on) {
				maze_set_avoid(temp_avoid, 0);   /* 回り道を 1 区画進んだので、よけるのをやめる */
				temp_avoid_on = 0;
			}
			t0 = now;
			state = ST_SETTLE;
		}
		break;

	case ST_WAIT_PEER: {
		cell_t n = maze_neighbor(pos, heading);
		if (!peer_blocks(n)) {
			start_state(ST_MOVE);   /* 空いた */
			break;
		}
		/*
		 * 正面同士でにらみ合っていて自分の番号が大きいとき、または長く待っても空かないときは、
		 * その区画をしばらく通らないことにして、回り道を探す（plan() の中）。
		 */
		int yield = peer_head_on(n) ? (AUTO_ROBOT_ID > 0) : (now - t0 >= AUTO_PEER_WAIT_MS);
		if (yield && (int32_t)(now - retry_at) >= 0 && !maze_avoided(n)) {
			retry_at = now + AUTO_PEER_WAIT_MS;
			maze_set_avoid(n, 1);
			temp_avoid = n;
			temp_avoid_on = 1;
			state = ST_PLAN;
		}
		break;
	}

	case ST_SETTLE:
		if (now - t0 >= SETTLE_MS) {
			sense();
			state = ST_PLAN;
		}
		break;

	case ST_APPROACH:
		if (!motion_is_busy() && arm_catch()) {
			state = ST_CATCH;
		}
		break;

	case ST_CATCH:
		if (!arm_is_busy() && arm_carry()) {
			state = ST_LIFT;
		}
		break;

	case ST_LIFT:
		if (!arm_is_busy()) {
			motion_straight(-AUTO_APPROACH_MM, AUTO_SPEED, 0, AUTO_ACCEL, 0);
			state = ST_BACK;
		}
		break;

	case ST_BACK:
		if (!motion_is_busy()) {
			maze_set_avoid(ball_cell(ball_index), 0);   /* もうボールはない */
			goal = GOAL_THROW;
			state = ST_PLAN;
		}
		break;

	case ST_THROW:
		if (!arm_is_busy() && arm_home()) {
			throws++;
			state = ST_HOME;
		}
		break;

	case ST_HOME:
		if (!arm_is_busy()) {
			ball_done[ball_index] = 1;
			next_ball();
		}
		break;

	case ST_DONE:
	case ST_STUCK:
		break;
	}
}
