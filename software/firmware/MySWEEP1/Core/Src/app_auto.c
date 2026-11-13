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

/* 何をしているか */
typedef enum {
	ST_WAIT_GATE,
	ST_PLAN,        /* 次にすることを決める */
	ST_TURN,        /* その場で回っている */
	ST_MOVE,        /* 1 区画進んでいる */
	ST_SETTLE,      /* 止まってからセンサの値が落ち着くのを待つ */
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
		ball_done[i] = 0;
	}
	goal = GOAL_BALL;
	gate_seen = 0;
	t0 = time_ms();
	state = ST_WAIT_GATE;
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

static void sense(void) {
	maze_update(pos, heading, wall_left(), wall_front(), wall_right());
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
	case ST_MOVE:
		motion_straight(CELL_MM, AUTO_SPEED, 0, AUTO_ACCEL, 1);
		break;
	case ST_APPROACH:
		motion_straight(AUTO_APPROACH_MM, AUTO_SPEED, 0, AUTO_ACCEL, 0);
		break;
	case ST_THROW:
		arm_throw();
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
static void next_ball(void) {
	goal = GOAL_BALL;
	for (int i = 0; i < AUTO_BALL_COUNT; i++) {
		if (ball_done[i]) {
			continue;
		}
		cell_t b = ball_cell(i);
		maze_flood(&b, 1);
		if (maze_dist(pos) != MAZE_DIST_INF) {
			ball_index = i;
			state = ST_PLAN;
			return;
		}
	}
	for (int i = 0; i < AUTO_BALL_COUNT; i++) {
		if (!ball_done[i]) {
			state = ST_STUCK;   /* 残っているボールに行けない */
			return;
		}
	}
	state = ST_DONE;
}

/* 1 区画進むか、目的の動作を始める */
static void plan(void) {
	if (goal == GOAL_BALL) {
		cell_t b = ball_cell(ball_index);
		maze_flood(&b, 1);
		int d = maze_next_dir(pos, heading);
		if (d < 0) {
			next_ball();   /* このボールには今は行けない。行けるボールに変える（なければ止まる） */
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
	face((dir_t)d, ST_MOVE);
}

void auto_loop(void) {
	uint32_t now = time_ms();

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
		} else if (now - t0 >= AUTO_GATE_OPEN_MS) {
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
			t0 = now;
			state = ST_SETTLE;
		}
		break;

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
