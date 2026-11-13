/*
 * app_auto.h
 *
 * 自律走行（config.h の ROBOT_MODE が ROBOT_MODE_AUTO のとき）。
 *
 * 流れ
 *   1. スタート区画でゲートの方を向いて置く。ゲートが開いたら走り出す。
 *   2. 1 つ目のボールの区画の隣まで、区画ごとに止まって壁を見ながら進む（足立法）。
 *   3. ボールの方を向いて少し進み、つかむ → 持ち上げる → 区画の中央へ戻る。
 *   4. 投げる場所（相手陣地との壁の 1 マス手前）へ行き、壁の方を向いて投げる。
 *      投げる場所がまだ分からなければ、探しに行く（maze.h）。
 *   5. 次のボールへ。全部終わったら止まる。
 *
 * 壁の判定（ctl_wall.c）と車輪の向き（config.h）は、実機で確かめてから使うこと。
 */
#ifndef INC_APP_AUTO_H_
#define INC_APP_AUTO_H_

#include <stdint.h>
#include "maze.h"

typedef enum {
	AUTO_WAIT_GATE = 0,   /* ゲートが開くのを待っている */
	AUTO_RUNNING,         /* 動いている */
	AUTO_DONE,            /* 全部終わった */
	AUTO_STUCK,           /* 行ける所がなくて止まった */
} auto_status_t;

void auto_init(void);

/* main ループから呼び続ける（arm_update() も別に呼ぶこと） */
void auto_loop(void);

auto_status_t auto_status(void);

/* 今いる区画と向き（ログ・テスト用） */
cell_t auto_cell(void);
dir_t auto_heading(void);

/* 投げた回数 */
int auto_throw_count(void);

/* 今ねらっているボールの番号（AUTO_BALLS の順。ログ用） */
int auto_ball_index(void);

#endif /* INC_APP_AUTO_H_ */
