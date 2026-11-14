/* テスト用: でたらめな点対称の迷路（test_maze.c と test_auto.c で使う） */
#pragma once
#include <stdint.h>
#include "maze.h"

extern uint8_t truth[MAZE_SIZE][MAZE_SIZE];   /* 壁があるビット（向きの番号） */
extern uint8_t ours[MAZE_SIZE][MAZE_SIZE];    /* 自分の陣地なら 1 */
extern const int DX[4], DY[4];

int in(int x, int y);
int twall(int x, int y, int d);
/* 行 y では x < s[y] が自分の陣地。s は min_split〜11（下半分の行） */
void make_maze(unsigned seed, int min_split);
/* 壁を 1 つ抜く（点対称の反対側も） */
void sim_open(int x, int y, int d);
