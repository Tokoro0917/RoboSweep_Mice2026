/*
 * ctl_wall.h
 *
 * 横の壁を見て、通路の真ん中をまっすぐ進むための補正（PD 制御）。
 * 出力 c を、右車輪に +c、左車輪に -c 足す。
 * c > 0 で左へ寄る（右の壁から離れる）。
 */
#ifndef INC_CTL_WALL_H_
#define INC_CTL_WALL_H_

void wall_control_reset(void);

/* 1ms ごとに呼ぶ。speed は今の前向きの速さ [mm/s] */
float wall_control_update(float speed);

/* 区画の判定用 */
int wall_left(void);
int wall_right(void);
int wall_front(void);

#endif /* INC_CTL_WALL_H_ */
