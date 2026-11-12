/*
 * ctl_motion.h
 *
 * 機体の動きを作る層。1ms ごとに motion_tick() で速度を少しずつ変え、
 * 左右の車輪の速さにしてモーターに渡す。
 *
 * 向き: 前が +、左回り（反時計回り）が +。
 *
 * 使い方（自律走行）
 *   motion_straight(CELL_MM, 500, 0, 2000, 1);   // 1 区画進んで止まる
 *   motion_wait();
 *   motion_turn(90, 360, 2000);                   // 左に 90 度
 *   motion_wait();
 */
#ifndef INC_CTL_MOTION_H_
#define INC_CTL_MOTION_H_

typedef struct {
	float x;       /* [mm] 起動したときの向きが +x */
	float y;       /* [mm] 起動したときの左が +y */
	float theta;   /* [deg] 左回りが + */
} pose_t;

void motion_init(void);

/* 1ms ごとに制御割り込みから呼ぶ */
void motion_tick(void);

/*
 * まっすぐ distance [mm] 進む（負なら後ろへ）。台形加速。
 *   end_speed が 0 なら止まる。0 でなければ、その速さで進み続ける（次の区画へつなぐとき）
 *   wall_control が 1 なら横の壁で補正する（前進のときだけ）
 */
void motion_straight(float distance, float max_speed, float end_speed, float accel,
		int wall_control);

/* その場で angle [deg] 回る（左回りが +） */
void motion_turn(float angle, float max_speed, float accel);

/* 指定の速さに向けて加減速し、そのまま動き続ける（リモコン用） */
void motion_drive(float speed, float accel, float turn_speed, float turn_accel,
		int wall_control);

/* すぐに止まる（減速しない） */
void motion_stop(void);

/* motion_straight / motion_turn の途中なら 1 */
int motion_is_busy(void);
void motion_wait(void);

/* 速さが 0 なら 1 */
int motion_is_stopped(void);

/* 今の速さ */
float motion_speed(void);
float motion_turn_speed(void);

/* パルス数から求めた位置。motion_reset_pose() で 0 に戻す */
pose_t motion_pose(void);
void motion_reset_pose(void);

/* 起動してから進んだ距離 [mm]（左右の平均）と回った角度 [deg] */
float motion_distance(void);
float motion_angle(void);

#endif /* INC_CTL_MOTION_H_ */
