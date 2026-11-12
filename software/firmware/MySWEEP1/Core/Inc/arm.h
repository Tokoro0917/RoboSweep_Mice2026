/*
 * arm.h
 *
 * ボールをつかむアーム（サーボ 2 個）と吸引ファン。
 * 角度や待ち時間は config.h の ARM_* で変える。
 * どの関数も、動き終わるまで待ってから戻る。
 */

#ifndef INC_ARM_H_
#define INC_ARM_H_

void arm_init(void);

/* アームを下ろしてボールをつかむ（最後に吸引ファンを ON） */
void arm_catch(void);

/* ホームポジションに戻す（吸引ファンを OFF） */
void arm_home(void);

/* ボールを投げる */
void arm_throw(void);

void fan_set(int on);
int fan_is_on(void);

#endif /* INC_ARM_H_ */
