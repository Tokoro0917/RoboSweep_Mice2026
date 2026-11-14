/*
 * app_arm.h
 *
 * アーム（サーボ 2 個）と吸引ファンの動作。
 * 動作を始める関数はすぐに戻り、arm_update() を main ループで呼び続けると
 * 決められた順番で進む。動いている間も走行などの処理は止まらない。
 */
#ifndef INC_APP_ARM_H_
#define INC_APP_ARM_H_

void arm_init(void);

/* main ループから呼び続ける */
void arm_update(void);

/* 動作を始める（動作中なら何もしないで 0 を返す） */
int arm_catch(void);   /* 下ろしてつかむ → ファン ON */
int arm_home(void);    /* ホームポジション → ファン OFF */
int arm_throw(void);   /* 投げる */
int arm_carry(void);   /* つかんだまま「構える」姿勢へ持ち上げる（ファンは ON のまま） */

int arm_is_busy(void);

#endif /* INC_APP_ARM_H_ */
