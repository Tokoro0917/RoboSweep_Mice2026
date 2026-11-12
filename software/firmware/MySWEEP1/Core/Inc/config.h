/*
 * config.h
 *
 * 機体のパラメータをまとめたファイル。
 * 調整するときは、基本的にこのファイルだけを書き換える。
 *
 * 単位: 距離 mm / 速度 mm/s / 加速度 mm/s^2 / 角度 deg / 角速度 deg/s / 時間 ms
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

/* ------------------------------------------------------------------ */
/* 制御周期                                                            */
/* ------------------------------------------------------------------ */
/* TIM6 の割り込み周期。tim.c / MySWEEP1.ioc で 1kHz に設定している。 */
#define CONTROL_HZ              1000
#define CONTROL_DT              (1.0f / CONTROL_HZ)

/* ------------------------------------------------------------------ */
/* 機体の寸法                                                          */
/* ------------------------------------------------------------------ */
#define TIRE_DIAMETER_MM        53.0f   /* タイヤ直径 */
#define TREAD_MM                85.0f   /* 左右のタイヤの間隔 */
#define CELL_MM                 180.0f  /* 迷路 1 区画の長さ */

/* ------------------------------------------------------------------ */
/* ステッピングモーター（PKE243A-L + SLA7073MPRT）                       */
/* ------------------------------------------------------------------ */
/* 1 パルスで回る角度。PKE243A は 1.8deg/step で、元のプログラムは
 * 0.9deg（ハーフステップ）として計算していた。SLA7073 の分割設定を
 * 変えたらここも変える。 */
#define STEP_ANGLE_DEG          0.9f

/* パルスを出すタイマーのクロック。
 * TIM3: APB1 タイマークロック 90MHz / プリスケーラ 8   = 11.25MHz
 * TIM8: APB2 タイマークロック 180MHz / プリスケーラ 16 = 11.25MHz
 * 元のプログラムは 10MHz として計算していたため、実際の速度が
 * 指定より 1.125 倍速かった。 */
#define STEP_TIMER_HZ           11250000.0f

/* 16bit タイマーで出せるいちばん遅いパルス（ARR = 65535）のときの速度は
 *   11.25MHz x 0.416mm / 65536 = 約 71.5 mm/s
 * これより遅い速度は出せないので、パルスを止める。
 * MOTION_START_SPEED と、MOTION_TURN_START_SPEED を車輪の速さに直した値
 * （100deg/s → 74mm/s）は、これより大きくしておくこと。 */
#define WHEEL_MIN_SPEED         72.0f   /* mm/s */

/*
 * 左右について（要確認）
 * ------------------------------------------------------------
 * 元のプログラムで「左」と呼んでいた車輪は TIM3_CH3（PC8）のパルスと
 * PB15 の回転方向ピンで動かしていた。ところが .ioc のピン名では
 * PC8 = MOTOR_CLOCK_R（右）、PB15 = MOTOR_CW/CCW_L（左）になっていて、
 * 名前が食い違っている。
 * このプログラムでは、動作が確認できている元のプログラムと同じピンの
 * 組み合わせ・同じ左右の呼び方を使っている。実機で確かめたら、
 * motor.c の表と、この説明を直すこと。
 */

/* ------------------------------------------------------------------ */
/* 壁センサ                                                            */
/* ------------------------------------------------------------------ */
/* センサの並び（ADC の順番）。値は「LED 点灯時 - 消灯時」で、壁に近いほど大きい。 */
#define WALL_SENSOR_FRONT_A     0       /* SENSOR_1 (PC3) 元: nowf1 */
#define WALL_SENSOR_LEFT        1       /* SENSOR_2 (PC2) 元: nowl  */
#define WALL_SENSOR_RIGHT       2       /* SENSOR_3 (PC1) 元: nowr  */
#define WALL_SENSOR_FRONT_B     3       /* SENSOR_4 (PC0) 元: nowf2 */

/* 区画の中央にいるときの横センサの値 */
#define WALL_CENTER_LEFT        2500
#define WALL_CENTER_RIGHT       2500
/* これより小さいと、その横には壁がないとみなす */
#define WALL_EXIST_LEFT         1500
#define WALL_EXIST_RIGHT        1500
/* 1 周期でこれ以上増えたら、壁の切れ目などのノイズとみなして使わない */
#define WALL_JUMP_LEFT          100
#define WALL_JUMP_RIGHT         100
/* 前センサがこれより大きいときは横の壁制御をやめる（前壁に近すぎる） */
#define WALL_FRONT_NEAR         2500
/* 前壁があるとみなす値（区画の判定用。要調整） */
#define WALL_EXIST_FRONT        450

/* 横の壁制御のゲイン（元のプログラムと同じ値） */
#define WALL_CONTROL_GAIN       0.02f
#define WALL_CONTROL_REF_SPEED  300.0f  /* この速度でゲインが 1 倍になる */

/* 電池電圧の分圧比（R4=100k, R5=22k と想定。要確認） */
#define BATTERY_DIVIDER         ((100.0f + 22.0f) / 22.0f)

/* ------------------------------------------------------------------ */
/* 走行                                                                */
/* ------------------------------------------------------------------ */
/* 止まった状態から動き出すときの速度（ステッピングモーターは 0 からゆっくり
 * 加速しなくても、この速さまでは脱調せずに動き出せる） */
#define MOTION_START_SPEED      100.0f  /* mm/s */
#define MOTION_TURN_START_SPEED 100.0f  /* deg/s */

/* ------------------------------------------------------------------ */
/* リモコン操作の速度（元のプログラムと同じ値）                           */
/* ------------------------------------------------------------------ */
#define REMOTE_STRAIGHT_SPEED   300.0f  /* mm/s */
#define REMOTE_STRAIGHT_ACCEL   1000.0f /* mm/s^2 */
#define REMOTE_TURN_SPEED       200.0f  /* deg/s */
#define REMOTE_TURN_ACCEL       1000.0f /* deg/s^2 */

/* リモコンの信号がこの時間届かなかったら、ボタンを離したとみなして止まる。
 * 0 にするとこの機能を切る。 */
#define REMOTE_TIMEOUT_MS       500

/* ------------------------------------------------------------------ */
/* アーム（RS204MD x2、角度は 0.1deg 単位、時間は 10ms 単位）            */
/* ------------------------------------------------------------------ */
#define SERVO_ID_LOWER          1       /* 根元側 */
#define SERVO_ID_UPPER          2       /* 先端側 */

/* つかむ */
#define ARM_CATCH_UPPER         500
#define ARM_CATCH_LOWER         1500
#define ARM_CATCH_TIME          50
#define ARM_CATCH_WAIT_MS       1000

/* ホームポジション */
#define ARM_HOME_LOWER_FIRST    450
#define ARM_HOME_FIRST_WAIT_MS  500
#define ARM_HOME_UPPER          0
#define ARM_HOME_LOWER          0
#define ARM_HOME_TIME           150

/* 投げる（構える → 振る） */
#define ARM_THROW_READY_UPPER   1000
#define ARM_THROW_READY_LOWER   450
#define ARM_THROW_READY_TIME    75
#define ARM_THROW_READY_WAIT_MS 1000
#define ARM_THROW_UPPER         800
#define ARM_THROW_UPPER_WAIT_MS 50
#define ARM_THROW_LOWER         1200
#define ARM_THROW_TIME          10

/* 吸引ファン（TIM4_CH4 = PB9）。比較値が小さいほど ON（元のプログラムと同じ）。 */
#define FAN_PWM_PERIOD          100
#define FAN_PWM_ON              2
#define FAN_PWM_OFF             100

#endif /* INC_CONFIG_H_ */
