/*
 * config.h
 *
 * 機体のパラメータ。調整するときは、基本的にこのファイルだけを書き換える。
 *
 * 単位: 距離 mm / 速度 mm/s / 加速度 mm/s^2 / 角度 deg / 角速度 deg/s / 時間 ms
 * 向き: 前が +、左回りが +
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

/* ================================================================== */
/* 制御周期                                                            */
/* ================================================================== */
/* TIM6 の割り込み周期（tim.c / MySWEEP1.ioc で 1kHz に設定） */
#define CONTROL_HZ              1000
#define CONTROL_DT              (1.0f / CONTROL_HZ)

/* ================================================================== */
/* 機体の寸法（要確認：実機で測る）                                      */
/* ================================================================== */
#define TIRE_DIAMETER_MM        53.0f
#define TREAD_MM                85.0f   /* 左右のタイヤの間隔 */
#define CELL_MM                 180.0f  /* 迷路 1 区画 */

/* ================================================================== */
/* ステッピングモーター（PKE243A-L + SLA7073MPRT）                       */
/* ================================================================== */
/* 1 パルスで回る角度（PKE243A は 1.8deg。SLA7073 をハーフステップと想定） */
#define STEP_ANGLE_DEG          0.9f

/* パルス用タイマーのクロック
 *   TIM3: APB1 タイマー 90MHz  / 8  = 11.25MHz
 *   TIM8: APB2 タイマー 180MHz / 16 = 11.25MHz */
#define STEP_TIMER_HZ           11250000.0f

/* 出せるいちばん遅い速度（ARR=65535 のとき約 71.5mm/s）。これより遅いと止める */
#define WHEEL_MIN_SPEED         72.0f

/*
 * 車輪とピンの組み合わせ（要確認）
 * ------------------------------------------------------------
 * .ioc のピン名は PC8 = MOTOR_CLOCK_R、PC9 = MOTOR_CLOCK_L。
 * 動作していた前のプログラムは PC8 側を「左」と呼んでいたが、旋回と壁制御の
 * 向きから考えると、前のプログラムの左右の名前がそっくり逆で、実物は
 * .ioc のピン名どおりと思われる。
 *
 * このプログラムでは .ioc のピン名どおり（PC8 = 右、PC9 = 左）にしている。
 * パルスと回転方向ピンの組み合わせは、前のプログラムと同じ
 * （右 = PC8 + PB15、左 = PC9 + PB14）。
 * 実機で「前進で両輪が前に回る」「左回りの指令で左に回る」を確かめ、
 * 違っていたら下の値を変える。
 */
#define STEPPER_SWAP_WHEELS             0   /* 1 にすると左右を入れ替える */
#define STEPPER_RIGHT_DIR_FORWARD_HIGH  0   /* 右（PB15）: 前進のときの値 */
#define STEPPER_LEFT_DIR_FORWARD_HIGH   1   /* 左（PB14）: 前進のときの値 */

/* ================================================================== */
/* 走行                                                                */
/* ================================================================== */
/* 止まった状態からいきなり出してよい速さ（脱調しない範囲） */
#define MOTION_START_SPEED      100.0f  /* mm/s */
#define MOTION_TURN_START_SPEED 100.0f  /* deg/s（車輪では約 74mm/s） */

/* ================================================================== */
/* 赤外線の壁センサ                                                     */
/* ================================================================== */
/* ADC の並び: [0]=電池(PC5) [1]=SENSOR_1(PC3) [2]=SENSOR_2(PC2) [3]=SENSOR_3(PC1) [4]=SENSOR_4(PC0)
 * SENSORLED_1 で SENSOR_1,2、SENSORLED_2 で SENSOR_3,4 の LED が光る。
 * 前のプログラムは SENSOR_2 を「左」、SENSOR_3 を「右」と呼んでいたが、
 * 車輪と同じ理由で実物は逆（SENSOR_2 = 右）と思われる。要確認。 */
#define IR_FRONT_RIGHT          0   /* SENSOR_1 */
#define IR_RIGHT                1   /* SENSOR_2 */
#define IR_LEFT                 2   /* SENSOR_3 */
#define IR_FRONT_LEFT           3   /* SENSOR_4 */

/* LED を切り替えてから ADC で測るまでの待ち時間 [us] */
#define IR_SETTLE_US            20

/* 壁があるとみなす値（要調整：ログを見て決める） */
#define IR_WALL_SIDE            1500
#define IR_WALL_FRONT           450

/* 区画の中央にいるときの横センサの値（要調整） */
#define IR_CENTER_LEFT          2500
#define IR_CENTER_RIGHT         2500

/* 1 回の測定でこれ以上増えたら、壁の切れ目などで値が跳ねたとみなして使わない */
#define IR_JUMP_LIMIT           100

/* 横の壁制御（PD）。出力は左右の車輪の速度差の半分 [mm/s] */
#define WALL_KP                 0.02f
#define WALL_KD                 0.0f
#define WALL_REF_SPEED          300.0f  /* この速度のときにゲインが 1 倍 */
#define WALL_MAX_CORRECTION     60.0f   /* 補正の上限 [mm/s] */
/* 前壁にこれより近いときは横の壁制御をしない */
#define IR_FRONT_TOO_NEAR       2500

/* ================================================================== */
/* 電池                                                                */
/* ================================================================== */
/* 分圧比（R4 = 100k、R5 = 22k と想定。要確認） */
#define BATTERY_DIVIDER         ((100.0f + 22.0f) / 22.0f)
/* これより低い状態が 1 秒続いたら LED で知らせる（3S Li-Po を想定） */
#define BATTERY_LOW_V           10.8f

/* ================================================================== */
/* リモコン                                                            */
/* ================================================================== */
#define REMOTE_SPEED            300.0f  /* mm/s */
#define REMOTE_ACCEL            1000.0f /* mm/s^2 */
#define REMOTE_TURN_SPEED       200.0f  /* deg/s */
#define REMOTE_TURN_ACCEL       1000.0f /* deg/s^2 */
#define REMOTE_WALL_CONTROL     1       /* 前進中に横の壁で補正する */

/* 受信用 ESP32 から何も届かない時間がこれを超えたら止まる。0 で無効 */
#define LINK_TIMEOUT_MS         500

/* ================================================================== */
/* 動かし方（自律機とリモコン機はハードが同じ。ここだけ変えて書き込む）     */
/* ================================================================== */
#define ROBOT_MODE_REMOTE       0   /* リモコンで操作する */
#define ROBOT_MODE_AUTO         1   /* 自律で動く（app_auto.c） */
#ifndef ROBOT_MODE   /* make CFLAGS_EXTRA=-DROBOT_MODE=1 のように外からも決められる */
#define ROBOT_MODE              ROBOT_MODE_REMOTE
#endif

/* ================================================================== */
/* 自律走行（ROBOT_MODE_AUTO のときだけ使う）                           */
/* ================================================================== */
/*
 * 区画の座標 (x, y) は 0〜15。フィールドの角の 1 つを (0, 0) にし、
 * そこから x 方向・y 方向に数える（相手陣地は (15-x, 15-y) の点対称になる）。
 * 向きの番号: 0 = +x、1 = +y、2 = -x、3 = -y（左回りで 1 増える）。
 * 試合ごとに、公表されたスタート位置・ゲートの向き・ボールの位置に合わせて書き換える（規程 3-1、4-5）。
 */
#define AUTO_START_X            0
#define AUTO_START_Y            0
#define AUTO_START_DIR          1   /* スタート区画でゲートの方を向いた向き */

/* 自分の陣地のボールの区画（要設定） */
#define AUTO_BALL_COUNT         2
#define AUTO_BALLS              { { 2, 5 }, { 5, 2 } }

/* スタートゲート: 前に壁（ゲート）が見えている状態から、見えなくなってこの時間たったら走り出す */
#define AUTO_GATE_SEEN_MS       500   /* まずゲートがこの時間見えていること */
#define AUTO_GATE_OPEN_MS       300

/* 区画を走る速さ */
#define AUTO_SPEED              300.0f  /* mm/s */
#define AUTO_ACCEL              1000.0f /* mm/s^2 */
#define AUTO_TURN_SPEED         270.0f  /* deg/s */
#define AUTO_TURN_ACCEL         1500.0f /* deg/s^2 */

/*
 * ボールの区画の隣の区画の中央から、ボールの方へ進んでからつかむ距離（要確認）。
 * CAD では、つかむ姿勢のボールの中心は車軸から約 119mm 前 → 180 - 119 = 61mm。
 */
#define AUTO_APPROACH_MM        61.0f

/* ================================================================== */
/* アーム（RS204MD）  角度は 0.1deg 単位、時間は 10ms 単位               */
/* ================================================================== */
#define SERVO_ID_LOWER          1   /* 根元 */
#define SERVO_ID_UPPER          2   /* 先端 */

/* つかむ: 先端・根元を下ろす → 待つ → ファン ON */
#define ARM_CATCH_UPPER         500
#define ARM_CATCH_LOWER         1500
#define ARM_CATCH_TIME          50
#define ARM_CATCH_WAIT_MS       1000

/* ホーム: 根元を少し上げる → 待つ → 両方 0 度、ファン OFF */
#define ARM_HOME_LOWER_FIRST    450
#define ARM_HOME_FIRST_WAIT_MS  500
#define ARM_HOME_UPPER          0
#define ARM_HOME_LOWER          0
#define ARM_HOME_TIME           150
#define ARM_HOME_WAIT_MS        1500

/* 投げる: 構える → 待つ → 先端を振る → 少し待つ → 根元を振る */
#define ARM_THROW_READY_UPPER   1000
#define ARM_THROW_READY_LOWER   450
#define ARM_THROW_READY_TIME    75
#define ARM_THROW_READY_WAIT_MS 1000
#define ARM_THROW_UPPER         800
#define ARM_THROW_UPPER_WAIT_MS 50
#define ARM_THROW_LOWER         1200
#define ARM_THROW_TIME          10
#define ARM_THROW_END_WAIT_MS   300

/* ================================================================== */
/* 吸引ファン（TIM4_CH4 = PB9）                                         */
/* ================================================================== */
/* 比較値が小さいほど ON（P-MOSFET でオンにする回路と思われる） */
#define FAN_PWM_PERIOD          100
#define FAN_PWM_ON              2
#define FAN_PWM_OFF             100

/* ================================================================== */
/* ログ（USART2、115200bps）                                           */
/* ================================================================== */
#define LOG_ENABLE              1
#define LOG_INTERVAL_MS         100

#endif /* INC_CONFIG_H_ */
