# ファームウェア

## `MySWEEP1/` — 機体のプログラム（STM32）

DOM-SWEEP 基板の STM32F446 で動くプログラムです。**STM32CubeIDE** で開きます。

リモコンで走行・アーム・吸引ファンを動かせます。自律走行で使う関数（1区画進む、その場で回る、壁の判定）も入っています。

> ⚠️ 競技では、ロボットは自律で動かす必要があります（[ルールまとめ](../../docs/rules/robosweep-rules.md)）。今はリモコン操作だけなので、自律走行は Issue #6〜#12 で作っていきます。

| リモコン | 動作 |
| --- | --- |
| X / B | 押している間、前進 / 後退（前進は横の壁で補正） |
| Y / A | 押している間、その場で左 / 右に回る |
| F | アームを下ろしてボールをつかむ（吸引ファン ON） |
| R | アームをホームポジションに戻す（吸引ファン OFF） |
| L | ボールを投げる |
| Back | 吸引ファンの ON / OFF |

プログラムの構成、ピンの割り当て、通信の形式は [機体の仕様書](../../docs/machine-spec.md) にまとめてあります。調整する値（速度・しきい値・アームの角度など）は、すべて `Core/Inc/config.h` にあります。

### 開き方

1. STM32CubeIDE で「File → Import → Existing Projects into Workspace」を選び、`MySWEEP1` フォルダを指定する
2. ビルドすると `Debug/` フォルダができます（Git には入りません）
3. デバッグ設定（`.launch`）は各自の PC で作ってください（Git には入りません）

STM32CubeIDE がなくても、`arm-none-eabi-gcc` があればビルドできます。

```bash
cd software/firmware/MySWEEP1
make
```

走行の計算（`ctl_motion.c`）、壁を見た補正（`ctl_wall.c`）、迷路の地図と道順（`maze.c`）は、PC 上でテストできます。`maze.c` のテストは、でたらめな迷路 2000 個でボールの区画と投げる場所まで行けるかを確かめます。

```bash
make -C software/firmware/MySWEEP1/test
```

## `remote_controller/` — リモコン（ESP32、Arduino）

ボタンの状態を **ESP-NOW** で機体に送るリモコンのプログラムです。Arduino IDE で開きます。

- ボタン 8 個（A / B / X / Y / L / R / F / Back）と、状態を表示する LCD（AQM0802A、I2C）
- 50ms ごとに送信
- 送り先の MAC アドレスは `receiverAddress` に書きます。機体を替えたら書き換えてください

## `robot_receiver/` — 機体に載せる受信用 ESP32（Arduino）

リモコンから ESP-NOW で届いたボタンの状態を、STM32 に UART で渡すプログラムです。

- XIAO ESP32C3 を想定しています。違うボードのときは、先頭の `PIN_UART_TX` を変えてください。
- 配線：ESP32 の TX（D6）を STM32 の UART4_RX（PA1、J4 の 3 番）に、GND 同士もつなぎます。
- 起動すると、USB シリアル（115200bps）にこの ESP32 の MAC アドレスを表示します。その値を、リモコンの `receiverAddress` に書いてください。
- どのボタンでどう動くかは、`to_stm32_buttons()` で変えられます。

| リモコン | 動作 |
| --- | --- |
| X / B | 前進 / 後退 |
| Y / A | 左旋回 / 右旋回 |
| F | つかむ |
| R | ホームポジション |
| L | 投げる |
| Back | 吸引ファン ON / OFF |
