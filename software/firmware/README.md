# ファームウェア

## `MySWEEP1/` — リモコン操作のテスト用プログラム（STM32）

DOM-SWEEP 基板の STM32F446 で動くプログラムです。**STM32CubeIDE** で開きます。

リモコンのボタンで、走行・アーム・吸引ファンを動かして**ハードウェアの動作を確認するためのもの**です。

> ⚠️ 競技ではロボットは自律で動かす必要があります（[ルールまとめ](../../docs/rules/robosweep-rules.md)）。
> このプログラムは試験用で、そのまま競技には使えません。

| ボタン | 動作 |
| --- | --- |
| 前 / 後 | 押している間、前進 / 後退（前進は壁センサで姿勢を補正） |
| 右 / 左 | 押している間、その場で旋回 |
| F | アームを下ろしてボールをつかむ（吸引ファン ON） |
| R | アームをホームポジションに戻す（吸引ファン OFF） |
| L | ボールを投げる |
| B | 吸引ファンの ON / OFF |

主なファイル:

- `Core/Src/main.c`: ボタンごとの動作
- `Core/Src/PL_Motor.c`: ステッピングモーターの台形加減速、壁センサを使った制御
- `Core/Src/PL_sensor.c`: 壁センサ（AD 変換）
- `Core/Src/Pl_servo.c`: アーム用サーボ（RS204MD）の通信
- `MySWEEP1.ioc`: CubeMX のピン・周辺機能の設定

ボタンの状態は UART で受け取っています（115200bps）。

### 開き方

1. STM32CubeIDE で「File → Import → Existing Projects into Workspace」を選び、`MySWEEP1` フォルダを指定する
2. ビルドすると `Debug/` フォルダができます（Git には入りません）
3. デバッグ設定（`.launch`）は各自の PC で作ってください（Git には入りません）

## `remote_controller/` — リモコン（ESP32、Arduino）

ボタンの状態を **ESP-NOW** で機体に送るリモコンのプログラムです。Arduino IDE で開きます。

- ボタン 8 個（A / B / X / Y / L / R / F / Back）と、状態を表示する LCD（AQM0802A、I2C）
- 50ms ごとに送信
- 送り先の MAC アドレスは `receiverAddress` に書きます。機体を替えたら書き換えてください

> 機体側で ESP-NOW を受信して STM32 に UART で渡すプログラムは、まだリポジトリに入っていません。
