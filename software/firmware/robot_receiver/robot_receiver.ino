/*
 * robot_receiver.ino
 *
 * 機体に載せる ESP32 のプログラム（Arduino）。
 *
 * 1. リモコン（remote_controller.ino）から ESP-NOW で届いたボタンの状態を、
 *    1 バイトにまとめて STM32 に UART で送る。
 *      リモコン --ESP-NOW--> この ESP32 --UART 115200bps--> STM32（UART4）
 *      STM32 に送る形: 0xAA, ボタン（1 バイト）, 0x55
 *
 * 2. 自律機が 2 台のとき、STM32 から届いた自分の位置を、もう 1 台の ESP32 に ESP-NOW で送る。
 *    もう 1 台から届いた位置は、先頭を 0xA6 にして STM32 に渡す。
 *      STM32 --UART--> この ESP32 --ESP-NOW（全員あて）--> もう 1 台の ESP32 --UART--> STM32
 *      STM32 から届く形: 0xA5, 番号, x, y, 次の x, 次の y, 状態, チェック（XOR）
 *
 * STM32 側は software/firmware/MySWEEP1/Core/Src/drv_link.c
 *
 * 起動すると、この ESP32 の MAC アドレスを USB シリアル（115200bps）に表示する。
 * その値を remote_controller.ino の receiverAddress に書くこと。
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ==========================================
// 設定（ボードや配線が違うときはここを変える）
// ==========================================

// STM32 とつなぐ UART のピン。XIAO ESP32C3 の TX=D6, RX=D7 を想定。
// この ESP32 の TX を STM32 の UART4_RX（PA1、J4 の 3 番）に、
// GND 同士をつなぐ。自律機を 2 台で動かすときは、この ESP32 の RX を STM32 の UART4_TX（PA0、J4 の 1 番）にもつなぐ。
const int PIN_UART_TX = D6;
const int PIN_UART_RX = D7;
const uint32_t UART_BAUD = 115200;

// リモコンと同じチャンネルにする（remote_controller.ino の peerInfo.channel）
const uint8_t WIFI_CHANNEL = 1;

// リモコンから届かなくなってから、ボタンを離した扱いにするまでの時間 [ms]
const uint32_t LINK_TIMEOUT_MS = 200;

// STM32 に送る間隔 [ms]。STM32 は 500ms 届かないと止まるので、それより短くする。
const uint32_t SEND_INTERVAL_MS = 50;

// ==========================================
// STM32 側のボタンのビット（drv_link.h と同じ）
// ==========================================
const uint8_t BTN_R     = 0x01;  // アーム: ホームポジション
const uint8_t BTN_B     = 0x02;  // 吸引ファン ON/OFF
const uint8_t BTN_F     = 0x04;  // アーム: つかむ
const uint8_t BTN_L     = 0x08;  // アーム: 投げる
const uint8_t BTN_WEST  = 0x10;  // 左に回る
const uint8_t BTN_EAST  = 0x20;  // 右に回る
const uint8_t BTN_NORTH = 0x40;  // 前に進む
const uint8_t BTN_SOUTH = 0x80;  // 後ろに進む

// ==========================================
// リモコンから届くデータ（remote_controller.ino と同じ形にすること）
// ==========================================
typedef struct struct_message {
  bool btnA;
  bool btnB;
  bool btnX;
  bool btnY;
  bool btnL;
  bool btnR;
  bool btnF;
  bool btnBack;
} struct_message;

// リモコンのボタン → STM32 のビット
//   X / B / Y / A を 上 / 下 / 左 / 右 として、前進 / 後退 / 左旋回 / 右旋回 に使う。
//   実機で向きが違ったら、ここを入れ替える。
uint8_t to_stm32_buttons(const struct_message &m) {
  uint8_t b = 0;
  if (m.btnX)    b |= BTN_NORTH;
  if (m.btnB)    b |= BTN_SOUTH;
  if (m.btnY)    b |= BTN_WEST;
  if (m.btnA)    b |= BTN_EAST;
  if (m.btnF)    b |= BTN_F;
  if (m.btnR)    b |= BTN_R;
  if (m.btnL)    b |= BTN_L;
  if (m.btnBack) b |= BTN_B;
  return b;   // 2 つ以上押すと、STM32 側では何もしない
}

// ==========================================
// 受信
// ==========================================
volatile uint8_t latest_buttons = 0;
volatile uint32_t last_recv_ms = 0;
volatile uint32_t recv_count = 0;

// もう 1 台の自律機の位置（STM32 からの形と同じ 8 バイト、先頭 0xA5）
const uint8_t POS_HEADER = 0xA5;
const int POS_LEN = 8;
uint8_t peer_packet[POS_LEN];
volatile bool peer_pending = false;
volatile uint32_t peer_recv_count = 0;

static void handle_data(const uint8_t *data, int len) {
  if (len == POS_LEN && data[0] == POS_HEADER) {
    // もう 1 台の位置。loop() で STM32 に渡す
    memcpy(peer_packet, data, POS_LEN);
    peer_pending = true;
    peer_recv_count++;
    return;
  }
  if (len != (int)sizeof(struct_message)) {
    return;   // 形が違うデータは使わない
  }
  struct_message m;
  memcpy(&m, data, sizeof(m));
  latest_buttons = to_stm32_buttons(m);
  last_recv_ms = millis();
  recv_count++;
}

// Arduino-ESP32 のバージョン 3 から、受信コールバックの引数が変わった
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  handle_data(data, len);
}
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  handle_data(data, len);
}
#endif

// ==========================================
// STM32 に送る
// ==========================================
void send_to_stm32(uint8_t buttons) {
  uint8_t packet[3] = {0xAA, buttons, 0x55};
  Serial1.write(packet, sizeof(packet));
}

// 自分の位置を、もう 1 台へ（全員あてに送る。リモコンの ESP32 は受け取っても使わない）
const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t pos_sent_count = 0;

// STM32 から届くバイトを集めて、1 回分そろったら送る
void read_from_stm32() {
  static uint8_t buf[POS_LEN];
  static int n = 0;
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (n == 0 && b != POS_HEADER) {
      continue;   // 先頭を待つ
    }
    buf[n++] = b;
    if (n == POS_LEN) {
      n = 0;
      uint8_t sum = 0;
      for (int i = 1; i < POS_LEN - 1; i++) sum ^= buf[i];
      if (sum == buf[POS_LEN - 1]) {
        esp_now_send(BROADCAST, buf, POS_LEN);
        pos_sent_count++;
      }
    }
  }
}

// ==========================================
// セットアップ
// ==========================================
void setup() {
  Serial.begin(115200);   // USB シリアル（確認用）
  Serial1.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  // 起動直後はボタンなしを送っておく
  send_to_stm32(0);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  delay(500);
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERR: esp_now_init");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // 全員あて（もう 1 台の自律機へ位置を送る）
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, BROADCAST, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERR: esp_now_add_peer (broadcast)");
  }
  Serial.println("Ready");
}

// ==========================================
// メインループ
// ==========================================
void loop() {
  static uint32_t last_send_ms = 0;
  static uint32_t last_log_ms = 0;
  uint32_t now = millis();

  // リモコンから届かなくなったら、ボタンを離した扱いにする（止まる）
  uint8_t buttons = latest_buttons;
  if (now - last_recv_ms > LINK_TIMEOUT_MS) {
    buttons = 0;
  }

  // 自律機どうしの位置
  read_from_stm32();
  if (peer_pending) {
    uint8_t out[POS_LEN];
    memcpy(out, peer_packet, POS_LEN);
    peer_pending = false;
    out[0] = 0xA6;   // STM32 には「もう 1 台の位置」として渡す
    Serial1.write(out, POS_LEN);
  }

  // 押している間も離している間も、決まった間隔で送り続ける
  if (now - last_send_ms >= SEND_INTERVAL_MS) {
    send_to_stm32(buttons);
    last_send_ms = now;
  }

  // 1 秒ごとに状態を USB シリアルに出す（確認用）
  if (now - last_log_ms >= 1000) {
    Serial.printf("recv=%lu buttons=0x%02X pos_sent=%lu peer_recv=%lu %s\n",
                  (unsigned long)recv_count, buttons,
                  (unsigned long)pos_sent_count, (unsigned long)peer_recv_count,
                  (now - last_recv_ms > LINK_TIMEOUT_MS) ? "(no signal)" : "");
    last_log_ms = now;
  }
}
