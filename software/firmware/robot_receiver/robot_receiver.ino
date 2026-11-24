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
 *      STM32 --UART--> この ESP32 --ESP-NOW--> もう 1 台の ESP32 --UART--> STM32
 *
 * 3. HAS_CAMERA を 1 にすると（XIAO ESP32S3 Sense）、カメラで黄色いボールを探し、結果を STM32 に送る。
 *      STM32 に送る形: 0xB0, 見つけたか, 向き[deg], 距離[cm], 大きさ[画素], 番号, チェック（XOR）
 *    ボールを見つける計算は ball_detect.h（PC でテストできる）。
 *    C3 など、カメラのないボードでは HAS_CAMERA を 0 にする（1 と 2 だけ動く）。
 *
 * ほかのチームの電波を取り違えないように、送ってきた相手の MAC アドレスを確かめる
 * （REMOTE_MAC、ROBOT_MACS）。
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

// カメラを使うか。XIAO ESP32S3 Sense は 1、XIAO ESP32C3 は 0。
// 1 のときは Arduino IDE で ボード「XIAO_ESP32S3」、PSRAM を有効（OPI PSRAM）にする。
#define HAS_CAMERA 0

#if HAS_CAMERA
#include "esp_camera.h"
#include "ball_detect.h"
#endif

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

// ------------------------------------------
// 送ってきた相手を確かめる（ほかのチームの電波を取り違えないため）
// MAC アドレスは、それぞれの ESP32 を起動したときに USB シリアルに表示される。
// ------------------------------------------
// リモコンの MAC アドレス（remote_controller.ino が "Remote MAC address" として表示する）。
// すべて 0 のままだと、どこから届いたボタンも使う（前と同じ動き。起動時に注意を表示する）。
const uint8_t REMOTE_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 自律機（2 台）の ESP32 の MAC アドレス（この ESP32 が "MAC address" として表示する）。
// 2 台とも同じ内容を書き込んでよい（自分の分は自動でとばす）。
// 位置は、ここに書いた相手にだけ送り、ここに書いた相手から届いたものだけを使う。
// すべて 0 のままだと、位置は送りも受けもしない（自律機 2 台で動かすときは必ず書く）。
const uint8_t ROBOT_MACS[2][6] = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // 1 台目（AUTO_ROBOT_ID 0）
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},   // 2 台目（AUTO_ROBOT_ID 1）
};
const int ROBOT_MAC_COUNT = 2;

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

static bool is_zero_mac(const uint8_t *m) {
  for (int i = 0; i < 6; i++) if (m[i] != 0) return false;
  return true;
}

static bool same_mac(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

// 自律機の一覧に入っている相手か
static bool is_robot(const uint8_t *mac) {
  for (int i = 0; i < ROBOT_MAC_COUNT; i++) {
    if (!is_zero_mac(ROBOT_MACS[i]) && same_mac(ROBOT_MACS[i], mac)) return true;
  }
  return false;
}

volatile uint32_t rejected_count = 0;   // 知らない相手から届いて捨てた数

static void handle_data(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == POS_LEN && data[0] == POS_HEADER) {
    if (!is_robot(mac)) {
      rejected_count++;
      return;   // 自分たちの自律機ではない
    }
    // もう 1 台の位置。loop() で STM32 に渡す
    memcpy(peer_packet, data, POS_LEN);
    peer_pending = true;
    peer_recv_count++;
    return;
  }
  if (len != (int)sizeof(struct_message)) {
    return;   // 形が違うデータは使わない
  }
  if (!is_zero_mac(REMOTE_MAC) && !same_mac(REMOTE_MAC, mac)) {
    rejected_count++;
    return;   // 自分たちのリモコンではない
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
  handle_data(info->src_addr, data, len);
}
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  handle_data(mac, data, len);
}
#endif

// ==========================================
// STM32 に送る
// ==========================================
void send_to_stm32(uint8_t buttons) {
  uint8_t packet[3] = {0xAA, buttons, 0x55};
  Serial1.write(packet, sizeof(packet));
}

// 自分の位置を、もう 1 台へ（ROBOT_MACS に書いた相手のうち、自分以外にだけ送る）
uint8_t my_mac[6];
uint32_t pos_sent_count = 0;

void send_position(const uint8_t *buf) {
  for (int i = 0; i < ROBOT_MAC_COUNT; i++) {
    if (is_zero_mac(ROBOT_MACS[i]) || same_mac(ROBOT_MACS[i], my_mac)) continue;
    esp_now_send(ROBOT_MACS[i], buf, POS_LEN);
    pos_sent_count++;
  }
}

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
        send_position(buf);
      }
    }
  }
}

#if HAS_CAMERA
// ==========================================
// カメラでボールを探す（別のタスクで動かし、loop() を待たせない）
// ==========================================
// XIAO ESP32S3 Sense のカメラのピン（Seeed の wiki の表より。PWDN / RESET はつながっていない）
#define CAM_PIN_PWDN  -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK  10
#define CAM_PIN_SIOD  40
#define CAM_PIN_SIOC  39
#define CAM_PIN_D7    48   // Y9
#define CAM_PIN_D6    11   // Y8
#define CAM_PIN_D5    12   // Y7
#define CAM_PIN_D4    14   // Y6
#define CAM_PIN_D3    16   // Y5
#define CAM_PIN_D2    18   // Y4
#define CAM_PIN_D1    17   // Y3
#define CAM_PIN_D0    15   // Y2
#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF  47
#define CAM_PIN_PCLK  13

// 画像の大きさ。QQVGA（160×120）にすると速い
const int CAM_W = 160;
const int CAM_H = 120;
// RGB565 のバイトの順番。黄色いボールが見つからず、ほかの色で反応するときは 0 にしてみる（要確認）
const int CAM_SWAP_BYTES = 1;

uint8_t *cam_mask = nullptr;
int32_t *cam_stack = nullptr;

// 最新の結果（カメラのタスクが書き、loop() が読む）
volatile bool ball_new = false;
volatile uint8_t ball_found = 0;
volatile int8_t ball_deg = 0;
volatile uint8_t ball_cm = 0;
volatile uint8_t ball_size = 0;
volatile uint32_t cam_frame_ms = 0;   // 1 枚にかかった時間（確認用）
volatile uint32_t cam_frames = 0;
uint8_t ball_seq = 0;

bool camera_begin() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = CAM_PIN_D0; c.pin_d1 = CAM_PIN_D1; c.pin_d2 = CAM_PIN_D2; c.pin_d3 = CAM_PIN_D3;
  c.pin_d4 = CAM_PIN_D4; c.pin_d5 = CAM_PIN_D5; c.pin_d6 = CAM_PIN_D6; c.pin_d7 = CAM_PIN_D7;
  c.pin_xclk = CAM_PIN_XCLK;
  c.pin_pclk = CAM_PIN_PCLK;
  c.pin_vsync = CAM_PIN_VSYNC;
  c.pin_href = CAM_PIN_HREF;
  c.pin_sccb_sda = CAM_PIN_SIOD;
  c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_pwdn = CAM_PIN_PWDN;
  c.pin_reset = CAM_PIN_RESET;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;
  c.frame_size = FRAMESIZE_QQVGA;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;   // いちばん新しい画像を使う
  if (esp_camera_init(&c) != ESP_OK) {
    return false;
  }
  cam_mask = (uint8_t *)ps_malloc(CAM_W * CAM_H);
  cam_stack = (int32_t *)ps_malloc(CAM_W * CAM_H * sizeof(int32_t));
  return cam_mask && cam_stack;
}

void camera_task(void *arg) {
  (void)arg;
  for (;;) {
    uint32_t t0 = millis();
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      delay(10);
      continue;
    }
    if (fb->width == CAM_W && fb->height == CAM_H && fb->format == PIXFORMAT_RGB565) {
      ball_blob_t b;
      ball_detect((const uint16_t *)fb->buf, CAM_W, CAM_H, CAM_SWAP_BYTES, cam_mask, cam_stack, &b);
      if (b.found) {
        float deg = ball_bearing_deg(&b, CAM_W);
        float cm = ball_distance_mm(&b, CAM_W) / 10.0f;
        ball_deg = (int8_t)constrain((int)lroundf(deg), -90, 90);
        ball_cm = (uint8_t)constrain((int)lroundf(cm), 0, 254);
        int size = b.w > b.h ? b.w : b.h;
        ball_size = (uint8_t)constrain(size, 0, 255);
        ball_found = b.clipped ? 2 : 1;   // 2 = 画像の端で切れている（距離が遠めに出る）
      } else {
        ball_found = 0;
      }
      ball_new = true;
    }
    esp_camera_fb_return(fb);
    cam_frame_ms = millis() - t0;
    cam_frames++;
    delay(1);   // ほかのタスク（Wi-Fi など）にも回す
  }
}

// STM32 に送る: 0xB0, 見つけたか, 向き, 距離, 大きさ, 番号, チェック
void send_ball_to_stm32() {
  uint8_t p[7];
  p[0] = 0xB0;
  p[1] = ball_found;
  p[2] = (uint8_t)ball_deg;
  p[3] = ball_cm;
  p[4] = ball_size;
  p[5] = ball_seq++;
  p[6] = p[1] ^ p[2] ^ p[3] ^ p[4] ^ p[5];
  Serial1.write(p, sizeof(p));
}
#endif

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

  // もう 1 台の自律機を送り先に登録する（自分は登録しない）
  WiFi.macAddress(my_mac);
  int peers = 0;
  for (int i = 0; i < ROBOT_MAC_COUNT; i++) {
    if (is_zero_mac(ROBOT_MACS[i]) || same_mac(ROBOT_MACS[i], my_mac)) continue;
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, ROBOT_MACS[i], 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("ERR: esp_now_add_peer (robot)");
    } else {
      peers++;
    }
  }
  if (is_zero_mac(REMOTE_MAC)) {
    Serial.println("NOTE: REMOTE_MAC is not set. Buttons from any sender are used.");
  }
  if (peers == 0) {
    Serial.println("NOTE: ROBOT_MACS is not set. Positions are not sent or received.");
  }
#if HAS_CAMERA
  if (camera_begin()) {
    // loop() と同じコア 1 で、同じ優先度で動かす（Wi-Fi はコア 0）
    xTaskCreatePinnedToCore(camera_task, "camera", 8192, nullptr, 1, nullptr, 1);
    Serial.println("Camera ready");
  } else {
    Serial.println("ERR: camera init (PSRAM が有効か確かめる)");
  }
#endif
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

#if HAS_CAMERA
  // ボールを探した結果（新しい結果が出るたびに送る）
  if (ball_new) {
    ball_new = false;
    send_ball_to_stm32();
  }
#endif

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
    Serial.printf("recv=%lu buttons=0x%02X pos_sent=%lu peer_recv=%lu rejected=%lu %s\n",
                  (unsigned long)recv_count, buttons,
                  (unsigned long)pos_sent_count, (unsigned long)peer_recv_count,
                  (unsigned long)rejected_count,
                  (now - last_recv_ms > LINK_TIMEOUT_MS) ? "(no signal)" : "");
#if HAS_CAMERA
    // カメラ: 1 秒の枚数、1 枚にかかった時間、ボール（見つけたか、向き、距離、大きさ）
    static uint32_t last_frames = 0;
    Serial.printf("camera fps=%lu frame_ms=%lu ball=%u deg=%d cm=%u size=%u\n",
                  (unsigned long)(cam_frames - last_frames), (unsigned long)cam_frame_ms,
                  ball_found, ball_deg, ball_cm, ball_size);
    last_frames = cam_frames;
#endif
    last_log_ms = now;
  }
}
