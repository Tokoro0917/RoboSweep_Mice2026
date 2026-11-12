#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

// ==========================================
// 1. LCD (AQM0802A) 制御用の直叩き関数
// ==========================================
const int I2C_ADDR = 0x3E;

void lcd_cmd(uint8_t cmd) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x00);
  Wire.write(cmd);
  Wire.endTransmission();
  delay(1);
}

void lcd_data(uint8_t data) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x40);
  Wire.write(data);
  Wire.endTransmission();
  delay(1);
}

void lcd_init() {
  delay(100);
  lcd_cmd(0x38); // Function set: 8-bit, 2-line, normal font, IS=0
  lcd_cmd(0x39); // Function set: 8-bit, 2-line, normal font, IS=1
  lcd_cmd(0x14); // Internal OSC frequency
  lcd_cmd(0x73); // Contrast set (見えにくい場合は 0x70〜0x7F で調整)
  lcd_cmd(0x56); // Power/ICON/Contrast: Booster ON (3.3V用)
  lcd_cmd(0x6C); // Follower control
  delay(200);    // 電圧昇圧待ち
  lcd_cmd(0x38); // Function set: IS=0
  lcd_cmd(0x0C); // Display ON, cursor OFF
  lcd_cmd(0x01); // Clear display
  delay(2);
}

void lcd_print(const char* str) {
  while (*str) {
    lcd_data(*str++);
  }
}

// ==========================================
// 2. ESP-NOW 通信設定
// ==========================================

// ★ここに機体側のMACアドレスを入力してください (例: 0x34, 0x85, 0x18, ...)
uint8_t receiverAddress[] = {0x90, 0x70, 0x69, 0x12, 0xBF, 0x90};

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

struct_message myData;

// ピン割り当て
const int PIN_SW_X = D0;
const int PIN_SW_A = D1;
const int PIN_SW_B = D2;
const int PIN_SW_Y = D3;
// SDA=D4, SCL=D5 はWire.beginで使用
const int PIN_SW_BACK = D6;
const int PIN_SW_R = D8;
const int PIN_SW_L = D9;
const int PIN_SW_F = D10;

String lastDisplay = ""; // 画面のちらつき防止用

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // 送信完了時のコールバック（必要に応じて処理追加）
}

// ==========================================
// 3. セットアップ
// ==========================================
void setup() {
  Serial.begin(115200);

  // LCDの初期化
  Wire.begin(D4, D5);
  lcd_init();

  lcd_cmd(0x80); // 1行目の先頭
  lcd_print("ESP-NOW ");
  lcd_cmd(0xC0); // 2行目の先頭
  lcd_print("Ready...");
  delay(1500);
  lcd_cmd(0x01); // 画面クリア
  delay(2);

  // ボタンピンの設定
  pinMode(PIN_SW_A, INPUT);
  pinMode(PIN_SW_B, INPUT);
  pinMode(PIN_SW_X, INPUT);
  pinMode(PIN_SW_Y, INPUT);
  pinMode(PIN_SW_L, INPUT);
  pinMode(PIN_SW_R, INPUT);
  pinMode(PIN_SW_F, INPUT);
  pinMode(PIN_SW_BACK, INPUT);

  // ESP-NOWの初期化
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    lcd_cmd(0x80); lcd_print("ERR:WIFI");
    return;
  }
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);

  // ピア(機体側)の登録
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    lcd_cmd(0x80); lcd_print("ERR:PEER");
    return;
  }
}

// ==========================================
// 4. メインループ
// ==========================================
void loop() {
  // ボタン状態取得 (押されるとLOWなので反転)
  myData.btnA    = !digitalRead(PIN_SW_A);
  myData.btnB    = !digitalRead(PIN_SW_B);
  myData.btnX    = !digitalRead(PIN_SW_X);
  myData.btnY    = !digitalRead(PIN_SW_Y);
  myData.btnL    = !digitalRead(PIN_SW_L);
  myData.btnR    = !digitalRead(PIN_SW_R);
  myData.btnF    = !digitalRead(PIN_SW_F);
  myData.btnBack = !digitalRead(PIN_SW_BACK);

  // LCD表示文字列の生成 (8文字制限)
  String currentDisplay = "IDLE    ";
  
  if (myData.btnF) {
    currentDisplay = "CMD: F  ";
  } else if (myData.btnBack) {
    currentDisplay = "CMD: BAC"; 
  } else if (myData.btnL) {
    currentDisplay = "CMD: L  ";
  } else if (myData.btnR) {
    currentDisplay = "CMD: R  ";
  } else if (myData.btnX) {
    currentDisplay = "CMD: X  ";
  } else if (myData.btnY) {
    currentDisplay = "CMD: Y  ";
  } else if (myData.btnA) {
    currentDisplay = "CMD: A  ";
  } else if (myData.btnB) {
    currentDisplay = "CMD: B  ";
  }

  // 状態が変わった時だけLCDを更新
  if (currentDisplay != lastDisplay) {
    lcd_cmd(0x80); // 1行目
    lcd_print("Status: ");
    
    lcd_cmd(0xC0); // 2行目
    lcd_print(currentDisplay.c_str());
    
    lastDisplay = currentDisplay;
  }

  // データ送信
  esp_now_send(receiverAddress, (uint8_t *) &myData, sizeof(myData));
  
  delay(50); // 送信間隔 (50ms)
}