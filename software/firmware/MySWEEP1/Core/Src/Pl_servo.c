/*
 * Pl_servo.c
 *
 *  Created on: Aug 27, 2026
 *      Author: kodamakotaro
 */


#include "usart.h"
#include "gpio.h"
#include "PL_servo.h"


void servo_On(){
	Torque(0x01, 0x01);         // ID = 1(0x01) , torque = ON   (0x01)
	Torque(0x02, 0x01);         // ID = 2(0x01) , torque = ON   (0x01)
}

void Torque (unsigned char ID, unsigned char data){
    unsigned char TxData[9];    // 送信�?ータバッファ [9byte]
    unsigned char CheckSum = 0; // チェ�?クサ�?計算用変数

    // パケ�?トデータ生�??
    TxData[0] = 0xFA;           // Header
    TxData[1] = 0xAF;           // Header
    TxData[2] = ID;             // ID
    TxData[3] = 0x00;           // Flags
    TxData[4] = 0x24;           // Address
    TxData[5] = 0x01;           // Length
    TxData[6] = 0x01;           // Count
    TxData[7] = data;           // Data
    // チェ�?クサ�?計�?
    for(int i=2; i<=7; i++){
        CheckSum = CheckSum ^ TxData[i]; // ID?��DATAまでのXOR
    }
    TxData[8] = CheckSum;       // Sum


    for(int i=0; i<=8; i++){
    	HAL_UART_Transmit(&huart1, &TxData[i], sizeof(TxData[i]), 1000);


    }

}

/*-------------------------------------------------*/
/* 名前   : Move                                   */
/* 引数   : ID (Servo ID)                          */
/*        : Angle (Present Posion L&H)             */
/*        : Speed (Present Time L&H)               */
/* 戻り�?� : 無�?                                   */
/*-------------------------------------------------*/
void Move (unsigned char ID, int Angle, int Speed){
    unsigned char TxData[12];   // 送信�?ータバッファ [12byte]
    unsigned char CheckSum = 0; // チェ�?クサ�?計算用変数

    // パケットデータ生成
    TxData[0] = 0xFA;           // Header
    TxData[1] = 0xAF;           // Header
    TxData[2] = ID;             // ID
    TxData[3] = 0x00;           // Flags
    TxData[4] = 0x1E;           // Address
    TxData[5] = 0x04;           // Length
    TxData[6] = 0x01;           // Count
    // Angle
    TxData[7] = (unsigned char)0x00FF & Angle;           // Low byte
    TxData[8] = (unsigned char)0x00FF & (Angle >> 8);    // Hi  byte
    // Speed
    TxData[9] = (unsigned char)0x00FF & Speed;           // Low byte
    TxData[10] = (unsigned char)0x00FF & (Speed >> 8);   // Hi  byte
    // チェックサム計算
    for(int i=2; i<=10; i++){
        CheckSum = CheckSum ^ TxData[i];  // ID?��DATAまでのXOR
    }
    TxData[11] = CheckSum;       // Sum

    // データ送信

    for(int i=0; i<=11; i++){
    	HAL_UART_Transmit(&huart1, &TxData[i], sizeof(TxData[i]), 1000);
    }

}
