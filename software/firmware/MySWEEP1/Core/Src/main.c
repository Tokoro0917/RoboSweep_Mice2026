/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "PL_servo.h"
#include "PL_Motor.h"
#include "PL_sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RB 1
#define LB 8
#define FB 4
#define BB 2
#define NB 64
#define EB 32
#define SB 128
#define WB 16
#define HOLD_STRAIGHT_SPEED 300
#define HOLD_TURN_SPEED 200
#define HOLD_STRAIGHT_ACCEL 1000
#define HOLD_TURN_ACCEL 1000
#define HOLD_STRAIGHT_START_SPEED 100
#define HOLD_TURN_START_SPEED 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rxByte = 0;                    // 1バイトずつ受け取るための変数
volatile uint8_t buttonState = 0;
int isfun = 0;
static uint8_t prevButtonState = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim8);
	// HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	// 右輪はTIM8のチャンネル4（PC9）でPWM出力
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	// グローバル変数

	// setup部 (while(1)の直前) で受信割り込みを開始

	/* USER CODE BEGIN 2 */

	// 1バイトずつ割り込みで受信を開始する
	HAL_UART_Receive_IT(&huart4, &rxByte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
		if (isfun) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2);
			__HAL_TIM_SET_AUTORELOAD(&htim4, 100);
		} else {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 100);
			__HAL_TIM_SET_AUTORELOAD(&htim4, 100);
		}
		//printf("SEN1=%f,SEN2=%f,SEN3=%f,SEN4=%f\n\r", nowf1,nowl,nowr,nowf2);
		//printf("%d\r\n", buttonState);

		uint8_t pressedEdge = (buttonState != prevButtonState);

		switch (buttonState) {
		case NB: // 前進(押している間だけ)
			motor_start_excitation();
			motor_start();
			isrotate = 0;
			isback = 0;
			Gv = HOLD_STRAIGHT_START_SPEED;
			Gacc = HOLD_STRAIGHT_ACCEL;
			isdaikei = 1;
			iswallcontrol = 1;
			while (Gv < HOLD_STRAIGHT_SPEED && buttonState == NB) {
			}
			if (buttonState == NB) {
				Gv = HOLD_STRAIGHT_SPEED;
				Gacc = 0;
				while (buttonState == NB) {
				}
			}
			iswallcontrol = 0;
			isdaikei = 0;
			Gacc = 0;
			Gv = 0;
			motor_stop();
			break;

		case SB: // 後退(押している間だけ)
			motor_start_excitation();
			motor_start();
			isrotate = 0;
			isback = 1;
			Gv = HOLD_STRAIGHT_START_SPEED;
			Gacc = HOLD_STRAIGHT_ACCEL;
			isdaikei = 1;
			while (Gv < HOLD_STRAIGHT_SPEED && buttonState == SB) {
			}
			if (buttonState == SB) {
				Gv = HOLD_STRAIGHT_SPEED;
				Gacc = 0;
				while (buttonState == SB) {
				}
			}
			isdaikei = 0;
			Gacc = 0;
			Gv = 0;
			isback = 0;
			motor_stop();
			break;

		case EB: // 右旋回(押している間だけ)
			motor_start_excitation();
			motor_start();
			isrotate = 1;
			direc = ROTATE_RIGHT;
			iswallcontrol = 0;
			Gv = cal_turnV(HOLD_TURN_START_SPEED);
			Gacc = cal_turnV(HOLD_TURN_ACCEL);
			isdaikei = 1;
			while (Gv < cal_turnV(HOLD_TURN_SPEED) && buttonState == EB) {
			}
			if (buttonState == EB) {
				Gv = cal_turnV(HOLD_TURN_SPEED);
				Gacc = 0;
				while (buttonState == EB) {
				}
			}
			isdaikei = 0;
			Gacc = 0;
			Gv = 0;
			isrotate = 0;
			motor_stop();
			break;

		case WB: // 左旋回(押している間だけ)
			motor_start_excitation();
			motor_start();
			isrotate = 1;
			direc = ROTATE_LEFT;
			iswallcontrol = 0;
			Gv = cal_turnV(HOLD_TURN_START_SPEED);
			Gacc = cal_turnV(HOLD_TURN_ACCEL);
			isdaikei = 1;
			while (Gv < cal_turnV(HOLD_TURN_SPEED) && buttonState == WB) {
			}
			if (buttonState == WB) {
				Gv = cal_turnV(HOLD_TURN_SPEED);
				Gacc = 0;
				while (buttonState == WB) {
				}
			}
			isdaikei = 0;
			Gacc = 0;
			Gv = 0;
			isrotate = 0;
			motor_stop();
			break;

		case FB: // アーム動作1(1回押しで実行) キャッチ
			if (pressedEdge) {
				servo_On();
				Move(2, 500, 50); //ID=2 50度へ
				//下
				Move(1, 1500, 50); //ID=1 150度へ
				HAL_Delay(1000);
				isfun = 1; // キャッチ完了で吸引ファンON
			}
			break;

		case RB: // アーム動作2(1回押しで実行)　ホームポジション
			if (pressedEdge) {
				servo_On();
				Move(1, 450, 150); //ID=下 45度へ
				HAL_Delay(500);
				Move(2, 00, 150); //ID=2上 0度へ
				Move(1, 00, 150); //ID=1下 0度へ
				isfun = 0; // ホームポジション完了で吸引ファンOFF
			}
			break;

		case LB: // アーム動作3(1回押しで実行)　投げる
			if (pressedEdge) {
				servo_On();
				Move(2, 1000, 75); //ID=2 100度へ
				//下
				Move(1, 450, 75); //ID=1 45度へ
				HAL_Delay(1000);
				Move(2, 800, 10); //ID=2 80度へ
				HAL_Delay(50);
				//下
				Move(1, 1200, 10); //ID=1下 0度へ
			}
			break;

		case BB: // 吸引モーターON/OFF(1回押しでトグル)
			if (pressedEdge) {
				isfun = !isfun;
			}
			break;
		}

		prevButtonState = buttonState;
		/*
		 servo_On();
		 // Move(2, 20, 100); //ID=2 62度へ
		 Move(1, 0, 50); //ID=1 0度へ
		 HAL_Delay(1000);
		 */

		/*
		 motor_start_excitation();
		 //printf("%f,%f\n",Gvl,Gvr);
		 motor_start();

		 daikeiR(200,500,200,500,90,ROTATE_LEFT);
		 motor_stop();


		 HAL_Delay(1000);
		 */

		//daikeiR(100,300,100,500,90,ROTATE_LEFT);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// 現在どこまで受信したかを記憶する変数 (staticをつけることで値が保持される)
	static uint8_t rx_state = 0;
	static uint8_t temp_data = 0;

	if (huart->Instance == UART4) {

		switch (rx_state) {
		case 0: // 状態0: スタートバイト(0xAA)待ち
			if (rxByte == 0xAA) {
				rx_state = 1; // 0xAAが来たら次の状態へ
			}
			break;

		case 1: // 状態1: データ本体の受信
			temp_data = rxByte; // データを一時保存
			rx_state = 2;       // 次の状態へ
			break;

		case 2: // 状態2: エンドバイト(0x55)待ち
			if (rxByte == 0x55) {
				// 正しいフォーマットで受け取れた場合のみ本番の変数に反映
				buttonState = temp_data;
			}
			rx_state = 0; // 状態をリセットして、再びスタートバイト待ちに戻る
			break;
		}

		//buttonState = rxByte;

		// 次の1バイトの受信割り込みを再開
		HAL_UART_Receive_IT(&huart4, &rxByte, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == UART4) {
		// エラーフラグを強制的にクリア
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);

		// 受信を強制的に再スタート (rxByteは受信用の1バイト変数)
		extern uint8_t rxByte;
		HAL_UART_Receive_IT(&huart4, &rxByte, 1);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
