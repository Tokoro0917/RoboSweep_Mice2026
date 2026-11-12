/*
 * wall_sensor.c
 *
 * 測り方は元のプログラム（PL_sensor.c）と同じ。
 *   段階 0: LED1 を点ける
 *   段階 1: センサ 0,1 の「点灯時」と 2,3 の「消灯時」を記録。LED1 を消して LED2 を点ける
 *   段階 2: センサ 0,1 の「消灯時」と 2,3 の「点灯時」を記録。LED を両方消す
 *   → 点灯時 - 消灯時 を新しい値にする
 *
 * ADC の並び: [0]=電池(PC5) [1]=SENSOR_1(PC3) [2]=SENSOR_2(PC2) [3]=SENSOR_3(PC1) [4]=SENSOR_4(PC0)
 */

#include "wall_sensor.h"

#include "main.h"
#include "adc.h"
#include "config.h"

#define ADC_CHANNELS 5

static uint16_t adc_buf[ADC_CHANNELS];
static volatile uint8_t step = 0;
static volatile uint8_t busy = 0;
static uint16_t value_on[WALL_SENSOR_COUNT];
static uint16_t value_off[WALL_SENSOR_COUNT];
static volatile int32_t value[WALL_SENSOR_COUNT];
static volatile float battery_v = 0.0f;

static void short_wait(int n) {
	for (volatile int i = 0; i < n; i++) {
	}
}

static void start_dma(void) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_CHANNELS);
}

void wall_sensor_start(void) {
	if (busy) {
		return;   /* 前の測定がまだ終わっていない */
	}
	busy = 1;
	step = 0;
	start_dma();
}

void wall_sensor_on_adc_complete(void) {
	HAL_ADC_Stop_DMA(&hadc1);

	switch (step) {
	case 0:
		HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin, GPIO_PIN_RESET);
		short_wait(500);   /* 光が安定するのを待つ */
		break;
	case 1:
		value_on[0] = adc_buf[1];
		value_on[1] = adc_buf[2];
		value_off[2] = adc_buf[3];
		value_off[3] = adc_buf[4];
		HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin, GPIO_PIN_SET);
		short_wait(500);
		break;
	case 2:
		value_off[0] = adc_buf[1];
		value_off[1] = adc_buf[2];
		value_on[2] = adc_buf[3];
		value_on[3] = adc_buf[4];
		HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin, GPIO_PIN_RESET);
		break;
	}

	battery_v = 3.3f * (float)adc_buf[0] / 4095.0f * BATTERY_DIVIDER;
	step++;

	if (step < 3) {
		start_dma();
	} else {
		for (int i = 0; i < WALL_SENSOR_COUNT; i++) {
			int32_t d = (int32_t)value_on[i] - (int32_t)value_off[i];
			value[i] = (d > 0) ? d : 0;
		}
		busy = 0;
	}
}

int32_t wall_sensor_get(int index) {
	return value[index];
}

float wall_sensor_battery(void) {
	return battery_v;
}

int wall_exists_left(void) {
	return wall_sensor_get(WALL_SENSOR_LEFT) >= WALL_EXIST_LEFT;
}

int wall_exists_right(void) {
	return wall_sensor_get(WALL_SENSOR_RIGHT) >= WALL_EXIST_RIGHT;
}

int wall_exists_front(void) {
	return wall_sensor_get(WALL_SENSOR_FRONT_A) >= WALL_EXIST_FRONT
			&& wall_sensor_get(WALL_SENSOR_FRONT_B) >= WALL_EXIST_FRONT;
}
