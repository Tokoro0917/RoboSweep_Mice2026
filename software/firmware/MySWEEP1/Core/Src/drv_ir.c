#include "drv_ir.h"

#include "main.h"
#include "adc.h"
#include "config.h"
#include "drv_time.h"

#define ADC_CHANNELS 5   /* [0]=電池 [1..4]=SENSOR_1..4 */

typedef enum { PHASE_A = 0, PHASE_B = 1 } phase_t;

static uint16_t adc_buf[ADC_CHANNELS];
static volatile int busy = 0;
static volatile phase_t phase = PHASE_A;

static uint16_t lit[IR_COUNT];
static uint16_t dark[IR_COUNT];
static volatile int32_t value[IR_COUNT];
static volatile uint32_t updates = 0;
static volatile float battery = 0.0f;

static void leds(int led1, int led2) {
	HAL_GPIO_WritePin(SENSORLED_1_GPIO_Port, SENSORLED_1_Pin, led1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SENSORLED_2_GPIO_Port, SENSORLED_2_Pin, led2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ir_init(void) {
	leds(0, 0);
	busy = 0;
	phase = PHASE_A;
}

void ir_tick(void) {
	if (busy) {
		return;   /* 前の変換が終わっていない */
	}
	if (phase == PHASE_A) {
		leds(1, 0);
	} else {
		leds(0, 1);
	}
	time_delay_us(IR_SETTLE_US);
	busy = 1;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_CHANNELS);
}

void ir_on_adc_complete(void) {
	HAL_ADC_Stop_DMA(&hadc1);
	leds(0, 0);

	if (phase == PHASE_A) {
		lit[0] = adc_buf[1];
		lit[1] = adc_buf[2];
		dark[2] = adc_buf[3];
		dark[3] = adc_buf[4];
		phase = PHASE_B;
	} else {
		dark[0] = adc_buf[1];
		dark[1] = adc_buf[2];
		lit[2] = adc_buf[3];
		lit[3] = adc_buf[4];
		for (int i = 0; i < IR_COUNT; i++) {
			int32_t d = (int32_t)lit[i] - (int32_t)dark[i];
			value[i] = (d > 0) ? d : 0;
		}
		updates++;
		phase = PHASE_A;
	}
	battery = 3.3f * (float)adc_buf[0] / 4095.0f * BATTERY_DIVIDER;
	busy = 0;
}

int32_t ir_get(int index) {
	return value[index];
}

uint32_t ir_update_count(void) {
	return updates;
}

float battery_voltage(void) {
	return battery;
}
