#include "drv_time.h"

#include "main.h"

void time_init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t time_ms(void) {
	return HAL_GetTick();
}

void time_delay_us(uint32_t us) {
	uint32_t start = DWT->CYCCNT;
	uint32_t cycles = us * (SystemCoreClock / 1000000U);
	while ((DWT->CYCCNT - start) < cycles) {
	}
}
