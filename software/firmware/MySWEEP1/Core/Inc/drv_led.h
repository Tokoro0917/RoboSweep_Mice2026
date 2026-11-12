/* drv_led.h  基板の LED（LED_1 = PA15、LED_2 = PC10、LED_T1 = PB8） */
#ifndef INC_DRV_LED_H_
#define INC_DRV_LED_H_

typedef enum { LED_1 = 0, LED_2 = 1, LED_T1 = 2 } led_t;

void led_set(led_t led, int on);

#endif /* INC_DRV_LED_H_ */
