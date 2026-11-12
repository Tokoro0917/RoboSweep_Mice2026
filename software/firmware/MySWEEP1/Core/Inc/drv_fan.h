/* drv_fan.h  吸引ファン（TIM4_CH4 = PB9） */
#ifndef INC_DRV_FAN_H_
#define INC_DRV_FAN_H_

void fan_init(void);
void fan_set(int on);
int fan_is_on(void);

#endif /* INC_DRV_FAN_H_ */
