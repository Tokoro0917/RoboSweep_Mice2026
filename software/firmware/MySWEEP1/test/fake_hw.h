#pragma once
#include <stdint.h>
#include "drv_stepper.h"
#include "drv_ir.h"
extern double fake_wheel_mm[STEPPER_COUNT];
extern float fake_wheel_speed[STEPPER_COUNT];
extern float fake_max_wheel_speed;
extern int32_t fake_ir[IR_COUNT];
void fake_advance_1ms(void);
