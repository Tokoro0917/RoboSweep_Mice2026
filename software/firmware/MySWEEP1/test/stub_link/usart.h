/* PC 上で drv_link.c をテストするための usart.h の代わり */
#pragma once
#include <stdint.h>
#define __disable_irq()
#define __enable_irq()
typedef enum { HAL_OK = 0, HAL_BUSY = 2 } HAL_StatusTypeDef;
#define HAL_UART_STATE_READY 0x20
typedef struct { uint32_t gState; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart4;
extern uint8_t *fake_rx_ptr;          /* HAL_UART_Receive_IT に渡されたバッファ */
extern uint32_t fake_tick;
extern uint8_t fake_tx[16];
extern int fake_tx_len;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *buf, uint16_t n);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, const uint8_t *buf, uint16_t n);
uint32_t HAL_GetTick(void);
#define __HAL_UART_CLEAR_OREFLAG(h)
#define __HAL_UART_CLEAR_NEFLAG(h)
#define __HAL_UART_CLEAR_FEFLAG(h)
