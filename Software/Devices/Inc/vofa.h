#ifndef VOFA_H
#define VOFA_H
#ifdef __cplusplus
extern "C" {
#endif
#include "bsp_uart.h"

#define VOFA_RX_HEAD 0x0A
#define VOFA_RX_TAIL 0x0B
#define VOFA_RX_NUM 8

typedef struct {
  UART_Instance *uart;
  float RxVar[VOFA_RX_NUM];
} VOFA_Instance;

VOFA_Instance *VOFA_Register(UART_HandleTypeDef *huart);
uint8_t VOFA_Send(VOFA_Instance *instance, float *send_buf,
                  uint16_t send_size);
#ifdef __cplusplus
}
#endif
#endif
