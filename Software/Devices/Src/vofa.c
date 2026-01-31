#include "vofa.h"
#include "bsp_uart.h"
#include "stdlib.h"
#include "string.h"

uint8_t vofa_tx_head[4] = {0x00, 0x00, 0x80, 0x7F}; // VOFA 发送数据帧头

/* 数据包格式为 | 0x0A | 序号（1字节） | 数据（4字节） | 0x0B | */
static void VOFA_AnalyzeRxData(VOFA_Instance *instance, uint16_t size) {
  /* 检验数据长度 */
  if (size != 7)
    return;

  /* 检验帧头、帧尾 */
  if (instance->uart->recv_buff[0] != VOFA_RX_HEAD ||
      instance->uart->recv_buff[6] != VOFA_RX_TAIL)
    return;

  /* 检验序号是否超出范围 */
  uint16_t id = instance->uart->recv_buff[1];
  if (id >= VOFA_RX_NUM)
    return;

  /* 将数据存入数组内 */
  for (uint16_t i = 0; i < 4; i++) {
    ((uint8_t *)&instance->RxVar[id])[i] = instance->uart->recv_buff[i + 2];
  }
}

/**
 * @brief VOFA 接收回调函数
 * @param device_instance VOFA 设备实例指针
 * @param size 接收到的数据长度
 */
void VOFA_RxCallback(void *device_instance, uint16_t size) {
  VOFA_Instance *instance = (VOFA_Instance *)device_instance;
  VOFA_AnalyzeRxData(instance, size);
}

/**
 * @brief 注册一个 VOFA 实例
 * @param huart 绑定的 UART 句柄
 * @retval VOFA 实例指针
 */
VOFA_Instance *VOFA_Register(UART_HandleTypeDef *huart) {
  /* 检验空指针 */
  if (huart == NULL)
    return NULL;

  /* 分配内存 */
  VOFA_Instance *instance = (VOFA_Instance *)malloc(sizeof(VOFA_Instance));
  if (instance == NULL)
    return NULL;

  /* 注册串口实例 */
  UART_Init_Config_s uart_config;
  uart_config.uart_handle = huart;
  uart_config.module_callback =
      (uart_device_callback)VOFA_RxCallback; // 设置回调函数

  instance->uart = UART_Register((void *)instance, &uart_config);
  if (instance->uart == NULL) {
    free(instance);
    return NULL;
  }

  /* 初始化变量 */
  memset(instance->RxVar, 0, sizeof(instance->RxVar));

  return instance;
}

/**
 * @brief VOFA 发送数据到上位机（用于打印波形）
 * @param instance VOFA 实例指针
 * @param send_buf 待发送数据缓冲区
 * @param send_size 待发送数据长度
 * @retval 0：发送成功；1：参数错误；2：发送帧头失败；3：发送数据失败
 */
uint8_t VOFA_Send(VOFA_Instance *instance, float *send_buf,
                  uint16_t send_size) {
  if (instance == NULL || send_buf == NULL || send_size == 0)
    return 1;

  if (UART_Send(instance->uart, vofa_tx_head, 4, UART_TRANSFER_BLOCKING) != HAL_OK)
    return 2;

  if (UART_Send(instance->uart, (uint8_t *)send_buf, send_size * 4, UART_TRANSFER_BLOCKING) != HAL_OK)
    return 3;

  return 0;
}
