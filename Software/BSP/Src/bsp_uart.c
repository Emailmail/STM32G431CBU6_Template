#include "bsp_uart.h"
#include "stdlib.h"
#include "stm32g4xx_hal_uart_ex.h"
#include "string.h"

/* UART 所有实例信息 */
static uint8_t idx; // 已注册的 UART 实例数量
static UART_Instance *usart_instance[DEVICE_UART_CNT] = {
    NULL}; // UART 实例指针数组

/**
 * @brief UART 实例注册
 * @param device_instance 设备实例指针（挂载到这个串口上的设备）
 * @param init_config UART 初始化配置
 * @retval UART 实例指针
 */
UART_Instance *UART_Register(void *device_instance,
                             UART_Init_Config_s *init_config) {
  /* 检测挂载到 UART 的设备是否存在 */
  if (device_instance == NULL || init_config == NULL)
    return NULL;

  /* 检测是 UART 否超过数量 */
  if (idx >= DEVICE_UART_CNT)
    return NULL;

  /* 检查此 UART 是否已被注册 */
  for (uint8_t i = 0; i < idx; i++)
    if (usart_instance[i]->uart_handle == init_config->uart_handle)
      return NULL;

  /* 分配内存 */
  UART_Instance *instance = (UART_Instance *)malloc(sizeof(UART_Instance));
  if (instance == NULL)
    return NULL;
  memset(instance, 0, sizeof(UART_Instance));

  /* 参数传递 */
  instance->device_instance = device_instance;
  instance->uart_handle = init_config->uart_handle;
  instance->module_callback = init_config->module_callback;

  /* 注册 UART ，且 idx 自增 */
  usart_instance[idx++] = instance;

  /* 启动 UART 接收服务 */
  UART_Service_Init(instance);

  /* 返回指针 */
  return instance;
}

/**
 * @brief 启动 UART 服务（每个 UART 注册之后自动启用）
 * @param uart_instance UART 实例
 * @retval 成功：HAL_OK；失败：HAL_ERROR
 */
HAL_StatusTypeDef UART_Service_Init(UART_Instance *uart_instance) {
  /* 停止 UART 的所有传输 */
  HAL_UART_DMAStop(uart_instance->uart_handle);

  /* 清除 DMA 半中断标志 */
  __HAL_DMA_CLEAR_FLAG(
      uart_instance->uart_handle->hdmarx,
      __HAL_DMA_GET_TC_FLAG_INDEX(uart_instance->uart_handle->hdmarx));

  /* 开启 DMA 空闲接收中断 */
  HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(
      uart_instance->uart_handle, uart_instance->recv_buff, UART_RXBUFF_LIMIT);

  /* 关闭 DMA 半传输中断 */
  __HAL_DMA_DISABLE_IT(uart_instance->uart_handle->hdmarx, DMA_IT_HT);

  return ret;
}

/**
 * @brief UART 发送
 * @param uart_instance UART 实例句柄
 * @param send_buf 发送缓冲区
 * @param send_size 发送长度
 * @param mode 发送模式
 * @retval 发送结果,具体意义参考HAL_StatusTypeDef的定义
 */
HAL_StatusTypeDef UART_Send(UART_Instance *uart_instance, uint8_t *send_buf,
                            uint16_t send_size, UART_TRANSFER_MODE mode) {
  switch (mode) {
  case UART_TRANSFER_BLOCKING:
    return HAL_UART_Transmit(uart_instance->uart_handle, send_buf, send_size,
                             100);
    break;
  case UART_TRANSFER_IT:
    return HAL_UART_Transmit_IT(uart_instance->uart_handle, send_buf,
                                send_size);
    break;
  case UART_TRANSFER_DMA:
    return HAL_UART_Transmit_DMA(uart_instance->uart_handle, send_buf,
                                 send_size);
    break;
  default:
    return HAL_ERROR;
    break;
  }
}

/**
 * @brief 查看该 UART 是否空闲(防止因为短时间内重复调用发送导致丢包)
 * @param uart_instance 串口实例
 * @retval 1：UART 空闲；0：UART 忙
 */
uint8_t UART_IsReady(UART_Instance *uart_instance) {
  if (uart_instance->uart_handle->gState & HAL_UART_STATE_BUSY_TX)
    return 0;
  else
    return 1;
}

/**
 * @brief 每次 DMA/IDLE 中断发生时，都会调用此函数，对于每个 UART
 * 实例会调用对应的回调进行进一步的处理
 * @param huart 发生中断的 UART
 * @param Size 此次接收到的总数据量
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  /* 查找是哪个串口触发的中断 */
  for (uint8_t i = 0; i < idx; ++i) {
    if (huart == usart_instance[i]->uart_handle) {

      /* 如果定义了回调函数,就调用 */
      if (usart_instance[i]->module_callback != NULL) {
        usart_instance[i]->module_callback(usart_instance[i]->device_instance,
                                           Size);
      }
      memset(usart_instance[i]->recv_buff, 0, UART_RXBUFF_LIMIT);

      /* 重启接收 */
      HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->uart_handle,
                                   usart_instance[i]->recv_buff,
                                   UART_RXBUFF_LIMIT);

      /* 关闭半中断 */
      __HAL_DMA_DISABLE_IT(usart_instance[i]->uart_handle->hdmarx, DMA_IT_HT);
      return;
    }
  }
}

/**
 * @brief 当 UART
 * 发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 * @param huart 发生错误的 UART
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  for (uint8_t i = 0; i < idx; ++i) {
    if (huart == usart_instance[i]->uart_handle) {
      /* 完全重启接收 */
      UART_Service_Init(usart_instance[i]);

      return;
    }
  }
}