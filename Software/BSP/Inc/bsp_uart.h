#ifndef BSP_UART_H
#define BSP_UART_H
#ifdef __cplusplus
extern "C" {
#endif
/* ------------------------------------------------- include
 * ------------------------------------------------- */
#include "stdint.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_uart.h"

/* ------------------------------------------------- define
 * ------------------------------------------------- */
#define DEVICE_UART_CNT 5    // 最大串口数量
#define UART_RXBUFF_LIMIT 64 // 串口接收缓冲区大小

/* ------------------------------------------------- typedef
 * ------------------------------------------------- */
/* 函数指针 */
typedef void (*uart_device_callback)(
    void *device_instance, uint16_t size); // 模块回调函数,用于解析协议

/* 发送模式枚举 */
typedef enum {
  UART_TRANSFER_BLOCKING, // 阻塞发送
  UART_TRANSFER_IT,       // 中断发送
  UART_TRANSFER_DMA,      // DMA发送
} UART_TRANSFER_MODE;

/* 串口实例结构体,每个module有且仅有一个实例 */
typedef struct {
  uint8_t recv_buff[UART_RXBUFF_LIMIT]; // 接收缓冲区
  UART_HandleTypeDef *uart_handle;      // 实例对应的uart_handle
  uart_device_callback module_callback; // 解析收到的数据的回调函数
  void *device_instance;                // 挂载到这个串口上的设备
} UART_Instance;

/* usart 初始化配置结构体 */
typedef struct {
  UART_HandleTypeDef *uart_handle;      // 实例对应的uart_handle
  uart_device_callback module_callback; // 解析收到的数据的回调函数
} UART_Init_Config_s;

/* ------------------------------------------------- functions
 * ------------------------------------------------- */
/**
 * @brief 注册一个串口实例,返回一个串口实例指针
 * @param device_instance 设备实例指针（挂载到这个串口上的设备）
 * @param init_config 传入串口初始化结构体
 */
UART_Instance *UART_Register(void *device_instance,
                             UART_Init_Config_s *init_config);

/**
 * @brief 启动串口服务,需要传入一个usart实例.一般用于lost callback的情况
 * @param uart_instance
 */
HAL_StatusTypeDef UART_Service_Init(UART_Instance *uart_instance);

/**
 * @brief
 * 通过调用该函数可以发送一帧数据,需要传入一个usart实例,发送buff以及这一帧的长度
 * @note
 * 在短时间内连续调用此接口,若采用IT/DMA会导致上一次的发送未完成而新的发送取消.
 * @note
 * 若希望连续使用DMA/IT进行发送,请配合UARTisReady()使用,或自行为你的module实现一个发送队列和任务.
 * @param uart_instance 串口实例
 * @param send_buf 待发送数据的buffer
 * @param send_size how many bytes to send
 */
HAL_StatusTypeDef UART_Send(UART_Instance *uart_instance, uint8_t *send_buf,
                            uint16_t send_size, UART_TRANSFER_MODE mode);

/**
 * @brief 判断串口是否准备好,用于连续或异步的IT/DMA发送
 * @param uart_instance 要判断的串口实例
 * @retval 1 空闲
 *         0 忙碌
 */
uint8_t UART_IsReady(UART_Instance *uart_instance);

#ifdef __cplusplus
}
#endif
#endif
