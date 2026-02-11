#include "as5047.h"
#include "arm_math.h"
#include "stdint.h"
// #include "stm32g431xx.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"
#include "stdlib.h"
#include "string.h"

#define AS5047P_OUTPUT_FORMAT 0 // 0：输出弧度值；1：输出角度值

#if AS5047P_OUTPUT_FORMAT
/**
 * @brief 将角度差限制在 [-180, 180] 内
 */
static float DegErr_Limit(float angle) {
  if (angle >= 180.0f)
    angle -= 360.0f;
  if (angle < -180.0f)
    angle += 360.0f;
  return angle;
}

/**
 * @brief 将角度差限制在 [-180, 180] 内
 */
static float Deg_Limit(float angle) {
  if (angle >= 360.0f)
    angle -= 360.0f;
  if (angle < 0.0f)
    angle += 360.0f;
  return angle;
}
#else
/**
 * @brief 将弧度差限制在 [-PI, PI) 内
 */
static float RadErr_Limit(float angle) {
  if (angle >= PI)
    angle -= 2 * PI;
  else if (angle < -PI)
    angle += 2 * PI;
  return angle;
}

/**
 * @brief 将弧度限制在 [0, 2PI) 内
 */
static float Rad_Limit(float angle) {
  if (angle >= 2 * PI)
    angle -= 2 * PI;
  if (angle < 0.0f)
    angle += 2 * PI;
  return angle;
}
#endif

/**
 * @brief 返回奇偶校验值
 */
static uint16_t Parity_bit_Calculate(uint16_t data) {
  uint16_t sum = 0;
  while (data != 0) {
    sum ^= data; // 异或运算，相同的为0，不同为1
    data >>= 1;  // 循环右移一位，直至数据为0（while的结束条件）
  }
  return (sum & 0x01); // 返回 sum 的最低位，即为奇偶校验值
}

/**
 * @brief AS5047 SPI 读写一个字节
 * @param cs_port CS 的 GPIO 端口
 * @param cs_pin CS 的 GPIO 引脚号
 * @param tx_data 要发送的数据
 */
static uint16_t AS5047P_ReadWriteByte(SPI_HandleTypeDef *spi,
                                     GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     uint16_t txdata) {
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET); // 拉低 CS
  uint16_t rxdata;
  if (HAL_SPI_TransmitReceive(spi, (uint8_t *)&txdata, (uint8_t *)&rxdata, 1,
                              HAL_MAX_DELAY) != HAL_OK) // 读写数据
    rxdata = 0;
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET); // 拉高 CS
  return rxdata;                                    // 返回接收到的数据
}

/**
 * @brief 注册 AS5047
 * @param spi SPI 句柄
 * @param cs_port CS 的 GPIO 端口
 * @param cs_pin CS 的 GPIO 引脚号
 * @re
 */
AS5047P_Instance *AS5047P_Register(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port,
                                  uint16_t cs_pin, float lowpass_alpha) {
  if (spi == NULL || cs_port == NULL || cs_pin == 0)
    return NULL;

  AS5047P_Instance *instance = (AS5047P_Instance *)malloc(sizeof(AS5047P_Instance));
  if (instance == NULL) 
    return NULL;
  memset(instance, 0, sizeof(AS5047P_Instance));

  instance->spi = spi;
  instance->cs_port = cs_port;
  instance->cs_pin = cs_pin;
  instance->angle = 0.0f;
  instance->lowpass.alpha = lowpass_alpha;

  return instance;
}

/**
 * @brief 读取 AS5047
 * @param instance AS5047 实例指针
 * @param addr 寄存器地址
 */
uint16_t AS5047P_Read(AS5047P_Instance *instance, uint16_t addr) {
  uint16_t data;
  addr |= 0x4000; // 将 bit14 置为1，表示读操作
  if (Parity_bit_Calculate(addr) == 1)
    addr = addr | 0x8000; // 如果前15位1的个数位奇数，则 Bit15 置1，将其设为偶数
  AS5047P_ReadWriteByte(instance->spi, instance->cs_port, instance->cs_pin,
                       addr); // 发送一条指令，不管读回的数据
  data = AS5047P_ReadWriteByte(
      instance->spi, instance->cs_port, instance->cs_pin,
      NOP | 0x4000); // 发送一条空指令，读取上一次指令返回的数据。
  data &= 0x3fff;
  return data;
}

/**
 * @brief 读取 AS5047 角度
 * @param instance AS5047 实例指针
 * @return 角度
 */
float AS5047P_ReadAngle(AS5047P_Instance *instance) {
  uint16_t data = AS5047P_Read(instance, ANGLECOM);
#if AS5047P_OUTPUT_FORMAT
  instance->lowpass.measure = data * AS5047P_RAW_TO_DEG;  // 记录这次的测量值
  instance->angle = instance->angle + DegErr_Limit(instance->lowpass.measure - instance->angle) * instance->lowpass.alpha;
  instance->angle = Deg_Limit(instance->angle);
#else
  instance->lowpass.measure = data * AS5047P_RAW_TO_RAD;  // 记录这次的测量值
  instance->angle = instance->angle + RadErr_Limit(instance->lowpass.measure - instance->angle) * instance->lowpass.alpha;
  instance->angle = Rad_Limit(instance->angle);
#endif
  return instance->angle;
}
