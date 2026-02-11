#ifndef AS5047P_H
#define AS5047P_H
#include "stdint.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>

/* AS5047 寄存器地址 */
#define NOP 0x0000      // 无操作
#define ERRFL 0x0001    // 错误寄存器
#define PROG 0x0003     // 编程寄存器
#define DIAAGC 0x3FFC   // 诊断和 AGC （自动增益控制）
#define MAG 0x3FFD      // CORDIC 幅度
#define ANGLEUNC 0x3FFE // 读取角度值，无动态角度误差补偿
#define ANGLECOM 0x3FFF // 读取角度值，有动态角度误差补偿

/* AS5047 角度转换 */
#define AS5047P_RAW_TO_RAD 0.0003834952
#define AS5047P_RAW_TO_DEG 0.021972656

typedef struct {
  float alpha;
  float measure;
} AS5047P_Lowpass;

typedef struct {
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;

  AS5047P_Lowpass lowpass;
  float angle;
} AS5047P_Instance;

AS5047P_Instance *AS5047P_Register(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port,
                                  uint16_t cs_pin, float lowpass_alpha);
uint16_t AS5047P_Read(AS5047P_Instance *instance, uint16_t addr);
float AS5047P_ReadAngle(AS5047P_Instance *instance);
#endif
