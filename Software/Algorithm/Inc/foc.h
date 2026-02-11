#ifndef FOC_H
#define FOC_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32g4xx_hal.h"

/* FOC 转换类型 */
typedef struct {
  float a;
  float b;
  float c;
} abc_Typedef;
typedef struct {
  float Alpha;
  float Beta;
} AlphaBeta_Typedef;
typedef struct {
  float q;
  float d;
} dq_Typedef;

/* 控制模式 */
typedef enum {
  FOC_OpenLoopMode,
  FOC_EncoderOpenLoopMode,
} FOC_ControlState;

/* 电机基本参数 */
typedef struct {
  float powerVol;
  float powerVol_half;

  dq_Typedef Udq;
  abc_Typedef Uabc;
  AlphaBeta_Typedef UAlphaBeta;

  uint8_t pole_pairs;            // 极对数
  float angle_mechanical;        // 机械角度
  float angle_electrical;        // 电角度
  float angle_electrical_offset; // 电角度零偏
} FOC_MotorParam;

/* PWM 参数 */
typedef struct {
  TIM_HandleTypeDef *tim;
  uint32_t period;
} FOC_PWM;

typedef struct {
  TIM_HandleTypeDef *tim;
  float powerVol;
  uint8_t pole_pairs;
} FOC_InitTypedef;

typedef struct {
  FOC_ControlState state;
  FOC_MotorParam param;
  FOC_PWM pwm;
} FOC_Instance;

FOC_Instance *FOC_Register(FOC_InitTypedef *init);
void FOC_Init(FOC_Instance *instance,float angle_electrical_offset);
void FOC_OpenLoop(FOC_Instance *instance, float Ud, float Uq,
                  float delta_theta);
void FOC_EncoderOpenLoop(FOC_Instance *instance, float Ud, float Uq,
                         float angle);

#ifdef __cplusplus
}
#endif
#endif
