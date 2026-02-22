#include "foc.h"
#include "arm_math.h"
// #include "math.h"
#include "stdlib.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include "string.h"
#include <stdint.h>

#define SQRT3 1.732050807568877f      // √3
#define SQRT3_DIV2 0.866025403784438f // √3 / 2
#define _2PI 6.283185307179586f       // 2 * PI

/* ---------------- 驱动函数 Begin ---------------- */
/**
 * @brief Park逆变换
 * @param instance FOC实例
 * @param theta 电角度
 */
static void InPark(dq_Typedef *dq, AlphaBeta_Typedef *AlphaBeta, float theta) {
  float cos_theta = arm_cos_f32(theta);
  float sin_theta = arm_sin_f32(theta);

  AlphaBeta->Alpha =
      dq->d * cos_theta - dq->q * sin_theta; // α = d  *cosθ - q * sinθ
  AlphaBeta->Beta =
      dq->d * sin_theta + dq->q * cos_theta; // β = d  *sinθ + q * cosθ
}

/**
 * @brief Clarke逆变换
 * @param instance FOC实例
 */
static void InClarke(AlphaBeta_Typedef *AlphaBeta, abc_Typedef *abc) {
  abc->a = AlphaBeta->Alpha; // a = α;
  abc->b = -0.5f * AlphaBeta->Alpha +
           SQRT3_DIV2 * AlphaBeta->Beta; // b = (-α + √3 * β) / 2
  abc->c = -0.5f * AlphaBeta->Alpha -
           SQRT3_DIV2 * AlphaBeta->Beta; // c = (-α - √3 * β) / 2
}

/**
 * @brief 角度限制，限制在 [-PI, PI) 内
 * @param theta 角度
 */
static void AngleLimit(float *theta) {
  while (*theta >= 2 * PI) {
    *theta -= 2 * PI;
  }
  while (*theta < 0) {
    *theta += 2 * PI;
  }
}

static void FOC_SetSPWM(FOC_Instance *instance) {
  /* 计算 CCR */
  uint32_t aCCR =
      (uint32_t)((instance->param.Uabc.a + instance->param.powerVol_half) /
                 instance->param.powerVol * instance->pwm.period);
  uint32_t bCCR =
      (uint32_t)((instance->param.Uabc.b + instance->param.powerVol_half) /
                 instance->param.powerVol * instance->pwm.period);
  uint32_t cCCR =
      (uint32_t)((instance->param.Uabc.c + instance->param.powerVol_half) /
                 instance->param.powerVol * instance->pwm.period);

  /* 设置 CCR */
  instance->pwm.tim->Instance->CCR1 = aCCR;
  instance->pwm.tim->Instance->CCR2 = bCCR;
  instance->pwm.tim->Instance->CCR3 = cCCR;
}
/* -------- SVPWM Begin -------- */
/**
 * @brief 扇区判断
 * @param AlphaBeta AlphaBeta轴上的电压
 * @return 所在的扇区（1 ~ 6）
 */
static uint8_t SecJud(AlphaBeta_Typedef AlphaBeta) {
  float A = AlphaBeta.Beta;
  float B = SQRT3 * AlphaBeta.Alpha - AlphaBeta.Beta;
  float C = -SQRT3 * AlphaBeta.Alpha - AlphaBeta.Beta;

  uint8_t sum = 0;
  uint8_t sector = 0;

  if (A > 0)
    sum += 1;
  if (B > 0)
    sum += 2;
  if (C > 0)
    sum += 4;

  switch (sum) {
  case 3:
    sector = 1;
    break;
  case 1:
    sector = 2;
    break;
  case 5:
    sector = 3;
    break;
  case 4:
    sector = 4;
    break;
  case 6:
    sector = 5;
    break;
  case 2:
    sector = 6;
    break;
  }

  return sector;
}

static void SetSVPWM(FOC_Instance *instance) {
  uint8_t sector = SecJud(instance->param.UAlphaBeta);

  float tmp = (float)instance->pwm.period * SQRT3 / instance->param.powerVol;
  float X = tmp * (instance->param.UAlphaBeta.Beta);
  float Y = tmp * (instance->param.UAlphaBeta.Alpha * SQRT3_DIV2 +
                   instance->param.UAlphaBeta.Beta * 0.5f);
  float Z = tmp * (-instance->param.UAlphaBeta.Alpha * SQRT3_DIV2 +
                   instance->param.UAlphaBeta.Beta * 0.5f);

  float T1, T2, T0;
  switch (sector) {
  case 1:
    T1 = -Z;
    T2 = X;
    break;
  case 2:
    T1 = Z;
    T2 = Y;
    break;
  case 3:
    T1 = X;
    T2 = -Y;
    break;
  case 4:
    T1 = -X;
    T2 = Z;
    break;
  case 5:
    T1 = -Y;
    T2 = -Z;
    break;
  case 6:
    T1 = Y;
    T2 = -X;
    break;
  }
  if (T1 + T2 > instance->pwm.period) { // 防止 T1，T2 超过周期
    T1 = instance->pwm.period * T1 / (T1 + T2);
    T2 = instance->pwm.period * T2 / (T1 + T2);
  }
  T0 = (float)instance->pwm.period - T1 - T2;

  abc_Typedef Tabc;
  Tabc.a = T0 / 4.0f;
  Tabc.b = Tabc.a + T1 / 2.0f;
  Tabc.c = Tabc.b + T2 / 2.0f;

  uint32_t aCCR, bCCR, cCCR;
  switch (sector) {
  case 1:
    aCCR = (uint32_t)Tabc.a;
    bCCR = (uint32_t)Tabc.b;
    cCCR = (uint32_t)Tabc.c;
    break;
  case 2:
    aCCR = (uint32_t)Tabc.b;
    bCCR = (uint32_t)Tabc.a;
    cCCR = (uint32_t)Tabc.c;
    break;
  case 3:
    aCCR = (uint32_t)Tabc.c;
    bCCR = (uint32_t)Tabc.a;
    cCCR = (uint32_t)Tabc.b;
    break;
  case 4:
    aCCR = (uint32_t)Tabc.c;
    bCCR = (uint32_t)Tabc.b;
    cCCR = (uint32_t)Tabc.a;
    break;
  case 5:
    aCCR = (uint32_t)Tabc.b;
    bCCR = (uint32_t)Tabc.c;
    cCCR = (uint32_t)Tabc.a;

    break;
  case 6:
    aCCR = (uint32_t)Tabc.a;
    bCCR = (uint32_t)Tabc.c;
    cCCR = (uint32_t)Tabc.b;
    break;
  }

  instance->pwm.tim->Instance->CCR1 = aCCR;
  instance->pwm.tim->Instance->CCR2 = bCCR;
  instance->pwm.tim->Instance->CCR3 = cCCR;
}
/* -------- SVPWM  End  -------- */
/* ---------------- 驱动函数  End  ---------------- */
/* ---------------- 用户函数 Begin ---------------- */
/**
 * @brief 注册 FOC实例
 * @param powerVol 电源（稳压后的）电压
 * @param step 每次自增的角度值
 * @return FOC实例
 */
FOC_Instance *FOC_Register(FOC_InitTypedef *init) {
  if (init->powerVol <= 0 || init->tim == NULL || init->pole_pairs == 0)
    return NULL;

  FOC_Instance *instance = (FOC_Instance *)malloc(sizeof(FOC_Instance));
  if (instance == NULL)
    return NULL;
  memset(instance, 0, sizeof(FOC_Instance));

  instance->state = FOC_OpenLoopMode;
  instance->pwm.tim = init->tim;
  instance->pwm.period = (init->tim->Init.Period + 1);
  instance->param.powerVol = init->powerVol;
  instance->param.powerVol_half = init->powerVol / 2.0f;
  instance->param.pole_pairs = init->pole_pairs;

  return instance;
}

/**
 * @brief FOC 初始化
 */
void FOC_Init(FOC_Instance *instance, float angle_electrical_offset) {
  instance->param.angle_electrical_offset = angle_electrical_offset;
  HAL_TIM_Base_Start_IT(instance->pwm.tim);
  instance->pwm.tim->Instance->CCR1 = 0;
  instance->pwm.tim->Instance->CCR2 = 0;
  instance->pwm.tim->Instance->CCR3 = 0;
  HAL_TIM_PWM_Start(instance->pwm.tim, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(instance->pwm.tim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(instance->pwm.tim, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(instance->pwm.tim, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(instance->pwm.tim, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(instance->pwm.tim, TIM_CHANNEL_3);
}

/**
 * @brief FOC 设置模式
 */
void FOC_SetMode(FOC_Instance *instance, FOC_ControlState mode) {
  instance->state = mode;
}

/**
 * @brief FOC 开环控制
 * @param instance FOC实例
 * @param Ud 直轴上的电压
 * @param Uq 交轴上的电压
 * @param angle 电角度
 */
void FOC_OpenLoop(FOC_Instance *instance, float Ud, float Uq, float angle) {
  /* 参数传递 */
  instance->param.Udq.d = Ud;
  instance->param.Udq.q = Uq;
  instance->param.angle_electrical = angle;
  AngleLimit(&instance->param.angle_electrical);

  /* 通过 Park 逆变换和 Clarke 逆变换，将 Uqd 转换为 Uabc */
  InPark(&instance->param.Udq, &instance->param.UAlphaBeta,
         instance->param.angle_electrical);
  InClarke(&instance->param.UAlphaBeta, &instance->param.Uabc);

  /* 根据计算得到的 Uabc 值，设置 SPWM */
  FOC_SetSPWM(instance);
}

/**
 * @brief FOC 带编码器的开环控制
 * @param instance FOC实例
 * @param Ud 直轴上的电压
 * @param Uq 交轴上的电压
 * @param angle 机械角度（编码器测得）
 */
void FOC_EncoderOpenLoop(FOC_Instance *instance, float Ud, float Uq,
                         float angle) {
  /* 参数传递 */
  instance->param.Udq.d = Ud;
  instance->param.Udq.q = Uq;
  instance->param.angle_mechanical = angle;
  instance->param.angle_electrical =
      (float)(angle * instance->param.pole_pairs) -
      instance->param.angle_electrical_offset;
  AngleLimit(&instance->param.angle_mechanical);
  AngleLimit(&instance->param.angle_electrical);

  /* 通过 Park 逆变换和 Clarke 逆变换，将 Uqd 转换为 Uabc */
  InPark(&instance->param.Udq, &instance->param.UAlphaBeta,
         instance->param.angle_electrical);
  InClarke(&instance->param.UAlphaBeta, &instance->param.Uabc);

  /* 根据计算得到的 Uabc 值，设置 SPWM */
  FOC_SetSPWM(instance);
}

/* ---------------- 用户函数  End  ---------------- */
