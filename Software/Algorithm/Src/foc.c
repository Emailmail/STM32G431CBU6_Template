#include "foc.h"
#include "arm_math.h"
// #include "math.h"
#include "stdlib.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include "string.h"

#define SQRT3 1.73205080757f    // √3
#define _2PI 6.283185307179586f // 2*PI

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
           (SQRT3 / 2.0f) * AlphaBeta->Beta; // b = (-α + √3 * β) / 2
  abc->c = -0.5f * AlphaBeta->Alpha -
           (SQRT3 / 2.0f) * AlphaBeta->Beta; // c = (-α - √3 * β) / 2
}

/**
 * @brief 角度限制，限制在 [-PI, PI) 内
 * @param theta 角度
 */
static void AngleLimit(float *theta) {
  while (*theta >= PI) {
    *theta -= 2 * PI;
  }
  while (*theta < -PI) {
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
/* ---------------- 驱动函数  End  ---------------- */
/* ---------------- 用户函数 Begin ---------------- */
/**
 * @brief 注册 FOC实例
 * @param powerVol 电源（稳压后的）电压
 * @param step 每次自增的角度值
 * @return FOC实例
 */
FOC_Instance *FOC_Register(FOC_InitTypedef *init) {
  if (init->powerVol <= 0 || init->tim == NULL)
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

  return instance;
}

/**
 * @brief FOC 初始化
 */
void FOC_Init(FOC_Instance *instance) {
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
 * @brief FOC 开环控制
 * @param instance FOC实例
 */
void FOC_OpenLoop(FOC_Instance *instance, float Ud, float Uq,
                  float delta_theta) {
  /* 参数传递 */
  instance->param.Udq.d = Ud;
  instance->param.Udq.q = Uq;
  instance->param.angle += delta_theta;
  AngleLimit(&instance->param.angle);

  /* 通过 Park 逆变换和 Clarke 逆变换，将 Uqd 转换为 Uabc */
  InPark(&instance->param.Udq, &instance->param.UAlphaBeta, instance->param.angle);
  InClarke(&instance->param.UAlphaBeta, &instance->param.Uabc);

  /* 根据计算得到的 Uabc 值，设置 SPWM */
  FOC_SetSPWM(instance);
}

/* ---------------- 用户函数  End  ---------------- */
