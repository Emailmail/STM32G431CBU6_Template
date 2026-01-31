#include "foc.h"
#include "arm_math.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"

#define SQRT3 1.73205080757f    // √3
#define _2PI 6.283185307179586f // 2*PI

FOC_Instance *FOC_Register(void) {
  FOC_Instance *instance = (FOC_Instance *)malloc(sizeof(FOC_Instance));
  if (instance == NULL)
    return NULL;

  memset(instance, 0, sizeof(FOC_Instance));

  return instance;
}

/**
 * @brief 限制角度在0-2PI之间
 *
 * @param ElAngle 输入角度
 * @return float 返回角度
 */
float FOC_Limit_Angle(float ElAngle) {
  while (ElAngle > _2PI || ElAngle < 0) {
    if (ElAngle > _2PI) {
      ElAngle -= _2PI;
    } else if (ElAngle < 0) {
      ElAngle += _2PI;
    }
  }

  return ElAngle;
}

/**
 * @brief Clarke变换
 * @param instance FOC实例
 */
static void FOC_ClarkeTransform(FOC_Instance *instance) {
  instance->clarke.alpha = instance->current.Ia;
  instance->clarke.beta =
      (instance->current.Ia + 2.0f * instance->current.Ib) / SQRT3;
}

/**
 * @brief Park变换
 * @param instance FOC实例
 * @param theta 电角度
 */
static void FOC_ParkTransform(FOC_Instance *instance) {
  float theta = atan2f(instance->clarke.beta, instance->clarke.alpha);

  float cos_theta = arm_cos_f32(theta);
  float sin_theta = arm_sin_f32(theta);

  instance->park.d =
      instance->clarke.alpha * cos_theta + instance->clarke.beta * sin_theta;
  instance->park.q =
      -instance->clarke.alpha * sin_theta + instance->clarke.beta * cos_theta;
}

void FOC_Transform(FOC_Instance *instance) {
  FOC_ClarkeTransform(instance);
  FOC_ParkTransform(instance);
}
