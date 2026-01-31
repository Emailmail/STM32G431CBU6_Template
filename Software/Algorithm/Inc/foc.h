#ifndef FOC_H
#define FOC_H
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float Ia;
  float Ib;
  float Ic;
} FOC_Current;

typedef struct
{
  float alpha;
  float beta;
} FOC_Clarke;

typedef struct
{
  float q;
  float d;
} FOC_Park;

typedef struct {
  FOC_Current current;
  FOC_Clarke clarke;
  FOC_Park park;
} FOC_Instance;

FOC_Instance *FOC_Register(void);
void FOC_Transform(FOC_Instance *instance);

#ifdef __cplusplus
}
#endif
#endif
