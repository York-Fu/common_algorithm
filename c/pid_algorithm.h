#ifndef _pid_algorithm_h_
#define _pid_algorithm_h_

#include <stdio.h>
#include <math.h>

#define PIDDataType_t double_t

typedef struct
{
  double_t kp;
  double_t ki;
  double_t kd;
  PIDDataType_t ek;   // e(k)
  PIDDataType_t ek_1; // e(k-1)
  PIDDataType_t ek_2; // e(k-2)
  PIDDataType_t integral;
  PIDDataType_t uk; //u(k)
} PIDParam_t;

PIDDataType_t positionPIDController(PIDParam_t *pid, double_t err, double_t min_integral, double_t max_integral);
PIDDataType_t incrementPIDController(PIDParam_t *pid, double_t err, double_t min_uk, double_t max_uk);

#endif
