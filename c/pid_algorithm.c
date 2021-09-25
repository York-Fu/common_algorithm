#include "pid_algorithm.h"

#define LIMITING(v, min, max) ((v) > (max) ? (max) : ((v) < (min) ? (min) : (v)))

PIDDataType_t positionPIDController(PIDParam_t *pid, PIDDataType_t err, PIDDataType_t min_integral, PIDDataType_t max_integral)
{
  pid->ek = err;
  pid->integral += pid->ek;
  pid->integral = LIMITING(pid->integral, min_integral, max_integral);
  pid->uk = pid->kp * pid->ek + pid->ki * pid->integral + pid->kd * (pid->ek - pid->ek_1);
  pid->ek_1 = pid->ek;
  return pid->uk;
}

PIDDataType_t incrementPIDController(PIDParam_t *pid, PIDDataType_t err, PIDDataType_t min_uk, PIDDataType_t max_uk)
{
  pid->ek = err;
  pid->uk += pid->kp * (pid->ek - pid->ek_1) + pid->ki * pid->ek + pid->kd * (pid->ek - 2 * pid->ek_1 + pid->ek_2);
  pid->uk = LIMITING(pid->uk, min_uk, max_uk);
  pid->ek_2 = pid->ek_1;
  pid->ek_1 = pid->ek;
  return pid->uk;
}
