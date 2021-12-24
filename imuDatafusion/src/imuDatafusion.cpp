#include "imuDatafusion.h"

ImuDatafusion::ImuDatafusion(float kp, float ki, float halfT)
{
  _kp = 15.0f;                     // 比例增益支配率收敛到加速度计/磁强计
  _ki = 0.004f;                    // 积分增益支配率的陀螺仪偏见的衔接
  _halfT = 0.005f;                 // 采样周期的一半
  q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // 四元数的元素，代表估计方向
  exInt = 0, eyInt = 0, ezInt = 0; // 按比例缩小积分误差

  for (size_t i = 0; i < 3; i++)
  {
    _gyroOffset[i] = 0.0;
    _accOffset[i] = 0.0;
    _magOffset[i] = 0.0;
    _eulerAngle[i] = 0.0;
  }
}

ImuDatafusion::~ImuDatafusion()
{
}

void ImuDatafusion::setFusionCoeff(float kp, float ki, float halfT)
{
  _kp = kp;
  _ki = ki;
  _halfT = halfT;
}

void ImuDatafusion::calibrateGyro(double gx, double gy, double gz)
{
  _gyroOffset[0] = gx;
  _gyroOffset[1] = gy;
  _gyroOffset[2] = gz;
}

void ImuDatafusion::calibrateAcc(double ax, double ay, double az)
{
  _accOffset[0] = ax;
  _accOffset[0] = ay;
  _accOffset[0] = az;
}

void ImuDatafusion::calibrateMag(double mx, double my, double mz)
{
  _magOffset[0] = mx;
  _magOffset[0] = my;
  _magOffset[0] = mz;
}

void ImuDatafusion::getEulerAngle(double *data)
{
  mtxRW.lock();
  for (size_t i = 0; i < 3; i++)
  {
    data[i] = _eulerAngle[i];
  }
  mtxRW.unlock();
}

//---------------------------------------------------------------------------------------------------

void ImuDatafusion::fusion(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float mx, float my, float mz,
                           float *r, float *p, float *y)
{
  if ((gx == 0.0f) && (gy == 0.0f) && (gz == 0.0f))
    return;

  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  // 测量正常化
  norm = sqrt(ax * ax + ay * ay + az * az);
  ax = ax / norm; //单位化
  ay = ay / norm;
  az = az / norm;

  // 估计方向的重力
  vx = 2 * (q1 * q3 - q0 * q2);
  vy = 2 * (q0 * q1 + q2 * q3);
  vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);

  // 积分误差比例积分增益
  exInt = exInt + ex * _ki;
  eyInt = eyInt + ey * _ki;
  ezInt = ezInt + ez * _ki;

  // 调整后的陀螺仪测量
  gx = gx + _kp * ex + exInt;
  gy = gy + _kp * ey + eyInt;
  gz = gz + _kp * ez + ezInt;

  // 整合四元数率和正常化
  q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * _halfT;
  q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * _halfT;
  q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * _halfT;
  q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * _halfT;

  // 正常化四元
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  // 转换为弧度
  *r = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1);      // roll
  *p = asin(-2 * q1 * q3 + 2 * q0 * q2);                                      // pitch
  *y = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3); // yaw
}

void ImuDatafusion::datafusion(double *gyro, double *acc, double *mag) // m/s^2, rad/s
{
  float rpy[3];
  for (size_t i = 0; i < 3; i++)
  {
    _gyro[i] = gyro[i] - _gyroOffset[i];
    _acc[i] = acc[i] - _accOffset[i];
    _mag[i] = mag[i] - _magOffset[i];
  }
  fusion(_gyro[0], _gyro[1], _gyro[2],
         _acc[0], _acc[1], _acc[2],
         _mag[0], _mag[1], _mag[2],
         &rpy[0], &rpy[1], &rpy[2]);
  mtxRW.lock();
  for (size_t i = 0; i < 3; i++)
  {
    _eulerAngle[i] = rpy[i];
  }
  mtxRW.unlock();
}
