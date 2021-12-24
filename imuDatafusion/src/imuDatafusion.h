#ifndef _ImuDatafusion_h_
#define _ImuDatafusion_h_

#include <iostream>
#include <math.h>
#include <iomanip>
#include <mutex>

class ImuDatafusion
{
public:
  ImuDatafusion(float kp = 15.0f, float ki = 0.004f, float halfT = 0.005f);
  ~ImuDatafusion();
  void setFusionCoeff(float kp, float ki, float halfT);
  void calibrateGyro(double gx, double gy, double gz);
  void calibrateAcc(double ax, double ay, double az);
  void calibrateMag(double mx, double my, double mz);
  void getEulerAngle(double *data);
  void datafusion(double *gyro, double *acc, double *mag);

private:
  void fusion(float gx, float gy, float gz,
              float ax, float ay, float az,
              float mx, float my, float mz,
              float *r, float *p, float *y);

  double _gyro[3];
  double _acc[3];
  double _mag[3];
  double _gyroOffset[3];
  double _accOffset[3];
  double _magOffset[3];
  double _eulerAngle[3];
  std::mutex mtxRW;

  float _kp;                 // 比例增益支配率收敛到加速度计/磁强计
  float _ki;                 // 积分增益支配率的陀螺仪偏见的衔接
  float _halfT;              // 采样周期的一半
  float q0, q1, q2, q3;      // 四元数的元素，代表估计方向
  float exInt, eyInt, ezInt; // 按比例缩小积分误差
};

#endif