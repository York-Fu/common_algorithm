#include "imuData.h"

#define IMU_SAMPLE_FREQ 0.0
#define IMU_CUTOFF_FREQ -1.0 // no filtering

ImuData::ImuData() : lpfGyroX(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ),
                     lpfGyroY(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ),
                     lpfGyroZ(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ),
                     lpfAccX(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ),
                     lpfAccY(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ),
                     lpfAccZ(IMU_SAMPLE_FREQ, IMU_CUTOFF_FREQ)
{
  dataOffset.angularVelocity.x = 0.0;
  dataOffset.angularVelocity.y = 0.0;
  dataOffset.angularVelocity.z = 0.0;
  dataOffset.linearAcceleration.x = 0.0;
  dataOffset.linearAcceleration.y = 0.0;
  dataOffset.linearAcceleration.z = 0.0;
  dataOffset.attitudeAngle.roll = 0.0;
  dataOffset.attitudeAngle.pitch = 0.0;
  dataOffset.attitudeAngle.yaw = 0.0;

  _kp = 15.0f;                     // 比例增益支配率收敛到加速度计/磁强计
  _ki = 0.004f;                    // 积分增益支配率的陀螺仪偏见的衔接
  _halfT = 0.005f;                 // 采样周期的一半
  q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // 四元数的元素，代表估计方向
  exInt = 0, eyInt = 0, ezInt = 0; // 按比例缩小积分误差
}

ImuData::~ImuData()
{
}

void ImuData::calibrateGyro(double_t gx, double_t gy, double_t gz)
{
  dataOffset.angularVelocity.x = gx;
  dataOffset.angularVelocity.y = gy;
  dataOffset.angularVelocity.z = gz;
}

void ImuData::calibrateAcc(double_t ax, double_t ay, double_t az)
{
  dataOffset.linearAcceleration.x = ax;
  dataOffset.linearAcceleration.y = ay;
  dataOffset.linearAcceleration.z = az;
}

void ImuData::calibrateAttitude(double_t r, double_t p, double_t y)
{
  dataOffset.attitudeAngle.roll = r;
  dataOffset.attitudeAngle.pitch = p;
  dataOffset.attitudeAngle.yaw = y;
}

void ImuData::setGyro(double_t gx, double_t gy, double_t gz)
{
  mtxRW.lock();
  imuData.angularVelocity.x = gx - dataOffset.angularVelocity.x;
  imuData.angularVelocity.y = gy - dataOffset.angularVelocity.y;
  imuData.angularVelocity.z = gz - dataOffset.angularVelocity.z;
  mtxRW.unlock();

}

void ImuData::setAcc(double_t ax, double_t ay, double_t az)
{
  mtxRW.lock();
  imuData.linearAcceleration.x = ax - dataOffset.linearAcceleration.x;
  imuData.linearAcceleration.y = ay - dataOffset.linearAcceleration.y;
  imuData.linearAcceleration.z = az - dataOffset.linearAcceleration.z;
  mtxRW.unlock();
}

void ImuData::setAttitude(double_t roll, double_t pitch, double_t yaw)
{
  mtxRW.lock();
  imuData.attitudeAngle.roll = roll;
  imuData.attitudeAngle.pitch = pitch;
  imuData.attitudeAngle.yaw = yaw;
  mtxRW.unlock();
}

void ImuData::setGyroLpf(double_t sampleFreq, double_t cutoffFreq)
{
  lpfGyroX.set_cutoff_frequency(sampleFreq, cutoffFreq);
  lpfGyroY.set_cutoff_frequency(sampleFreq, cutoffFreq);
  lpfGyroZ.set_cutoff_frequency(sampleFreq, cutoffFreq);
}

void ImuData::setAccLpf(double_t sampleFreq, double_t cutoffFreq)
{
  lpfAccX.set_cutoff_frequency(sampleFreq, cutoffFreq);
  lpfAccY.set_cutoff_frequency(sampleFreq, cutoffFreq);
  lpfAccZ.set_cutoff_frequency(sampleFreq, cutoffFreq);
}

Axis_t ImuData::getGyro()
{
  mtxRW.lock();
  Axis_t data = imuData.angularVelocity;
  mtxRW.unlock();
  return data;
}

Axis_t ImuData::getAcc()
{
  mtxRW.lock();
  Axis_t data = imuData.linearAcceleration;
  mtxRW.unlock();
  return data;
}

AttitudeAngle_t ImuData::getAttitude()
{
  mtxRW.lock();
  AttitudeAngle_t data = imuData.attitudeAngle;
  mtxRW.unlock();
  return data;
}

ImuParam_t ImuData::getData()
{
  mtxRW.lock();
  ImuParam_t data = imuData;
  mtxRW.unlock();
  return data;
}

//---------------------------------------------------------------------------------------------------

void ImuData::fusion(float gx, float gy, float gz, float ax, float ay, float az)
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
  imuData.attitudeAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1);     // roll
  imuData.attitudeAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2);                                    // pitch
  imuData.attitudeAngle.yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3); // yaw
}

void ImuData::datafusion()
{
  float acc[3] = {0.0f, 0.0f, 0.0f};  //m/s^2
  float gyro[3] = {0.0f, 0.0f, 0.0f}; //rad/s
  float mag[3] = {0.0f, 0.0f, 0.0f};

  mtxRW.lock();
  gyro[0] = lpfGyroX.apply(imuData.angularVelocity.x);
  gyro[1] = lpfGyroY.apply(imuData.angularVelocity.y);
  gyro[2] = lpfGyroZ.apply(imuData.angularVelocity.z);
  acc[0] = lpfAccX.apply(imuData.linearAcceleration.x);
  acc[1] = lpfAccY.apply(imuData.linearAcceleration.y);
  acc[2] = lpfAccZ.apply(imuData.linearAcceleration.z);
  mag[0] = 0.0f;
  mag[1] = 0.0f;
  mag[2] = 0.0f;
  mtxRW.unlock();

  fusion(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
#if 0
  std::cout
      << "gx:" << imuData.angularVelocity.x * (180.0 / M_PI)
      << std::setw(10) << "gy:" << imuData.angularVelocity.y * (180.0 / M_PI)
      << std::setw(10) << "gz:" << imuData.angularVelocity.z * (180.0 / M_PI)
      << std::setw(10) << "ax:" << imuData.linearAcceleration.x
      << std::setw(10) << "ay:" << imuData.linearAcceleration.y
      << std::setw(10) << "az:" << imuData.linearAcceleration.z
      << std::setw(10) << "R:" << imuData.attitudeAngle.roll * (180.0 / M_PI)
      << std::setw(10) << "P:" << imuData.attitudeAngle.pitch * (180.0 / M_PI)
      << std::setw(10) << "Y:" << imuData.attitudeAngle.yaw * (180.0 / M_PI)
      << "\r";
#endif
}

void ImuData::fusionCoefficient(float kp, float ki, float halfT)
{
  _kp = kp;
  _ki = ki;
  _halfT = halfT;
}
