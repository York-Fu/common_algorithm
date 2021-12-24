#include <iostream>
#include <unistd.h>
#include <thread>
#include "imuDatafusion.h"

ImuDatafusion imuFusion;

void datafusionThread()
{
  double gyro[3] = {0, 0, 0};
  double acc[3] = {0, 0, 9.8};
  double mag[3] = {0, 0, 0};
  while (1)
  {
    imuFusion.datafusion(gyro, acc, mag);
    usleep(1000);
  }
}

int main(int argv, char **argc)
{
  imuFusion.calibrateGyro(0.0, 0.0, 0.0);
  double eulerAngle[3];

  std::thread threadImu(datafusionThread);

  while (1)
  {
    imuFusion.getEulerAngle(eulerAngle);
    std::cout << "rpy: " << eulerAngle[0] << ", " << eulerAngle[1] << ", " << eulerAngle[2] << "\n";
    sleep(1);
  }

  threadImu.join();

  return 0;
}
