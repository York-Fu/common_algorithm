#include <iostream>
#include <unistd.h>
#include "kalman.h"

int main(int argv, char **argc)
{
  double dt = 0.001;
  
  Kalman kalmanFilter(3, 3);

  double_t Q[3] = {1, 1, 1};
  double_t R[3] = {1, 1, 1};
  kalmanFilter.setQR(Q, R);

  Eigen::MatrixXd F(3, 3);
  F << 1., dt, dt * dt / 2.,
      0., 1., dt,
      0., 0., 1.;
  Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  kalmanFilter.update_F_H(F, H);

  Eigen::Vector3d measure;
  Eigen::Vector3d result;
  for (size_t i = 0; i < 20; i++)
  {
    measure << 1, 1, 1;
    result = kalmanFilter.updateData(measure);
    std::cout << "result: " << result.transpose() << "\n";
  }

  return 0;
}
