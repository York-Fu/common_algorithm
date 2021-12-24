#ifndef _kalman_h_
#define _kalman_h_

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>

class Kalman
{
public:
  Kalman();
  Kalman(int state, int measure);

  void setQR(double *Q, double *R);
  void update_F_H(Eigen::MatrixXd F, Eigen::MatrixXd H);
  Eigen::MatrixXd updateData(Eigen::MatrixXd z);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x;

private:
  int _state, _measure;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _F;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _P;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _H;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _K;
};

#endif
