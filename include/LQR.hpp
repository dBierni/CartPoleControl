#pragma once

#include <Eigen/Dense>

class LQR {
public:
  static Eigen::RowVector4d solveDiscreteARE(const Eigen::Matrix4d& Ad, const Eigen::Vector4d& Bd, const Eigen::Matrix4d& Q,  double R,
         int max_iterations = 10000,double tolerance = 1e-9);
};