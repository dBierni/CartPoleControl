#include "LQR.hpp"

#include <stdexcept>

Eigen::RowVector4d LQR::solveDiscreteARE( const Eigen::Matrix4d& Ad,const Eigen::Vector4d& Bd, const Eigen::Matrix4d& Q, double R,
    int max_iterations,  double tolerance) {
  if (R <= 0.0) {
    throw std::invalid_argument("R must be positive.");
  }

  Eigen::Matrix4d P = Q;

  for (int i = 0; i < max_iterations; ++i) {
    Eigen::Matrix<double, 1, 1> S =  Eigen::Matrix<double, 1, 1>::Constant(R)  + Bd.transpose() * P * Bd;

    Eigen::Matrix4d P_next = Ad.transpose() * P * Ad - Ad.transpose() * P * Bd * S.inverse() * Bd.transpose() * P * Ad + Q;

    double diff = (P_next - P).norm();
    P = P_next;

    if (diff < tolerance) {
      break;
    }
  }

  Eigen::Matrix<double, 1, 1> S =  Eigen::Matrix<double, 1, 1>::Constant(R) + Bd.transpose() * P * Bd;

  Eigen::RowVector4d K =   S.inverse() * Bd.transpose() * P * Ad;

  return K;
}