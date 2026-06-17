#pragma once

#include <Eigen/Dense>
#include <vector>

class MPC {
public:
  MPC(const Eigen::Matrix4d &Ad, const Eigen::Vector4d &Bd,
      const Eigen::Matrix4d &Q, double R, int horizon, double u_min,
      double u_max);

  double solve(const Eigen::Vector4d &x0, int iterations = 60,
               double alpha = 0.001);

private:
  std::vector<Eigen::Vector4d> rollout(const Eigen::Vector4d &x0,
                                       const std::vector<double> &U) const;

  double computeCost(const Eigen::Vector4d &x0,
                     const std::vector<double> &U) const;

  double clamp(double u) const;

private:
  Eigen::Matrix4d Ad_;
  Eigen::Vector4d Bd_;
  Eigen::Matrix4d Q_;

  double R_;

  int horizon_;

  double u_min_;
  double u_max_;

  std::vector<double> U_;
};