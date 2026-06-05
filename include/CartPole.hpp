#pragma once

#include <Eigen/Dense>

class CartPole {
public:
  CartPole(double cart_mass, double pole_mass, double pole_length, double gravity);

  // Nonlinear dynamics: state = [x, x_dot, theta, theta_dot]
  Eigen::Vector4d dynamics(const Eigen::Vector4d& state, double force) const;

  // Linearized around upright equilibrium theta = 0
  Eigen::Matrix4d A() const;
  Eigen::Vector4d B() const;

private:
  double M_; // cart mass
  double m_; // pole mass
  double l_; // pole CoM length
  double g_; // gravity
};