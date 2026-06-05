#include "CartPole.hpp"

#include <cmath>

CartPole::CartPole(double cart_mass, double pole_mass, double pole_length, double gravity)
    : M_(cart_mass), m_(pole_mass), l_(pole_length), g_(gravity){}


Eigen::Vector4d CartPole::dynamics(const Eigen::Vector4d& state, double force) const {
  const double x_dot = state(1);
  const double theta = state(2);
  const double theta_dot = state(3);

  const double s = std::sin(theta);
  const double c = std::cos(theta);

  const double denom = M_ + m_ * s * s;

  const double x_ddot = (force + m_ * s * (l_ * theta_dot * theta_dot - g_ * c)) / denom;

  const double theta_ddot = (-force * c - m_ * l_ * theta_dot * theta_dot * c * s  + (M_ + m_) * g_ * s) / (l_ * denom);

  Eigen::Vector4d state_dot;
  state_dot << x_dot,  x_ddot,  theta_dot,  theta_ddot;

  return state_dot;
}

Eigen::Matrix4d CartPole::A() const {
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

  A(0, 1) = 1.0;
  A(1, 2) = -(m_ * g_) / M_;
  A(2, 3) = 1.0;
  A(3, 2) = ((M_ + m_) * g_) / (M_ * l_);

  return A;
}

Eigen::Vector4d CartPole::B() const {
  Eigen::Vector4d B = Eigen::Vector4d::Zero();

  B(1) = 1.0 / M_;
  B(3) = -1.0 / (M_ * l_);

  return B;
}