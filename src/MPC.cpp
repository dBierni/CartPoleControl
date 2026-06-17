#include "MPC.hpp"

#include <algorithm>

MPC::MPC(const Eigen::Matrix4d& Ad,const Eigen::Vector4d& Bd, const Eigen::Matrix4d& Q, double R,
         int horizon, double u_min, double u_max)
    : Ad_(Ad),
      Bd_(Bd),
      Q_(Q),
      R_(R),
      horizon_(horizon),
      u_min_(u_min),
      u_max_(u_max),
      U_(horizon, 0.0)
{
}

double MPC::clamp(double u) const {
    return std::max(u_min_, std::min(u_max_, u));
}

std::vector<Eigen::Vector4d> MPC::rollout(const Eigen::Vector4d& x0, const std::vector<double>& U) const {
    std::vector<Eigen::Vector4d> X;

    X.reserve(horizon_ + 1);

    Eigen::Vector4d x = x0;

    X.push_back(x);

    for (int k = 0; k < horizon_; ++k) {
        x = Ad_ * x + Bd_ * clamp(U[k]);
        X.push_back(x);
    }

    return X;
}

double MPC::computeCost(const Eigen::Vector4d& x0, const std::vector<double>& U) const {
    auto X = rollout(x0, U);

    double cost = 0.0;

    for (int k = 0; k < horizon_; ++k) {
        cost += (X[k].transpose() * Q_ * X[k])(0, 0);
        cost += R_ * U[k] * U[k];
    }

    cost += (X.back().transpose() * Q_ * X.back())(0, 0);

    return cost;
}

double MPC::solve(const Eigen::Vector4d& x0, int iterations, double alpha) {
    const double eps = 1e-4;

    for (int iter = 0; iter < iterations; ++iter) {
        const double base_cost = computeCost(x0, U_);

        std::vector<double> gradient(horizon_, 0.0);

        for (int k = 0; k < horizon_; ++k) {
            std::vector<double> U_test = U_;

            U_test[k] += eps;

            double cost_plus = computeCost(x0, U_test);

            gradient[k] = (cost_plus - base_cost) / eps;
        }

        for (int k = 0; k < horizon_; ++k) {
            U_[k] -= alpha * gradient[k];
            U_[k] = clamp(U_[k]);
        }
    }

    double u0 = clamp(U_[0]);

    // Receding horizon warm start
    for (int k = 0; k < horizon_ - 1; ++k) {
        U_[k] = U_[k + 1];
    }

    U_.back() = 0.0;

    return u0;
}