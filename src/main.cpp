#include "CartPole.hpp"
#include "LQR.hpp"
#include "MPC.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>

Eigen::Vector4d rk4Step(const CartPole& cartpole, const Eigen::Vector4d& state, double force,double dt) {
    Eigen::Vector4d k1 = cartpole.dynamics(state, force);
    Eigen::Vector4d k2 = cartpole.dynamics(state + 0.5 * dt * k1, force);
    Eigen::Vector4d k3 = cartpole.dynamics(state + 0.5 * dt * k2, force);
    Eigen::Vector4d k4 = cartpole.dynamics(state + dt * k3, force);

    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

int main(int argc, char **argv) {

  // -------------------------------------------------
  // Controller selection
  // -------------------------------------------------

  std::string controller = "lqr";

  if (argc > 1) {
    controller = argv[1];
  }

  if (controller != "lqr" && controller != "mpc") {
    std::cerr << "Usage: ./cartpole_control [lqr|mpc]\n";
    return 1;
  }

  std::cout << "Controller: " << controller << "\n";

  // -------------------------------------------------
  // Cart-pole parameters
  // -------------------------------------------------
  const double M = 1.0;
  const double m = 0.1;
  const double l = 0.5;
  const double g = 9.81;
  CartPole cartpole(M, m, l, g);

  // -------------------------------------------------
  // Simulation parameters
  // -------------------------------------------------
  const double dt = 0.001;
  const int mpc_update_steps = 20;
  const double mpc_dt = dt * mpc_update_steps;
  const double T = 8.0;

  const int steps = static_cast<int>(T / dt);

  // -------------------------------------------------
  // Linear model
  // -------------------------------------------------
  Eigen::Matrix4d A = cartpole.A();
  Eigen::Vector4d B = cartpole.B();

  Eigen::Matrix4d Ad = Eigen::Matrix4d::Identity() + A * dt;
  Eigen::Vector4d Bd = B * dt;

  Eigen::Matrix4d Ad_mpc = Eigen::Matrix4d::Identity() + A * mpc_dt;
  Eigen::Vector4d Bd_mpc = B * mpc_dt;
  // -------------------------------------------------
  // Cost matrices
  // -------------------------------------------------
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0, 0) = 10.0;     // cart position
  Q(1, 1) = 2.0;     // cart velocity
  Q(2, 2) = 100.0;   // pole angle
  Q(3, 3) = 10.0;    // pole angular velocity

  double R = 0.1;

  // -------------------------------------------------
  // LQR controller
  // -------------------------------------------------
  Eigen::RowVector4d K = LQR::solveDiscreteARE(Ad, Bd, Q, R);

  // -------------------------------------------------
  // MPC controller
  // -------------------------------------------------

  const int horizon = 50;

  const double u_min = -10.0;
  const double u_max = 10.0;

  MPC mpc(Ad_mpc, Bd_mpc, Q, R, horizon, u_min, u_max);

  // -------------------------------------------------
  // Initial state
  // -------------------------------------------------
  Eigen::Vector4d state;
  state << 0.0, // cart position [m]
      0.0,      // cart velocity [m/s]
      0.15,      // pole angle [rad]
      0.0;      // pole angular velocity [rad/s]
  std::string filename = "../data/" + controller + ".csv";
  std::ofstream log(filename);

  if (!log.is_open()) {
      std::cerr << "ERROR: Could not open ../data/simulation.csv\n";
      return 1;
  }

  log << "t,x,x_dot,theta,theta_dot,u\n";
  double u = 0.0;
  for (int i = 0; i < steps; ++i) {
    double time = i * dt;

    if (controller == "lqr") {
      u = -(K * state)(0);
      u = std::clamp(u, u_min, u_max);
    } else if (controller == "mpc") {

      if (i % mpc_update_steps == 0) {
        // optimization iterations, // gradient step size
        u = mpc.solve(state,  80, 0.0005);
      }
    }
    state = rk4Step(cartpole, state, u, dt);

    log << time << "," << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << "," << u << "\n";
  }

  log.close();

  std::cout << filename + " written.\n";

  return 0;
}