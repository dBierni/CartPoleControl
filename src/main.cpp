#include <iostream>
#include <fstream>
#include <algorithm>
#include "CartPole.hpp"
#include "LQR.hpp"

Eigen::Vector4d rk4Step(const CartPole& cartpole, const Eigen::Vector4d& state, double force,double dt) {
    Eigen::Vector4d k1 = cartpole.dynamics(state, force);
    Eigen::Vector4d k2 = cartpole.dynamics(state + 0.5 * dt * k1, force);
    Eigen::Vector4d k3 = cartpole.dynamics(state + 0.5 * dt * k2, force);
    Eigen::Vector4d k4 = cartpole.dynamics(state + dt * k3, force);

    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

int main() {
    const double M = 1.0;
    const double m = 0.1;
    const double l = 0.5;
    const double g = 9.81;

    CartPole cartpole(M, m, l, g);

    const double dt = 0.001;
    const double T = 8.0;
    const int steps = static_cast<int>(T / dt);

    Eigen::Matrix4d A = cartpole.A();
    Eigen::Vector4d B = cartpole.B();

    Eigen::Matrix4d Ad = Eigen::Matrix4d::Identity() + A * dt;
    Eigen::Vector4d Bd = B * dt;

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0, 0) = 1.0;     // cart position
    Q(1, 1) = 1.0;     // cart velocity
    Q(2, 2) = 100.0;   // pole angle
    Q(3, 3) = 10.0;    // pole angular velocity

    double R = 0.1;

    Eigen::RowVector4d K =  LQR::solveDiscreteARE(Ad, Bd, Q, R);

    std::cout << "A:\n" << A << "\n\n";
    std::cout << "B:\n" << B << "\n\n";
    std::cout << "K:\n" << K << "\n\n";

    Eigen::Vector4d state;
    state << 0.0,   // cart position [m]
             0.0,   // cart velocity [m/s]
             0.15,  // pole angle [rad]
             0.0;   // pole angular velocity [rad/s]

    std::ofstream log("../data/simulation.csv");

    if (!log.is_open()) {
        std::cerr << "ERROR: Could not open ../data/simulation.csv\n";
        return 1;
    }

    log << "t,x,x_dot,theta,theta_dot,u\n";

    for (int i = 0; i < steps; ++i) {
        double time = i * dt;

        double u = -(K * state)(0);
        // actuator saturation
        const double u_max = 10.0;
        u = std::clamp(u, -u_max, u_max);

        state = rk4Step(cartpole, state, u, dt);

        log << time << ","  << state(0) << "," << state(1) << ","  << state(2) << ","  << state(3) << ","  << u << "\n";
    }

    log.close();

    std::cout << "simulation.csv written.\n";

    return 0;
}