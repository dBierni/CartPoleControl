import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("../data/simulation.csv")

t = df["t"]
x = df["x"]
x_dot = df["x_dot"]
theta = df["theta"]
theta_dot = df["theta_dot"]
u = df["u"]

# -----------------------------
# Cart position and velocity
# -----------------------------
plt.figure(figsize=(10, 5))
plt.plot(t, x, label="x [m]")
plt.plot(t, x_dot, label="x_dot [m/s]")
plt.title("Cart State Response")
plt.xlabel("Time [s]")
plt.ylabel("State value")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("../data/cart_state_response.png", dpi=200)

# -----------------------------
# Pole angle and angular velocity
# -----------------------------
plt.figure(figsize=(10, 5))
plt.plot(t, theta, label="theta [rad]")
plt.plot(t, theta_dot, label="theta_dot [rad/s]")
plt.axhline(0.0, linestyle="--", linewidth=1)
plt.title("Pole State Response")
plt.xlabel("Time [s]")
plt.ylabel("State value")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("../data/pole_state_response.png", dpi=200)

# -----------------------------
# Control force
# -----------------------------
plt.figure(figsize=(10, 4))
plt.plot(t, u, label="u [N]")
plt.axhline(10.0, linestyle="--", linewidth=1, label="force limit")
plt.axhline(-10.0, linestyle="--", linewidth=1)
plt.title("LQR Control Force")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("../data/control_force.png", dpi=200)

# -----------------------------
# Phase portrait: pole
# -----------------------------
plt.figure(figsize=(6, 6))
plt.plot(theta, theta_dot)
plt.scatter([theta.iloc[0]], [theta_dot.iloc[0]], label="start")
plt.scatter([theta.iloc[-1]], [theta_dot.iloc[-1]], label="end")
plt.scatter([0], [0], marker="x", s=100, label="equilibrium")
plt.title("Pole Phase Portrait")
plt.xlabel("theta [rad]")
plt.ylabel("theta_dot [rad/s]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("../data/pole_phase_portrait.png", dpi=200)

print("Saved plots:")
print("../data/cart_state_response.png")
print("../data/pole_state_response.png")
print("../data/control_force.png")
print("../data/pole_phase_portrait.png")