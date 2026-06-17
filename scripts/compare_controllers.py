import pandas as pd
import matplotlib.pyplot as plt

# -------------------------------------------------
# Load controller results
# -------------------------------------------------
lqr = pd.read_csv("../data/lqr.csv")
mpc = pd.read_csv("../data/mpc.csv")

# -------------------------------------------------
# Pole angle comparison
# -------------------------------------------------
plt.figure(figsize=(10, 5))
plt.plot(lqr["t"], lqr["theta"], label="LQR")
plt.plot(mpc["t"], mpc["theta"], label="MPC")
plt.axhline(0.0, linestyle="--", linewidth=1)

plt.title("Pole Angle: LQR vs MPC")
plt.xlabel("Time [s]")
plt.ylabel("Theta [rad]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("../data/lqr_vs_mpc_theta.png", dpi=200)

# -------------------------------------------------
# Cart position comparison
# -------------------------------------------------
plt.figure(figsize=(10, 5))
plt.plot(lqr["t"], lqr["x"], label="LQR")
plt.plot(mpc["t"], mpc["x"], label="MPC")
plt.axhline(0.0, linestyle="--", linewidth=1)

plt.title("Cart Position: LQR vs MPC")
plt.xlabel("Time [s]")
plt.ylabel("Cart position [m]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("../data/lqr_vs_mpc_cart_position.png", dpi=200)

# -------------------------------------------------
# Control force comparison
# -------------------------------------------------
plt.figure(figsize=(10, 5))
plt.plot(lqr["t"], lqr["u"], label="LQR")
plt.plot(mpc["t"], mpc["u"], label="MPC")
plt.axhline(10.0, linestyle="--", linewidth=1, label="Force limit")
plt.axhline(-10.0, linestyle="--", linewidth=1)

plt.title("Control Force: LQR vs MPC")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("../data/lqr_vs_mpc_force.png", dpi=200)

# -------------------------------------------------
# Pole phase portrait comparison
# -------------------------------------------------
plt.figure(figsize=(6, 6))
plt.plot(lqr["theta"], lqr["theta_dot"], label="LQR")
plt.plot(mpc["theta"], mpc["theta_dot"], label="MPC")

plt.scatter([lqr["theta"].iloc[0]], [lqr["theta_dot"].iloc[0]], label="start", s=50)
plt.scatter([0], [0], marker="x", s=100, label="equilibrium")

plt.title("Pole Phase Portrait: LQR vs MPC")
plt.xlabel("Theta [rad]")
plt.ylabel("Theta dot [rad/s]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("../data/lqr_vs_mpc_phase.png", dpi=200)

print("Saved comparison plots:")
print("../data/lqr_vs_mpc_theta.png")
print("../data/lqr_vs_mpc_cart_position.png")
print("../data/lqr_vs_mpc_force.png")
print("../data/lqr_vs_mpc_phase.png")