import sys
import pandas as pd
import matplotlib.pyplot as plt

# -------------------------------------------------
# Parse command line
# -------------------------------------------------

controller = "lqr"

if len(sys.argv) > 1:
    controller = sys.argv[1]

if controller not in ["lqr", "mpc"]:
    print("Usage: python plot_results.py [lqr|mpc]")
    sys.exit(1)

csv_file = f"../data/{controller}.csv"

# -------------------------------------------------
# Load data
# -------------------------------------------------

df = pd.read_csv(csv_file)

t = df["t"]
x = df["x"]
x_dot = df["x_dot"]
theta = df["theta"]
theta_dot = df["theta_dot"]
u = df["u"]

# -------------------------------------------------
# Cart position and velocity
# -------------------------------------------------

plt.figure(figsize=(10, 5))

plt.plot(t, x, label="x [m]")
plt.plot(t, x_dot, label="x_dot [m/s]")

plt.title(f"Cart State Response ({controller.upper()})")
plt.xlabel("Time [s]")
plt.ylabel("State value")

plt.grid(True)
plt.legend()
plt.tight_layout()

filename = f"../data/{controller}_cart_state_response.png"
plt.savefig(filename, dpi=200)

# -------------------------------------------------
# Pole angle and angular velocity
# -------------------------------------------------

plt.figure(figsize=(10, 5))

plt.plot(t, theta, label="theta [rad]")
plt.plot(t, theta_dot, label="theta_dot [rad/s]")

plt.axhline(0.0, linestyle="--", linewidth=1)

plt.title(f"Pole State Response ({controller.upper()})")
plt.xlabel("Time [s]")
plt.ylabel("State value")

plt.grid(True)
plt.legend()
plt.tight_layout()

filename = f"../data/{controller}_pole_state_response.png"
plt.savefig(filename, dpi=200)

# -------------------------------------------------
# Control force
# -------------------------------------------------

plt.figure(figsize=(10, 4))

plt.plot(t, u, label="u [N]")

plt.axhline(10.0, linestyle="--", linewidth=1, label="force limit")
plt.axhline(-10.0, linestyle="--", linewidth=1)

plt.title(f"Control Force ({controller.upper()})")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")

plt.grid(True)
plt.legend()
plt.tight_layout()

filename = f"../data/{controller}_control_force.png"
plt.savefig(filename, dpi=200)

# -------------------------------------------------
# Pole phase portrait
# -------------------------------------------------

plt.figure(figsize=(6, 6))

plt.plot(theta, theta_dot)

plt.scatter([theta.iloc[0]], [theta_dot.iloc[0]], label="start")
plt.scatter([theta.iloc[-1]], [theta_dot.iloc[-1]], label="end")
plt.scatter([0], [0], marker="x", s=100, label="equilibrium")

plt.title(f"Pole Phase Portrait ({controller.upper()})")
plt.xlabel("theta [rad]")
plt.ylabel("theta_dot [rad/s]")

plt.grid(True)
plt.legend()
plt.tight_layout()

filename = f"../data/{controller}_pole_phase_portrait.png"
plt.savefig(filename, dpi=200)

# -------------------------------------------------
# Summary
# -------------------------------------------------

print(f"Generated plots for {controller.upper()}:")
print(f"../data/{controller}_cart_state_response.png")
print(f"../data/{controller}_pole_state_response.png")
print(f"../data/{controller}_control_force.png")
print(f"../data/{controller}_pole_phase_portrait.png")