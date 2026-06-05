import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

df = pd.read_csv("../data/simulation.csv")

x = df["x"].values
theta = df["theta"].values

# Must match C++ pole length
l = 0.5

cart_width = 0.35
cart_height = 0.18

# Downsample for smaller GIF
stride = 20
x_anim = x[::stride]
theta_anim = theta[::stride]

fig, ax = plt.subplots(figsize=(8, 4))

ax.set_aspect("equal")
ax.set_title("Cart-Pole LQR Stabilization")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.grid(True)

margin = 0.8
ax.set_xlim(x.min() - margin, x.max() + margin)
ax.set_ylim(-0.3, l + 0.4)

# Ground line
ax.plot([x.min() - margin, x.max() + margin], [0, 0], "k--", linewidth=1)

cart = plt.Rectangle(
    (x_anim[0] - cart_width / 2, 0),
    cart_width,
    cart_height,
    fill=False,
    linewidth=2
)
ax.add_patch(cart)

pole_line, = ax.plot([], [], linewidth=3)
mass_point, = ax.plot([], [], "o", markersize=8)


def init():
    pole_line.set_data([], [])
    mass_point.set_data([], [])
    return cart, pole_line, mass_point


def update(frame):
    cart_x = x_anim[frame]
    th = theta_anim[frame]

    cart.set_xy((cart_x - cart_width / 2, 0))

    pivot_x = cart_x
    pivot_y = cart_height

    # theta = 0 upright
    pole_x = pivot_x + l * np.sin(th)
    pole_y = pivot_y + l * np.cos(th)

    pole_line.set_data([pivot_x, pole_x], [pivot_y, pole_y])
    mass_point.set_data([pole_x], [pole_y])

    return cart, pole_line, mass_point


anim = FuncAnimation(
    fig,
    update,
    frames=len(x_anim),
    init_func=init,
    blit=True,
    interval=40
)

anim.save("../data/cartpole_lqr.gif", writer=PillowWriter(fps=25))
print("Saved ../data/cartpole_lqr.gif")