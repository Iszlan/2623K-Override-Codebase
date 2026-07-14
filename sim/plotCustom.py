import csv
import matplotlib.pyplot as plt
import matplotlib.collections as mcoll
import numpy as np
import os

DATA_DIR = "data"
OUT_DIR = "plots"
os.makedirs(OUT_DIR, exist_ok=True)


def read_csv(name):
    with open(os.path.join(DATA_DIR, name)) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0].keys()}


def coloured_line(ax, x, y, c, cmap="viridis", lw=3):
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = mcoll.LineCollection(segments, cmap=cmap, linewidths=lw)
    lc.set_array(c)
    ax.add_collection(lc)
    ax.autoscale()
    return lc


path = read_csv("custom_route_path_points.csv")
time_traj = read_csv("custom_route_time_trajectory.csv")

fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))

lc = coloured_line(axes[0], path["x"], path["y"], np.abs(path["curvature"]), cmap="viridis")
fig.colorbar(lc, ax=axes[0], label="|curvature| (1/in)")
axes[0].add_patch(plt.Rectangle((-72, -72), 144, 144, fill=False, edgecolor="grey", linestyle="--", linewidth=1.5))
axes[0].plot(path["x"][0], path["y"][0], "go", markersize=10, label="start")
axes[0].plot(path["x"][-1], path["y"][-1], "rs", markersize=10, label="end")
axes[0].set_xlim(-72, 72)
axes[0].set_ylim(-72, 72)
axes[0].set_xlabel("x (in)")
axes[0].set_ylabel("y (in)")
axes[0].set_title("Custom route - coloured by curvature")
axes[0].legend(loc="best")
axes[0].set_aspect("equal")
axes[0].grid(alpha=0.3)

axes[1].plot(time_traj["time"], time_traj["velocity"], color="tab:green", label="linear velocity")
axes[1].plot(time_traj["time"], time_traj["leftVelocity"], color="tab:cyan", alpha=0.6, label="left wheel")
axes[1].plot(time_traj["time"], time_traj["rightVelocity"], color="tab:pink", alpha=0.6, label="right wheel")
axes[1].axhline(0, color="grey", linewidth=1)
axes[1].set_xlabel("time (s)")
axes[1].set_ylabel("velocity (in/s)")
axes[1].set_title("Time-parameterised velocity profile")
axes[1].legend()
axes[1].grid(alpha=0.3)

fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "custom_route.png"), dpi=150)
plt.close(fig)

total_distance = path["distance"][-1]
total_time = time_traj["time"][-1]
print(f"route length: {total_distance:.2f} in")
print(f"total time: {total_time:.2f} s")
print(f"average speed: {total_distance/total_time:.2f} in/s")
print("Wrote plot to", os.path.abspath(os.path.join(OUT_DIR, "custom_route.png")))