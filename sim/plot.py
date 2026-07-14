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


# Plot 1: doc example path, coloured by curvature
path = read_csv("doc_example_path_points.csv")
control_points = np.array([[12, 36], [12, 48], [36, 36], [36, 48]])

fig, ax = plt.subplots(figsize=(7, 6))
lc = coloured_line(ax, path["x"], path["y"], np.abs(path["curvature"]), cmap="viridis")
fig.colorbar(lc, ax=ax, label="|curvature| (1/in)")
ax.plot(control_points[:, 0], control_points[:, 1], "o--", color="grey", alpha=0.6, label="Bezier control points")
ax.plot(path["x"][0], path["y"][0], "go", markersize=10, label="start (12, 36)")
ax.plot(path["x"][-1], path["y"][-1], "rs", markersize=10, label="end (36, 48)")
ax.set_xlabel("x (in)")
ax.set_ylabel("y (in)")
ax.set_title("Doc example path - coloured by curvature")
ax.legend(loc="lower right")
ax.set_aspect("equal")
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "1_doc_example_path.png"), dpi=150)
plt.close(fig)

# Plot 2: 3-stage trajectory generation progression
stages = read_csv("doc_example_trajectory_stages.csv")
fig, axes = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
axes[0].plot(stages["distance"], stages["velocityLimitOnly"], color="tab:red")
axes[0].set_title("Stage 1: velocity limiting only (no forward pass)")
axes[0].set_ylabel("velocity (in/s)")
axes[0].grid(alpha=0.3)
axes[1].plot(stages["distance"], stages["forwardPassOnly"], color="tab:orange")
axes[1].set_title("Stage 2: full constraints, forward pass only")
axes[1].set_ylabel("velocity (in/s)")
axes[1].grid(alpha=0.3)
axes[2].plot(stages["distance"], stages["final"], color="tab:green")
axes[2].set_title("Stage 3: final = min(forward pass, backward pass)")
axes[2].set_ylabel("velocity (in/s)")
axes[2].set_xlabel("distance (in)")
axes[2].grid(alpha=0.3)
fig.suptitle("Doc example - trajectory generation stages", y=1.0)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "2_doc_example_stages.png"), dpi=150)
plt.close(fig)

# Plot 3: time-parameterised velocity + wheel velocities
time_traj = read_csv("doc_example_time_trajectory.csv")
fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
axes[0].plot(time_traj["time"], time_traj["velocity"], color="tab:blue", label="linear velocity")
axes[0].plot(time_traj["time"], time_traj["angularVelocity"], color="tab:purple", label="angular velocity", alpha=0.7)
axes[0].set_ylabel("velocity (in/s) / (rad/s)")
axes[0].set_title("Time-parameterised profile")
axes[0].legend()
axes[0].grid(alpha=0.3)
axes[1].plot(time_traj["time"], time_traj["leftVelocity"], color="tab:cyan", label="left wheel")
axes[1].plot(time_traj["time"], time_traj["rightVelocity"], color="tab:pink", label="right wheel")
axes[1].set_xlabel("time (s)")
axes[1].set_ylabel("wheel velocity (in/s)")
axes[1].set_title("Left/right wheel velocities")
axes[1].legend()
axes[1].grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "3_doc_example_time_and_wheels.png"), dpi=150)
plt.close(fig)

# Plot 4: multi-waypoint field path + velocity profile
field_path = read_csv("field_example_path_points.csv")
field_time = read_csv("field_example_time_trajectory.csv")
waypoints = np.array([[-48, -48], [-24, -12], [24, 0], [48, 48]])

fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
axes[0].add_patch(plt.Rectangle((-72, -72), 144, 144, fill=False, edgecolor="grey", linestyle="--", linewidth=1.5))
axes[0].plot(field_path["x"], field_path["y"], color="tab:blue", linewidth=2.5)
axes[0].plot(waypoints[:, 0], waypoints[:, 1], "o--", color="grey", alpha=0.7, label="waypoints")
axes[0].set_xlim(-72, 72)
axes[0].set_ylim(-72, 72)
axes[0].set_xlabel("x (in)")
axes[0].set_ylabel("y (in)")
axes[0].set_title("Multi-waypoint path on a 144x144in field (centre origin)")
axes[0].legend()
axes[0].set_aspect("equal")
axes[0].grid(alpha=0.3)
axes[1].plot(field_time["time"], field_time["velocity"], color="tab:green", label="linear velocity")
axes[1].plot(field_time["time"], field_time["leftVelocity"], color="tab:cyan", alpha=0.6, label="left wheel")
axes[1].plot(field_time["time"], field_time["rightVelocity"], color="tab:pink", alpha=0.6, label="right wheel")
axes[1].set_xlabel("time (s)")
axes[1].set_ylabel("velocity (in/s)")
axes[1].set_title("Time-parameterised velocity profile")
axes[1].legend()
axes[1].grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "4_field_example.png"), dpi=150)
plt.close(fig)

# Plot 5: forward vs backward driving comparison
fwd_time = read_csv("doc_example_time_trajectory.csv")
back_time = read_csv("backward_example_time_trajectory.csv")

fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
axes[0].plot(fwd_time["time"], fwd_time["velocity"], color="tab:green", label="forwards=true")
axes[0].plot(back_time["time"], back_time["velocity"], color="tab:red", label="forwards=false")
axes[0].axhline(0, color="grey", linewidth=1)
axes[0].set_xlabel("time (s)")
axes[0].set_ylabel("velocity (in/s)")
axes[0].set_title("Linear velocity: forward vs backward")
axes[0].legend()
axes[0].grid(alpha=0.3)
axes[1].plot(fwd_time["time"], np.degrees(fwd_time["heading"]), color="tab:green", label="forwards=true")
axes[1].plot(back_time["time"], np.degrees(back_time["heading"]), color="tab:red", label="forwards=false")
axes[1].set_xlabel("time (s)")
axes[1].set_ylabel("heading (deg)")
axes[1].set_title("Heading: forward vs backward (should differ by 180deg)")
axes[1].legend()
axes[1].grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "5_forward_vs_backward.png"), dpi=150)
plt.close(fig)

print("Wrote plots to", os.path.abspath(OUT_DIR))
