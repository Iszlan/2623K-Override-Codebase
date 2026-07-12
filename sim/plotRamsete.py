import csv
import matplotlib.pyplot as plt
import numpy as np
import os

DATA_DIR = "data"
OUT_DIR = "plots"
os.makedirs(OUT_DIR, exist_ok=True)


def read_csv(name):
    with open(os.path.join(DATA_DIR, name)) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0].keys()}


sim = read_csv("ramsete_sim.csv")

fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
axes[0].plot(sim["targetX"], sim["targetY"], "--", color="grey", linewidth=2, label="reference trajectory")
axes[0].plot(sim["robotX"], sim["robotY"], color="tab:red", linewidth=2, label="simulated robot (Ramsete-corrected)")
axes[0].plot(sim["robotX"][0], sim["robotY"][0], "o", color="tab:red", markersize=10, label="robot start (disturbed)")
axes[0].plot(sim["targetX"][0], sim["targetY"][0], "o", color="grey", markersize=10, label="reference start")
axes[0].set_xlabel("x (in)")
axes[0].set_ylabel("y (in)")
axes[0].set_title("Ramsete closed-loop tracking\n(robot starts 5in / 20deg off course)")
axes[0].legend(loc="lower right", fontsize=9)
axes[0].set_aspect("equal")
axes[0].grid(alpha=0.3)

axes[1].plot(sim["time"], sim["trackingError"], color="tab:blue")
axes[1].axhline(0, color="grey", linewidth=1, alpha=0.5)
axes[1].set_xlabel("time (s)")
axes[1].set_ylabel("tracking error (in)")
axes[1].set_title("Position error over time (should decay to ~0)")
axes[1].grid(alpha=0.3)

fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "6_ramsete_convergence.png"), dpi=150)
plt.close(fig)

print("initial error:", sim["trackingError"][0], "in")
print("final error:", sim["trackingError"][-1], "in")
print("Wrote plot to", os.path.abspath(os.path.join(OUT_DIR, "6_ramsete_convergence.png")))
