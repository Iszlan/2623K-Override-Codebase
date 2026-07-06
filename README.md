# 2623K Kingsmen VEX Robotics Codebase

A modular robotics control system for **high-precision autonomous movement** in VEX Robotics, featuring:

* IME + IMU based odometry
* RAMSETE nonlinear controller
* 2D motion profiling and trajectory following

This repository is designed for teams aiming to achieve **repeatable, accurate, and fast autonomous routines**.

---

### Odometry (IME + IMU Fusion)

* Uses **Integrated Motor Encoders (IMEs)** for linear displacement
* Uses **IMU** for heading estimation
* Combines both for robust pose tracking:

  * `(x, y, θ)` estimation
* Designed to be:

  * Drift-resistant (with correction hooks)
  * Lightweight (no external tracking wheels required)

---

### RAMSETE Controller

* Nonlinear time-varying feedback controller
* Tracks trajectories with:

  * Smooth convergence
  * Minimal overshoot
* Handles:

  * Pose error in global frame
  * Velocity-based trajectory tracking

**Why RAMSETE?**

* More stable than PID-only tracking
* Better performance on curves and dynamic paths

---

### 2D Motion Profiling

* Generates smooth trajectories in 2D space
* Supports:

  * Velocity constraints
  * Acceleration limits
* Outputs:

  * Time-parameterised trajectories
  * Target pose + velocity at each timestep

---
