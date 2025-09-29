# Husky Differential-Drive Modeling & Control

This repository provides a structured MATLAB framework for modeling, simulating, and controlling a Husky-like nonholonomic robot. It is designed as a **teaching/learning reference** and as a **foundation for advanced controllers (LQR, MPC, NMPC, etc.)**.

---

## Contents
- [Goals](#goals)
- [Installation](#installation)
- [Prerequisites](#prerequisites)
- [Project Layout](#project-layout)
- [Conventions](#conventions)
- [Robot Model Summary](#robot-model-summary)
- [Typical Parameters](#typical-parameters)
- [Quick Start](#quick-start)
- [Tasks (roadmap)](#tasks-roadmap)
- [Validation Checklist](#validation-checklist)
- [Tuning Guide](#tuning-guide)
- [Troubleshooting](#troubleshooting)
- [Future Extensions](#future-extensions)
- [References](#references)
- [Changelog](#changelog)

---

## Goals
- Build a **clear MATLAB baseline** for differential-drive (unicycle) kinematics.
- Implement **six incremental tasks**: simulate → verify constraint → go-to-point → line following → pure pursuit → go-to-pose.
- Keep code **modular** with a minimal utility library reusable for future controllers.

---

## Installation
1. Clone the repo:
   ```bash
   git clone https://github.com/your-username/husky-diffdrive.git
   cd husky-diffdrive
   ```
2. Add to MATLAB path:
   ```matlab
   addpath(genpath('husky'));
   savepath; % optional
   ```

---

## Prerequisites
- **MATLAB** R2021a or newer (no toolboxes required).
- Familiarity with vectors, angles (radians), and plotting.
- Optional: ROS/ROS2 if you later deploy to a physical robot.

---

## Project Layout
```
husky/
├─ README.md
├─ husky_utils.m                   # Task 0: common utilities (angles, clamps, kinematics, mappings)
├─ husky_task1_model.m             # Task 1: kinematics simulation
├─ husky_task2_constraint.m        # Task 2: nonholonomic constraint check
├─ husky_task3_gotopoint.m         # Task 3: go-to-point (x,y)
├─ husky_task4_linefollow.m        # Task 4: follow a line ax+by+c=0
├─ husky_task5_purepursuit.m       # Task 5: pure pursuit path following
└─ husky_task6_gotopose.m          # Task 6: go-to-pose (x,y,θ)
```

---

## Conventions
- **Frames:** world frame $\{W\}$ is inertial; body heading $\theta$ is measured from $+x_W$.
- **State:** $x = [x, y, \theta]^\top$.
- **Inputs (unicycle):** linear $v$ [m/s], angular $\omega$ [rad/s].
- **Units:** meters, seconds, radians.
- **Angle wrapping:** always wrap angles to $(-\pi, \pi]$ using `wrapToPi()`.

---

## Robot Model Summary

**Unicycle (diff-drive) kinematics**

$$
\dot{x} = v \cos\theta, \quad 
\dot{y} = v \sin\theta, \quad 
\dot{\theta} = \omega
$$

**Nonholonomic (no-sideways) constraint**

$$
\dot{y}\cos\theta - \dot{x}\sin\theta = 0
$$

**Wheel geometry**
- wheel radius $r$, track width $W$ (center-to-center).

**Unicycle ↔ Wheel mapping**

$$
v = \frac{v_R + v_L}{2}, \quad \omega = \frac{v_R - v_L}{W}
$$

$$
v_R = v + \frac{W}{2}\,\omega, \quad v_L = v - \frac{W}{2}\,\omega
$$

$$
\dot{\phi}_{R,L} = \frac{v_{R,L}}{r}
$$

---

## Typical Parameters
Defaults (Husky-like):
| Symbol | Meaning                              | Default |
|-------:|--------------------------------------|:-------:|
| `r`    | wheel radius [m]                     | 0.165   |
| `W`    | track width [m]                      | 0.55    |
| `vMax` | max linear speed [m/s]               | 1.0     |
| `wMax` | max angular speed [rad/s]            | 1.5     |

In MATLAB:
```matlab
P = husky_utils.params('r',0.165,'W',0.55,'vMax',1.0,'wMax',1.5);
```

---

## Quick Start
1. Add folder:
   ```matlab
   addpath(genpath('husky'));
   ```
2. Run a short simulation (constant \(v,\omega\)):
   ```matlab
   P = husky_utils.params();
   x = [0;0;0]; dt = 0.01; v = 0.6; w = 0.2;
   N = 1000; X = zeros(3,N); T = (0:N-1)*dt;
   for k = 1:N
       x = husky_utils.rk4step_unicycle(x, v, w, dt);
       X(:,k) = x;
   end
   plot(X(1,:), X(2,:)); axis equal; grid on; title('Husky path');
   ```
3. Proceed with tasks in order: `husky_task1_model.m` → … → `husky_task6_gotopose.m`.

---

## Tasks (roadmap)

### **Task 0 — Utilities**  (`husky_utils.m`)
- Angle tools: `wrapToPi`, `angdiff`
- Saturation: `sat`
- Kinematics: `eulerstep_unicycle`, `rk4step_unicycle`
- Conversions: `uni2wheels`, `wheels2uni`, `wheelAngularSpeeds`
- Diagnostics: `lateralVelocityWorld`

### Task 1 — Kinematic Simulation
**File:** `husky_task1_model.m`

- Integrate the unicycle model with constant / time-varying $(v, \omega)$.
- Plots: trajectory $(x, y)$, heading $\theta(t)$, and inputs.
- Demos:
  - Straight line: $\omega = 0$
  - Circle: constant $(v, \omega)$

### Task 2 — Nonholonomic Constraint
**File:** `husky_task2_constraint.m`

- Residual: $\dot{y}\cos\theta - \dot{x}\sin\theta$  
- Validate: residual $\approx 0$

### Task 3 — Go-to-Point (x,y)
**File:** `husky_task3_gotopoint.m`

- Errors:  
  $\rho = \sqrt{(x_g - x)^2 + (y_g - y)^2}$  
  $\alpha = \text{wrap}\!\big(\text{atan2}(y_g - y, x_g - x) - \theta\big)$  
- Control: $v = k_\rho \rho$, $\omega = k_\alpha \alpha$  
- Tune: $k_\rho \in [0.5, 1.5]$, $k_\alpha \in [1, 3]$  
- Stop if $\rho < \varepsilon$


### Task 4 — Line Following
**File:** `husky_task4_linefollow.m`

- Cross-track: $d = \frac{ax + by + c}{\sqrt{a^2 + b^2}}$  
- Heading error: $\theta_e = \text{wrap}(\theta^* - \theta)$  
- Control: $v = v_0$, $\omega = -K_d d + K_\theta \theta_e$



### Task 5 — Pure Pursuit
**File:** `husky_task5_purepursuit.m`

- Target point at look-ahead $L_d$  
- Curvature: $\kappa = \frac{2 \sin \alpha}{L_d}$  
- Command: $\omega = v \kappa$


### Task 6 — Go-to-Pose
**File:** `husky_task6_gotopose.m`

- Errors:  
  $\rho = \sqrt{(\Delta x)^2 + (\Delta y)^2}$  
  $\alpha = \text{wrap}\!\big(\text{atan2}(\Delta y, \Delta x) - \theta\big)$  
  $\beta = \text{wrap}\!\big(\theta_g - \theta - \alpha\big)$  
- Control: $v = k_\rho \rho$, $\omega = k_\alpha \alpha + k_\beta \beta$  
- Stable if: $k_\rho > 0,\ k_\beta < 0,\ k_\alpha > k_\rho$

---

## Validation Checklist
- **Task 1:** path matches line / circle  
- **Task 2:** residual $\approx 0$  
- **Task 3:** $\rho(t) \to 0$  
- **Task 4:** $d, \theta_e \to 0$  
- **Task 5:** smooth path tracking  
- **Task 6:** $\rho, \alpha, \beta \to 0$

---

## Tuning Guide
- **Saturation first:** clip $v, \omega \leq$ (`vMax`, `wMax`)  
- **Go-to-Point:** start small, raise $k_\rho$ for speed, adjust $k_\alpha$  
- **Line follow:** small $K_d$, moderate $K_\theta$  
- **Pure pursuit:** $L_d \approx 1$–$2 \times$ robot length  
- **Go-to-Pose:** good start → $(k_\rho, k_\alpha, k_\beta) = (1.0, 2.0, -1.0)$

---

## Troubleshooting
- **Plot errors:** pre-allocate arrays, ensure time sync  
- **Angle jumps:** always wrap with `wrapToPi`  
- **Spins in place:** reduce $k_\alpha$, add minimum velocity  
- **Chatter:** lower gains or increase $L_d$

---

## Future Extensions
- 🟦 ROS2 bridge (`/cmd_vel`, odometry feedback).
- 🟦 Gazebo/Webots simulation.
- 🟦 LQR, MPC, NMPC controllers.
- 🟦 Mobile manipulation (Husky + UR5).

---

## References
- Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press. [modernrobotics.org](http://modernrobotics.org)
- Corke, P. (2017). *Robotics, Vision & Control*. Springer.
- Clearpath Husky: [clearpathrobotics.com](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/)

---

## Changelog
- **v0.2** — Improved structure, added Installation, References, Future Extensions.  
- **v0.1** — Initial reference README.
