# Autonomous Spacecraft Docking & Trajectory Tracking via LQR

![MATLAB](https://img.shields.io/badge/Platform-MATLAB-orange)
![Control Theory](https://img.shields.io/badge/Control-State--Space%20%7C%20LQR-blue)
![Field](https://img.shields.io/badge/Domain-Aerospace%20%26%20Robotics-brightgreen)
![License](https://img.shields.io/badge/License-MIT-green)

Autonomous trajectory stabilization and terminal guidance of a heavy spacecraft docking module. The project implements a continuous-time **Linear Quadratic Regulator (LQR)** applied to a linearized 6-DOF planar dynamic model under gravitational influence.

---

## 🚀 Dynamic Simulation

<!-- Uncomment when your GIF is uploaded to the assets folder -->
<!-- <p align="center">
  <img src="assets/trajectory_animation.gif" alt="Rocket Docking Trajectory" width="600">
</p> -->

The simulation tracks the spacecraft from an initial offset state $[x_0, y_0] = [1000\,\text{m}, 1000\,\text{m}]$ with non-zero initial drift velocities down to a stabilized precision docking point at the origin $[0, 0]$ with zero terminal attitude error ($\phi = 0$).

---

## 📐 Mathematical Formulation & State-Space Model

The continuous-time linear time-invariant (LTI) system is formulated as:

$$\dot{x}(t) = A x(t) + B u(t), \quad y(t) = C x(t) + D u(t)$$

### 1. State and Input Vectors
* **State Vector:**
  $$x = \begin{bmatrix} x & \dot{x} & y & \dot{y} & \phi & \dot{\phi} \end{bmatrix}^T$$
  Where $x, y$ are planar Cartesian coordinates, $\dot{x}, \dot{y}$ are translational velocities, $\phi$ is the pitch angle, and $\dot{\phi}$ is the angular body rate.

* **Control Input Vector:**
  $$u = \begin{bmatrix} u_y & u_\phi \end{bmatrix}^T$$
  Where $u_y$ is the axial vertical thrust force and $u_\phi$ is the attitude reaction control torque.

### 2. System Matrices
Given vehicle mass $m = 1.6 \times 10^6\,\text{kg}$, characteristic radius $R = 35\,\text{m}$, and moment of inertia $I = \frac{m(2R)^2}{12}$:

$$A = \begin{bmatrix} 
0 & 1 & 0 & 0 & 0 & 0 \\ 
0 & 0 & 0 & 0 & g & 0 \\ 
0 & 0 & 0 & 1 & 0 & 0 \\ 
0 & 0 & 0 & 0 & 0 & 0 \\ 
0 & 0 & 0 & 0 & 0 & 1 \\ 
0 & 0 & 0 & 0 & 0 & 0 
\end{bmatrix}, \quad
B = \begin{bmatrix} 
0 & 0 \\ 
0 & g \\ 
0 & 0 \\ 
\frac{1}{m} & 0 \\ 
0 & 0 \\ 
0 & \frac{m \cdot g \cdot R}{I} 
\end{bmatrix}$$

---

## 🎛️ Optimal Controller Synthesis (LQR)

The infinite-horizon cost function balances state tracking accuracy against actuator effort:

$$J = \int_{0}^{\infty} \left( x(t)^T Q x(t) + u(t)^T R u(t) \right) dt$$

### Tuning Strategy & Matrix Weights
* **Attitude Penalty:** Angular deviations ($\phi, \dot{\phi}$) are heavily penalized ($10^{11}$) to enforce strict orientation alignment during docking and avoid tumbling.
* **Position & Velocity:** $x$ and $\dot{x}$ are prioritized over vertical descent rates to ensure horizontal alignment prior to touchdown.
* **Effort Regularization:** The input weight matrix $R$ is scaled to avoid actuator saturation while guaranteeing exponential convergence:

$$Q = \text{diag}([10^5, \, 10^6, \, 10^3, \, 10^5, \, 10^{11}, \, 10^{11}]), \quad R = \text{diag}([10^{-12}, \, 1])$$

The feedback gain matrix $K$ is solved using the Algebraic Riccati Equation (ARE), yielding the closed-loop system:

$$\dot{x} = (A - BK)x$$

---

## 💻 How to Run

1. Clone this repository:
   ```bash
   git clone [https://github.com/your-username/spacecraft-docking-lqr-control.git](https://github.com/your-username/spacecraft-docking-lqr-control.git)
