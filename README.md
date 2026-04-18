
# 6-DOF Rocket Trajectory with Pitch Program Optimization

[![Difficulty](https://img.shields.io/badge/Difficulty-★★★★☆-orange)]()
[![Topic](https://img.shields.io/badge/Topic-Aerospace-blue)]()
[![Solver](https://img.shields.io/badge/Solver-ODE45%20%7C%20FSOLVE-green)]()

## 🎯 Problem Statement

A two-stage liquid-fuel rocket launches from the surface and must achieve a **200 km circular orbit apogee** at engine cutoff. The rocket's pitch angle follows a quadratic profile:

$$\theta(t) = a + bt + ct^2$$

Find the coefficients $(a, b, c)$ that achieve the target apogee while maintaining attitude stability.

### Key Constraints
- Maximum dynamic pressure ≤ 50 kPa
- Angle of attack ≤ 5°
- Structural load factor ≤ 4g

## 📐 Mathematical Model

### States (14 total)

| State | Symbol | Unit | Description |
|-------|--------|------|-------------|
| Position | $x, y, z$ | m | ECI frame |
| Velocity | $u, v, w$ | m/s | ECI frame |
| Quaternion | $q_0, q_1, q_2, q_3$ | - | Attitude |
| Angular velocity | $p, q, r$ | rad/s | Body frame |
| Mass | $m$ | kg | Time-varying |

### Governing Equations

#### Translational Dynamics
$$\dot{\mathbf{r}} = \mathbf{v}$$
$$\dot{\mathbf{v}} = \frac{\mathbf{T} + \mathbf{D} + \mathbf{G}}{m}$$

where:
- $\mathbf{T}$ = Thrust vector (body frame → inertial via quaternion)
- $\mathbf{D}$ = Drag force $= -\frac{1}{2}\rho V^2 C_D A \frac{\mathbf{v}}{V}$
- $\mathbf{G}$ = Gravity $= -\frac{GM}{r^2} \frac{\mathbf{r}}{r}$

#### Attitude Dynamics (Quaternion)
$$\dot{\mathbf{q}} = \frac{1}{2} \boldsymbol{\Omega}(\boldsymbol{\omega}) \mathbf{q}$$

$$\boldsymbol{\Omega}(\boldsymbol{\omega}) = \begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y \\
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}$$

#### Rotational Dynamics
$$\dot{\boldsymbol{\omega}} = \mathbf{I}^{-1} \left( \boldsymbol{\tau} - \boldsymbol{\omega} \times \mathbf{I}\boldsymbol{\omega} \right)$$

#### Mass Depletion
$$\dot{m} = -\dot{m}_{fuel}$$

### Atmosphere Model (Exponential)

$$\rho(h) = \rho_0 e^{-h/H}$$

where $\rho_0 = 1.225$ kg/m³, $H = 8500$ m

## 🚀 How to Run

### Basic Execution
```matlab
>> cd Problem1_Rocket_6DOF
>> rocket_main
