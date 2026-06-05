# Two-Wheeled Self-Balancing Robot

A two-wheeled self-balancing robot operating as an inverted pendulum system,
using real-time IMU feedback, a complementary filter, and PID control to
maintain an upright position. Combines theoretical modeling, simulation,
and physical hardware implementation.
Built as a university course project at Egypt University of Informatics.

**April 2026 – June 2026**

---

## Overview

This project implements a fully functional self-balancing robot validated
from first principles through to physical prototype. The system was first
modeled analytically using Newton-Euler and Lagrange methods, linearized,
and represented in state-space form. A Simscape Multibody simulation was
built directly from the SolidWorks CAD model to validate behavior before
hardware assembly. On the physical robot, an MPU6050 IMU feeds tilt angle
data — filtered via a complementary filter — into a PID controller running
on an Arduino Uno, which drives two GA25-370 DC motors through an L298N
motor driver. The robot successfully detects tilt, corrects orientation,
and maintains balance for 5–8 seconds.

---

## Physical Model
<img width="769" height="574" alt="Solidworks Assembly" src="https://github.com/user-attachments/assets/13cf2c20-013b-4398-a590-dace72754c7f" />

<img width="941" height="1672" alt="Physical Model od Self-balancing Robot" src="https://github.com/user-attachments/assets/35027cc4-cdfb-4e9a-8827-cac6455e2cda" />

https://github.com/user-attachments/assets/1e3a8120-55f2-44b8-9ff1-c3eb335fe951


---

## Features

- Inverted pendulum dynamics modeled using Newton-Euler and Lagrange methods
- State-space representation derived from linearized equations of motion
- Simscape Multibody simulation built from SolidWorks CAD model
- MPU6050 IMU for real-time tilt angle measurement via I2C
- Complementary filter for accurate, noise-reduced angle estimation
- PID controller tuned through iterative hardware testing
- PWM motor control via L298N H-bridge driver
- 3-layer 3D-printed PLA chassis designed in SolidWorks
- Successfully balances for 5–8 seconds — theoretical modeling
  validated through practical implementation

---

## Hardware Components

| Component                  | Quantity |
|----------------------------|----------|
| Arduino Uno                | 1        |
| MPU6050 IMU                | 1        |
| L298N Motor Driver         | 2        |
| GA25-370 DC Motors         | 2        |
| 3D Printed PLA Chassis     | 1        |
| Battery Pack               | 1        |
| Breadboard / Wiring        | —        |

---

## Software & Tools

- Arduino C++ (control loop, PID, complementary filter)
- MATLAB / Simulink (system modeling and analysis)
- Simscape Multibody (3D dynamic simulation from CAD)
- SolidWorks (chassis design and 3D printing preparation)
- I2C communication protocol (MPU6050 interface)
- PWM motor control

---

## System Architecture

### 1. Mathematical Modeling
The robot is modeled as an inverted pendulum on a cart. Equations of
motion were derived using both Newton-Euler and Lagrange (energy) methods,
then linearized around the upright equilibrium point and expressed in
state-space form for analysis and control design.

### 2. Simulation
A Simscape Multibody model was built directly from the SolidWorks CAD
assembly to simulate dynamic behavior and validate the control approach
before hardware implementation.

### 3. Angle Estimation
The MPU6050 provides raw accelerometer and gyroscope data over I2C.
A complementary filter fuses both signals to produce a stable, accurate
tilt angle estimate — eliminating gyroscope drift and accelerometer noise.

### 4. PID Control
The filtered angle feeds into a PID controller that computes motor output
to correct tilt in real time. Gains (Kp, Ki, Kd) were tuned iteratively
through physical hardware testing.

### 5. Motor Control
The Arduino sends PWM signals to the L298N motor driver, which controls
the speed and direction of both DC motors independently to correct
the robot's balance.

---

## Known Limitations

- Balance duration of 5–8 seconds; longer balance requires further
  PID tuning and mechanical refinement
- L298N motor driver current limitations caused hardware issues during
  testing — managed through careful power distribution
- Motor direction reversal required debugging during hardware bring-up

---
## Team

Developed as a collaborative team project for a university course at
**Egypt University of Informatics** (2026).
