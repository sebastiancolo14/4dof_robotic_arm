# Redundant 4-DOF Manipulator on LEGO Mindstorms EV3: Kinematics and Joint-Limit Avoidance

Modelling, simulation and hardware validation of a 4-DOF kinematically redundant manipulator built with a LEGO Mindstorms EV3 kit. The project was developed for the Biomedical Robotics course (a.y. 2025/2026).

## Overview

- **Modelling:** Denavit-Hartenberg (DH) parametrisation of the manipulator.
- **Kinematics:** forward, inverse and differential kinematics, all implemented from scratch in Python.
- **Task:** end-effector trajectory in operational space, composed of two segments.
- **Redundancy resolution:** a null-space term of the Jacobian keeps the joints clear of their mechanical limits without altering the end-effector task.
- **Validation:** simulation first, then the physical manipulator.

## Objectives

1. Build a 4-DOF redundant manipulator with the LEGO EV3 platform.
2. Model the system with DH parameters.
3. Plan an operational-space trajectory made of two segments.
4. Implement forward and inverse kinematics algorithms.
5. Exploit kinematic redundancy through a null-space term of the Jacobian.
6. Define a functional that keeps the joints away from their mechanical limits.
7. Compare the functional applied to 3 joints versus 4 joints.
8. Validate the results in simulation and on the real manipulator.

## Platform

Manipulator built with a LEGO Mindstorms EV3 kit, using four servo motors with integrated encoders.

DH link parameters:

| Parameter | Value |
|---|---|
| l1 | 0.9 m |
| l2 | 0.005 m |
| l3 | 0.035 m |
| l4 | 0.02 m |
| d3 | 0.05 m |

## Method

### Kinematics and trajectory

- DH-based forward kinematics.
- Operational-space trajectory planning with two segments.
- Differential inverse kinematics, with the redundancy handled through a null-space projection of the Jacobian.

### Redundancy exploitation

Kinematic redundancy is used to keep the joints away from their mechanical limits throughout the trajectory. This is a safety and wear requirement, not merely a formal nicety. The secondary objective is a functional that is minimised in the null space of the Jacobian, so the end-effector task is left untouched.

The functional was tested in two variants:

- applied to the **3 joints** that have real joint limits;
- extended to all **4 joints**, including the fourth joint, which has no real travel constraint.

### Evaluation metrics

Joint configuration quality is evaluated with dedicated metrics:

- minimum normalised distance to the joint limits;
- mean normalised distance to the joint limits;
- percentage of time spent in the critical band near the limits.

### Gain tuning

Gains were tuned experimentally under a multi-objective criterion that balances tracking accuracy against joint configuration quality. The three configurations compared:

| Configuration | k | ka |
|---|---|---|
| 1 | 180 | 0 |
| 2 | 180 | 15 |
| 3 | 180 | 10 |

The optimised gains keep the tracking error low while improving the joints' distance from their limits.

## Simulation Results

- In all three cases the trajectory is followed with a tracking error on the order of 10⁻⁷ m.
- The null-space term changes the joints' behaviour without altering the end-effector task.
- Effect of the optimisation on the joints:
  - **q1:** moves closer to its limit;
  - **q2:** behaviour essentially unchanged;
  - **q3:** pushed towards the middle of its range.
- **Best case:** the functional applied to the 3 joints with real limits, which outperforms the extension to the fourth joint.

## Simulation-to-Hardware Transfer

Moving from simulation to the physical robot required:

- runtime timing management: `dt` is not fixed but measured at every control cycle;
- online trajectory planning;
- encoder feedback;
- non-blocking actuation.

## Hardware Results

The physical manipulator followed the trajectory in the correct direction, confirming the feasibility of the approach. The motion showed significant oscillations around the ideal path, attributable to:

- mechanical backlash;
- encoder quantisation;
- latency introduced by non-blocking control.

## Conclusions

- The 4-DOF redundant manipulator was successfully built and modelled with the LEGO EV3 platform.
- Forward and inverse kinematics allow the two-segment trajectory to be tracked with very low error in simulation (about 10⁻⁷ m).
- The null-space redundancy term shapes the joints' behaviour without changing the end-effector task.
- Applying the functional to the three limited joints gave better results than extending it to the fourth, which has no real travel constraint.
- Hardware tests confirmed feasibility, but the oscillations show that backlash, encoder resolution and control latency are the main limits of the platform.

## Requirements

- Python 3
- NumPy, Matplotlib
- EV3 interface library for Python (to be specified, e.g. `ev3dev2` / `pybricks`)

## Repository Structure

```
.
├── kinematics/      # DH model, forward, inverse and differential kinematics
├── simulation/      # trajectory planning, redundancy resolution, gain tuning
├── hardware/        # EV3 control
