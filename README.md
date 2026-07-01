# Starlink GNC Simulator

**A Simple Python Prototype for Guidance, Navigation & Control (GNC) Logic of Phased-Array User Terminals**

---

## Project Summary

This project is a self-contained, Python simulation that demonstrates core GNC capabilities required for user terminals (phased-array antennas) operating on dynamic platforms such as ships, aircraft, vehicles, or spacecraft.

It implements:

- Real-time **angular (attitude) and translational navigation** using simulated onboard sensors (IMU + GNSS).
- **Optimal state estimation** via an Extended Kalman Filter (EKF) for loosely-coupled GNSS/INS fusion and attitude determination.
- **Beam-steering logic** that tracks LEO satellites, performs intelligent hand-offs, and maximizes link integrity in challenging environments.
- High-fidelity modeling and simulation of platform dynamics, sensors, and a simplified satellite constellation.
- Clean software architecture with modular design, unit testing, and visualization — practices that translate directly to production C++ development.

The simulation runs a closed-loop scenario of a moving platform (e.g., a ship) continuously estimating its pose while pointing a phased-array antenna at the best available satellite.

---

## Features Demonstrated (Mapped to Job Requirements)

| Job Responsibility / Preferred Experience                  | How This Project Addresses It |
|------------------------------------------------------------|-------------------------------|
| Design and analysis of GNC logic for phased arrays | Full closed-loop GNC simulation with attitude estimation + beam pointing |
| Real-time angular & translational navigation               | IMU propagation + GNSS updates at 100 Hz |
| Optimal state estimation techniques                        | Extended Kalman Filter (loosely-coupled GNSS/INS + attitude) |
| Beam-steering logic + satellite hand-off + link integrity  | Explicit selection & hand-off algorithm maximizing elevation/quality |
| Support for ships, aircraft, spacecraft applications       | Configurable moving platform dynamics |
| Model derivation & high-fidelity simulation                | Sensor noise/bias models, quaternion kinematics, constellation model |
| Implementation, validation, unit testing of software       | Modular Python code + pytest unit tests |
| State estimation, attitude determination, robotics         | Quaternion-based attitude EKF |
| Inertial navigation systems (INS)                          | Full IMU-based dead reckoning |
| GNSS (loosely/tightly coupled concepts)                    | Loosely-coupled fusion (easily extensible to tightly-coupled) |
| Classical dynamics, modeling & simulation                  | Quaternion kinematics, NED frame, gravity, etc. |
| Linux development & clean software practices               | Virtualenv, requirements.txt, modular structure, tests |

---

## Project Structure
| Plot / Metric                    | What it represents                                      | Related to sat2?                          |
|----------------------------------|---------------------------------------------------------|-------------------------------------------|
| Platform Trajectory              | True vs Estimated position of the user terminal         | No                                        |
| Position Estimation Error        | Error of the platform navigation (GNSS/INS)             | No                                        |
| Satellite Tracking & Hand-offs   | Which satellite ID is being tracked over time           | Yes (shows when sat2 is selected)         |
| Tracked Satellite Elevation      | Elevation of the currently selected satellite           | Only when sat2 is selected                |
| Antenna Pointing Error           | How well the antenna is pointing at the current sat     | Only when sat2 is selected                |
| Link Integrity / Quality         | Simple quality metric (based on elevation) of current sat | Only when sat2 is selected              |

> **Note:** The simulator tracks only **one satellite at a time** (the best one according to the beam steering logic). Metrics related to sat2 only appear when satellite ID 2 is currently selected.