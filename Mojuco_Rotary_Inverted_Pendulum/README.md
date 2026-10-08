# MuJoCo Rotary Inverted Pendulum Model

The shared XML model used by this repository's PID, LQR, and reinforcement-learning controller experiments.

[Open the model](rotary%20inverted%20pendulum.xml) to review the geometry, physical parameters, joints, actuators, and visualization settings before changing a controller.

## Usage

Install the repository's Python dependencies and run a controller as described in the [main README](../README.md). Controller scripts resolve this model relative to their own locations; no duplicate XML files are required.

Keep the model and controller assumptions consistent when changing physical parameters. Simulation results alone do not validate behavior on a physical pendulum.
