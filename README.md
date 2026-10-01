# Iris

**Iris** is a drone trajectory tracking system developed with MATLAB/Simulink as part of a cyber-physical systems project.

The project focuses on controlling a quadcopter and making it follow predefined trajectories using a feedback control architecture. The dynamics of the system are modeled using the **HyEQ Toolbox**.

## Features

- Quadcopter dynamic model
- Trajectory generation
- LQR-based control
- Trajectory tracking
- Hybrid system simulation
- Disturbance and wind modeling
- MATLAB/Simulink simulation
- Visualization of desired and simulated trajectories

## Requirements

- MATLAB
- Simulink
- Control System Toolbox
- **HyEQ Toolbox**

Make sure the **HyEQ Toolbox** is installed and added to your MATLAB path before running the Simulink model.

## Usage

Run:

```matlab
init
```
The initialization script generates the reference trajectory, loads the drone parameters and computes the LQR controller gains required by the simulation.

Then open and run:

```text
drone_HyEQ.slx
```

Once the simulation is complete, run:
```text
test
```
test.m generates plots for the drone's position, velocity and attitude, compares simulated states with their references, and displays the finite-state machine state used in the simulation.

## Simulation
<img width="784" height="472" alt="image" src="https://github.com/user-attachments/assets/16f47bdf-d028-4865-8d95-64814cb7bdd4" />
<img width="712" height="491" alt="image" src="https://github.com/user-attachments/assets/630ff1f7-5ea5-4e3d-b143-cd56b55d194e" />

🌱
