# Minimum Snap Trajectory Optimization for Drone using minsnap-trajectories library
# First, install the library: pip install minsnap-trajectories numpy matplotlib
# This library implements minimum snap trajectory generation suitable for quadrotors/drones.
# Reference: https://pypi.org/project/minsnap-trajectories/

import numpy as np
import minsnap_trajectories as ms
import matplotlib.pyplot as plt

# Define waypoints for the drone trajectory (3D positions at specific times)
# Example: Start at (0,0,10), intermediate at (10,0,10) with velocity and accel, end at (20,0,10)
refs = [
    ms.Waypoint(
        time=0.0,
        position=np.array([0.0, 0.0, 10.0]),  # Starting position (x,y,z)
    ),
    ms.Waypoint(
        time=8.0,
        position=np.array([10.0, 0.0, 10.0]),
        velocity=np.array([0.0, 5.0, 0.0]),  # Optional: specify velocity
        acceleration=np.array([0.1, 0.0, 0.0]),  # Optional: specify acceleration
    ),
    ms.Waypoint(
        time=16.0,
        position=np.array([20.0, 0.0, 10.0]),
        # Optional: higher derivatives like jerk can be specified
    ),
]

# Generate the piecewise polynomial trajectory
# Using closed-form algorithm, minimizing jerk and snap (orders 3 and 4), continuous up to jerk (order 3)
polys = ms.generate_trajectory(
    refs,
    degree=8,  # Polynomial degree (higher for smoother, but computationally heavier)
    idx_minimized_orders=(3, 4),  # Minimize jerk and snap
    num_continuous_orders=3,  # Ensure continuity in position, velocity, acceleration
    algorithm="closed-form",  # Or "constrained" for other cases
)

# Sample the trajectory derivatives (position, velocity, acceleration, etc.)
t_sample = np.linspace(0, 16, 200)  # Time points to sample
pva = ms.compute_trajectory_derivatives(polys, t_sample, 2)  # Up to acceleration (order 2)
position = pva[0, ...].T  # Shape: (num_samples, 3) for x,y,z
velocity = pva[1, ...].T
acceleration = pva[2, ...].T

# For drone-specific: Compute quadrotor states and inputs (thrust, etc.)
# Assumes a simple quadrotor model
t_quad = np.linspace(0, 16, 200)
states, inputs = ms.compute_quadrotor_trajectory(
    polys,
    t_quad,
    vehicle_mass=1.0,  # Drone mass in kg
    yaw="velocity",  # Yaw aligned with velocity direction
    drag_params=ms.RotorDragParameters(0.1, 0.2, 1.0),  # Rotor drag coefficients
)

# Plot the trajectory (position over time)
fig, axs = plt.subplots(3, 1, figsize=(10, 8))
axs[0].plot(t_sample, position[:, 0], label='x')
axs[0].plot(t_sample, position[:, 1], label='y')
axs[0].plot(t_sample, position[:, 2], label='z')
axs[0].set_ylabel('Position (m)')
axs[0].legend()

axs[1].plot(t_sample, np.linalg.norm(velocity, axis=1))
axs[1].set_ylabel('Speed (m/s)')

axs[2].plot(t_sample, np.linalg.norm(acceleration, axis=1))
axs[2].set_ylabel('Acceleration magnitude (m/s²)')
axs[2].set_xlabel('Time (s)')

plt.tight_layout()
plt.show()

# The 'states' includes position, velocity, attitude, etc., for drone control
# 'inputs' includes thrust and body rates for the drone actuators
print("Trajectory generated successfully. Use 'states' for position/orientation and 'inputs' for control.")