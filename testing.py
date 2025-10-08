<<<<<<< HEAD
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
=======
from OptimalTrajectory import OptimalTrajectory
from TrajectoryWaypoint import TrajectoryWaypoint
import numpy as np
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import Header
import rospy
from math import atan2, asin


class DroneGate:
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation


class DroneTrajectory:
    def __init__(self):
        self.gates = []
        self.waypoints = []
        self.splines = []
        self.start_pos = None
        self.start_velocity = None
        self.end_pos = None
        self.end_velocity = None
        self.trajectory = None
        self.direction_radius = 0.1

        self.spacing = 0.2
        self.ext_radius = 0.15  # ensure that this is less than spacing/sqrt(3) to properly handle diagonal gates.
        self.int_radius = 0

    def set_start(self, position, velocity):
        if isinstance(position, np.ndarray):
            position = position.ravel()
        if isinstance(velocity, np.ndarray):
            velocity = velocity.ravel()
        self.start_pos = list(position)
        self.start_velocity = velocity

    def set_end(self, position, velocity):
        if isinstance(position, np.ndarray):
            position = position.ravel()
        if isinstance(velocity, np.ndarray):
            velocity = velocity.ravel()
        self.end_pos = position
        self.end_velocity = velocity

    def clear_gates(self):
        self.gates = []

    def add_gate(self, position, orientation):
        if isinstance(position, np.ndarray):
            position = position.ravel()
        self.gates.append(DroneGate(position, orientation))

    def solve(self, aggressiveness):
        if self.start_pos is None:
            return None
        if len(self.gates) == 0 and self.end_pos is None:
            return None

        self.waypoints = []

        gate_waypoints = []
        # outer guiding waypoints have lower constraint, full equality constraint at center
        radii = [self.ext_radius, self.int_radius, 0, self.int_radius, self.ext_radius]
        guide_spacing = self.spacing * np.arange(start=-1, stop=1+1)
        for gate in self.gates:
            rotm = Rotation.from_quat(gate.orientation).as_dcm()
            dirvec = rotm.dot(np.array([1, 0, 0]))
            for ri, offset in enumerate(guide_spacing):
                radius = radii[ri]
                guide_pos = rotm.dot(np.array([offset, 0, 0])) + np.array(gate.position).transpose()
                if np.isclose(offset, 0, atol=1e-9):
                    # if is true gate, don't allow soft constraint
                    guide_wp = TrajectoryWaypoint(tuple(guide_pos.ravel()))
                    guide_wp.add_soft_directional_constraint(1, tuple(dirvec), self.direction_radius)
                    gate_waypoints.append(guide_wp)
                else:
                    guide_wp = TrajectoryWaypoint(tuple(guide_pos.ravel()))
                    # guide_wp.add_soft_directional_constraint(1, tuple(dirvec), self.direction_radius)
                    # gate_waypoints.append(guide_wp)


        start_waypoint = TrajectoryWaypoint(tuple(self.start_pos))
        start_waypoint.add_hard_constraints(1, tuple(self.start_velocity))

        self.waypoints.append(start_waypoint)
        self.waypoints.extend(gate_waypoints)

        if self.end_pos is not None:
            end_waypoint = TrajectoryWaypoint(tuple(self.end_pos))
            if self.end_velocity is not None:
                end_waypoint.add_hard_constraints(1, tuple(self.end_velocity))
            self.waypoints.append(end_waypoint)

        self.trajectory = OptimalTrajectory(5, 3, self.waypoints)
        self.trajectory.solve(aggressiveness)

    def val(self, t, order=0, dim=None):
        if self.trajectory is None:
            return None
        last_time = self.trajectory.end_time()
        if t > last_time:
            t = last_time
        return self.trajectory.val(t, dim, order)
    
    def full_pose(self, time_elapsed):
        pos = self.val(time_elapsed)
        vel = self.val(time_elapsed, order=1)
        unit_vec = np.array(vel) / np.linalg.norm(np.array(vel))

        psi = atan2(unit_vec[1], unit_vec[0])
        theta = asin(-unit_vec[2])
        q = Rotation.from_euler('ZYX', [psi, theta, 0]).as_quat()
        
        return pos, vel, q

    def as_path(self, dt, start_time, frame='odom'):
        if self.trajectory is None:
            return None
        ts = np.arange(0, self.trajectory.end_time(), dt)

        poses = []
        for t in ts:
            pos = self.val(t)
            vel = self.val(t, order=1)

            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp = start_time + rospy.Duration(t)
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]

            vel = np.array(vel) / np.linalg.norm(np.array(vel))

            psi = atan2(vel[1], vel[0])
            theta = asin(-vel[2])
            q = Rotation.from_euler('ZYX', [psi, theta, 0]).as_quat()
            pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            poses.append(pose)

        path = Path()
        path.header.frame_id = frame
        path.header.stamp = start_time
        path.poses = poses

        return path

