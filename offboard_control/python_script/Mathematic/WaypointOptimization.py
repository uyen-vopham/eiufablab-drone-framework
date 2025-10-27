#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# pip3 install osqp

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from OptimalTrajectory import OptimalTrajectory
from TrajectoryWaypoint import TrajectoryWaypoint

class OptimizeWaypoints():
    def __init__(self):
        self.file_path = ""
        print("✅ OptimizeWaypoints instance created!")

    def load_waypoints_from_csv(self, file_path):
        df = pd.read_csv(file_path)
        # waypoints = []
        # for _, row in df.iterrows():
        #     position = np.array([row['x'], row['y'], row['z']])
        #     velocity = np.array([row['vx'], row['vy'], row['vz']])
        #     waypoints.append(TrajectoryWaypoint(position, velocity))
        # # return waypoints
        # print("Number of waypoints found: ", waypoints)
        return df[['x', 'y', 'z']].to_numpy()

    def generate_trajectory_from_csv(self, waypoints_list, output_csv, aggressiveness=1.0, sample_hz=1):
        #read input data
        # print(f"📂 Reading CSV file: LALALALALALALA")
        waypoints = waypoints_list
        number_of_waypoints = len(waypoints)
        if number_of_waypoints < 2:
            raise ValueError("Need at least two waypoints to generate a trajectory.")

        # Create TrajectoryWaypoint objects
        waypoints_objects = []
        for i, pos in enumerate(waypoints):
            wp = TrajectoryWaypoint(tuple(pos))
            #constraint at start or end points
            if i == 0 or i == number_of_waypoints - 1:
                wp.add_hard_constraints(1, (0, 0, 0))  # Start with zero velocity
            waypoints_objects.append(wp)

        # solve minimum-snap math problem
        trajectory = OptimalTrajectory(5, 3, waypoints_objects) #bậc 5, 3D
        trajectory.solve(aggressiveness)                        # call solver QP/closed-form

        # Sample trajectory
        T = trajectory.end_time()
        t = np.linspace(0, T, int(T * sample_hz)+1)
        pos = np.array([trajectory.val(tt, order=0) for tt in t])
        vel = np.array([trajectory.val(tt, order=1) for tt in t])
        acc = np.array([trajectory.val(tt, order=2) for tt in t])

        # Save to CSV
        output_data = pd.DataFrame({
            'time': t,
            'x': pos[:, 0], 'y': pos[:, 1], 'z': pos[:, 2],
            'dx': vel[:, 0], 'dy': vel[:, 1], 'dz': vel[:, 2],
            'ddx': acc[:, 0], 'ddy': acc[:, 1], 'ddz': acc[:, 2]
        })
        output_data.to_csv(output_csv, index=False)
        print(f"Trajectory saved to {output_csv}")

        # 6️⃣ Vẽ quỹ đạo
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot(pos[:,0], pos[:,1], pos[:,2], label='Minimum-snap')
        # ax.scatter(waypoints[:,0], waypoints[:,1], waypoints[:,2],
        #            c='r', label='Waypoints')
        # ax.set_xlabel('X [m]')
        # ax.set_ylabel('Y [m]')
        # ax.set_zlabel('Z [m]')
        # ax.legend()
        # plt.tight_layout()
        # plt.show()

        return t, pos, vel, acc

    def check_dynamic_limits(self, vel, acc, vmax=4.0, amax=4.0):
        
        v_norm = np.linalg.norm(vel, axis=1)
        a_norm = np.linalg.norm(acc, axis=1)

        v_max_measured = np.max(v_norm)
        a_max_measured = np.max(a_norm)

        print(f"Max velocity: {v_max_measured:.2f} m/s (limit: {vmax} m/s)")
        print(f"Max acceleration: {a_max_measured:.2f} m/s² (limit: {amax} m/s²)")

        if v_max_measured > vmax:
            print("🚨 Cảnh báo: Vượt giới hạn vận tốc!")
        if a_max_measured > amax:
            print("🚨 Cảnh báo: Vượt giới hạn gia tốc!")

        return (v_max_measured <= vmax) and (a_max_measured <= amax)
# 

# if __name__ == "__main__":

#     output_csv = "waypoints.csv"
#     path_csv = [[-2.25058e-11, -0.0110615, 7],[10.9268, 11.0505, 6.99998],[21.8536, 22.112, 6.99992],[32.7804, 33.1735, 6.99983],[32.7804, 33.1735, 6.99983 ],
# [43.7072, 44.2351, 6.9997],[54.634, 55.2966, 6.99952],[54.634, 55.2966, 6.99952],[65.5607, 66.3581, 6.99932],
# [76.4875, 77.4197, 6.99907],[87.4143, 88.4812, 6.99878],[98.341, 99.5428,6.99846],[109.268, 110.604, 6.9981 ],
# [120.194, 121.666, 6.9977],[120.194, 121.666, 6.9977],[131.121, 132.727, 6.99726],[142.048, 143.789, 6.99679],
# [152.975, 154.851, 6.99627],[163.901, 165.912, 6.99572]] 

#     waypoints = np.array(path_csv)
#     optimization = OptimizeWaypoints()
#     t, pos, vel, acc = optimization.generate_trajectory_from_csv(waypoints,output_csv)
#     check_dynamic = optimization.check_dynamic_limits(vel, acc)
#     if not check_dynamic:
#         print("Trajectory exceeds dynamic limits. Please adjust waypoints or aggressiveness.")
#     else:
#         print("Trajectory is within dynamic limits.")
