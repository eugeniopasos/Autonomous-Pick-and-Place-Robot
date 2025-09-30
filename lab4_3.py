import sys
import os

# Add the 'classes' directory to the PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

import numpy as np
import matplotlib.pyplot as plt
from Robot import Robot
from TrajPlanner import TrajPlanner
import time
import pickle

def collect_data():
    """
    Collects data for the robot's movement and saves it to a pickle file.
    """
    traj_time = 5  # Defines the trajectroy time
    points_num = 998 # Defines the number if intermediate waypoints for a trajectory
    robot = Robot()  # Creates robot object

    # Define setpoints
    ee_poses = np.array([
        [25, -100, 150, -60],
        [150, 80, 300, 0],
        [250, -115, 75, -45],
        [25, -100, 150, -60]
    ])
    # try:
    #     joint_angles = np.array([
    #         robot.get_ik(ee_poses[0, :]),
    #         robot.get_ik(ee_poses[1, :]),
    #         robot.get_ik(ee_poses[2, :]),
    #         robot.get_ik(ee_poses[0, :])
    #     ])
    # except ValueError as e:
    #     raise ValueError("End-Effector Pose Unreachable: " + str(e))
    
    # Create Trajectory between setpoint angles
    tj = TrajPlanner(ee_poses)
    trajectories = tj.get_quintic_traj(traj_time, points_num)
    time_step = trajectories[2,0]-trajectories[1,0]

    print(f'time_step={time_step}')

    # Pre-allocate data
    data_time = np.zeros(points_num+2)
    data_ee_poses = np.zeros((points_num+2, 4))
    data_q = np.zeros((points_num+2, 4))
    count = 0

    # Send to first vertex to start
    robot.write_motor_state(True)  # Write position mode
    
    robot.write_time(traj_time)
    robot.write_joints(np.rad2deg(robot.get_ik(trajectories[0, 1:])))
    time.sleep(traj_time)  # Wait for trajectory completion

    robot.write_time(time_step)
    start_time = time.time()

    # Move the robot along all trajectories
    for i in range(1, len(trajectories)):
        robot.write_joints(np.rad2deg(robot.get_ik(trajectories[i, 1:])))  # Write joint values
        # Collect a reading periodically until the waypoint is reached
        while time.time() - start_time < (i * time_step):
            data_q[count, :] = robot.get_joints_readings()[0, :]
            data_time[count] = time.time() - start_time
            data_ee_poses[count, :] = robot.get_ee_pos(data_q[count, :])[0:4]
            count += 1

    # Trim unused space in data
    data_time = data_time[:count]
    data_ee_poses = data_ee_poses[:count, :]
    data_q = data_q[:count, :]

    # Save data to a picke file (TODO)
    data_to_save = {
    'joint_angles': np.array([data_q]),
    'ee_positions': np.array([data_ee_poses]),
    'timestamps': np.array([data_time])
}
    save_to_pickle(data_to_save, 'data_lab_4_quintic.pkl')

def save_to_pickle(data, filename):
    with open(filename, 'wb') as file:
        pickle.dump(data, file) 

def plot_data():
    """
    Loads data from a pickle file and plots it.
    """


    

if __name__ == "__main__":
    
    collect_data()
    plot_data()
