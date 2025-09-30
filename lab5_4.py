import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

from Robot import Robot
import time
import numpy as np
import matplotlib.pyplot  as plt
import pickle
from TrajPlanner import TrajPlanner

# Robot Setup
traj_time = 4
tolerance = 10
robot = Robot()
robot.write_time(traj_time)
robot.write_motor_state(True)
initial = robot.get_ik([25, -100, 150, 0])
robot.write_joints(np.rad2deg(initial))

time.sleep(traj_time)
robot.write_mode("velocity")




# Initilize Joint position and Time Arrays
t = []
joint = []
ee = []
vel = []
ee_vel = []

# Robot Movement
# Define base waypoints
targets =  [#[25, -100, 150, 0],
            [150, 80, 300,   0],
            [250, -115, 75,  0],
            [25, -100, 150,  0]] 

for target in targets:          # Iterate through waypoints   
    norm = 20
    while norm > tolerance:
        j = np.radians(robot.get_joints_readings())
        ee_pose = robot.get_ee_pos(j[0])
        distance = target[:3] - ee_pose[:3]
        norm = np.linalg.norm(distance)
        direction = distance/norm
        velocities = 50*direction
        J = robot.get_jacobian(j[0])     
        Jinv = np.linalg.pinv(J[:3])
        j_velocities = Jinv @ velocities
        robot.write_velocities(j_velocities)    # Write joint values
        ee_velocity = robot.get_forward_diff_kinematics(j[0],j[1] )
        #Make lists of joint positions, time, and end effector positions
        ee_vel.append(ee_velocity)
        vel.append(j[1])
        joint.append(robot.get_joints_readings()[0])        
        t.append(time.time())
        a = np.radians(robot.get_joints_readings()[0])
        ee.append(robot.get_ee_pos(a))

    robot.write_velocities([0, 0, 0, 0])
robot.write_velocities([0, 0, 0, 0])

data_to_save = {
    'joint_angles': np.array(joint),
    'ee_positions': np.array(ee),
    'timestamps': np.array(t),
    'joint_velocities':np.array(vel),
    'ee_velocities': np.array(ee_vel)
}

# Save Data to pickle File
def save_to_pickle(data, filename):
    with open(filename, 'wb') as file:
        pickle.dump(data, file)

# Loading data from a pickle file
def load_from_pickle(filename):
    with open(filename, 'rb') as file:
        return pickle.load(file)

save_to_pickle(data_to_save, 'lab_5_data1.pkl')