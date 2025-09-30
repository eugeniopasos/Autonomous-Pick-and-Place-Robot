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
robot = Robot()
robot.write_time(traj_time)
robot.write_motor_state(True)

robot.write_joints([0, 0, 0, 0])
time.sleep(traj_time)

# Initilize Joint position and Time Arrays
t = []
joint = []
ee = []

# Robot Movement
# Define base waypoints
positions = [[25, -100, 150,-60],
            [150, -80, 300,   0],
            [250, -115, 75, -45],
            [25, -100, 150, -60]] 
waypoints = []
for position in positions:
    waypoints.append(np.rad2deg(robot.get_ik(position)))



for waypoint in waypoints:          # Iterate through waypoints
    robot.write_joints(waypoint)    # Write joint values
    start_time = time.time()        # Start timer
    while time.time() - start_time < traj_time:
        #Make lists of joint positions, time, and end effector positions
        joint.append(robot.get_joints_readings()[0])        
        t.append(time.time())
        a = np.radians(robot.get_joints_readings()[0])
        ee.append(robot.get_ee_pos(a))