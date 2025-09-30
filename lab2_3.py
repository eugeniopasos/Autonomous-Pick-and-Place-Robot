import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))
from Robot import Robot

import numpy  as np
import time

# Robot Setup
traj_time = 3
robot = Robot()
robot.write_time(traj_time)
robot.write_motor_state(True)

robot.write_joints([0, 0, 0, 0])
print(robot.get_ee_pos([0,0,0,0]))
time.sleep(traj_time)

# Initilize Joint position and Time Arrays
t = []
joint = []

# Robot Movement
robot.write_joints([-90, 15, 30, -45])  # Write joint values
start_time = time.time()                # Start timer
while time.time() - start_time < traj_time:
    # Print forward kinematics and end effector pose
    print(robot.get_current_fk())
    a = np.radians(robot.get_joints_readings()[0])
    print(robot.get_ee_pos(a))
time.sleep(1)

