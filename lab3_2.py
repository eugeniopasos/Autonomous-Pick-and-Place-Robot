import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

from Robot import Robot
import time
import numpy as np
import matplotlib.pyplot  as plt
import pickle

# Robot Setup
# traj_time = 4
# robot = Robot()
# robot.write_time(traj_time)
# robot.write_motor_state(True)

# robot.write_joints([0, 0, 0, 0])
# time.sleep(traj_time)

# Initilize Joint position and Time Arrays
t = []
joint = []
ee = []

# Robot Movement
# Define base waypoints
waypoints = [[150, -80, 300,   0]]  

# for waypoint in waypoints:          # Iterate through waypoints
#     robot.write_joints(waypoint)    # Write joint values
#     start_time = time.time()        # Start timer
#     while time.time() - start_time < traj_time:
        # Make lists of joint positions, time, and end effector positions
        # joint.append(robot.get_joints_readings()[0])        
        # t.append(time.time())
        # a = np.radians(robot.get_joints_readings()[0])
        # ee.append(robot.get_ee_pos(a))

def get_ik(p):
    l1 = 77
    l2 = 130
    l3 = 124
    l4 = 126
    l21 = 128
    l22 = 24
    r = np.sqrt(p[0]**2 + p[1]**2)
    rw = r - l4*np.cos(np.radians(p[3]))
    zw = p[2] - l1 - l4*np.sin(np.radians(p[3]))
    dw = np.sqrt(rw**2 + zw**2)

    u = np.arctan2(zw, rw)
    cosb = (l2**2 + l3**2 - dw**2) / (2*l2*l3)
    sinb = np.sqrt(1 - cosb**2)
    cosg = (dw**2 + l2**2 - l3**2) / (2*dw*l2)
    sing = np.sqrt(1 - cosg**2)
    
    b = np.arctan2(sinb, cosb)
    g = np.arctan2(sing, cosg)
    d = np.arctan2(l22, l21)

    bb = np.arctan2(-sinb, cosb)
    gg = np.arctan2(-sing, cosg)
    
    # Solution 1 (Elbow Up)
    t1 = np.arctan2(p[1], p[0])
    t2 = np.pi/2 - d - g - u
    t3 = np.pi/2 + d - b
    t4 = -np.radians(p[3]) - t2 - t3

    # Solution 2 (Elbow Down)
    tt1 = np.arctan2(p[1], p[0])
    tt2 = np.pi/2 - d - gg- u
    tt3 = np.pi/2 + d - bb
    tt4 = -np.radians(p[3]) - tt2 - tt3

    #if (-180 < np.rad2deg(t1) < 180) and (-115 < np.rad2deg(t2) < 90) and (-90 < np.rad2deg(t3) < 88) and (-100 < np.rad2deg(t4) < 115):
    return [[t1, t2, t3, t4],[tt1, tt2, tt3, tt4]]
    
    if (-180 < np.rad2deg(tt1) < 180) and (-115 < np.rad2deg(tt2) < 90) and (-90 < np.rad2deg(tt3) < 88) and (-100 < np.rad2deg(tt4) < 115):
        return [tt1, tt2, tt3, tt4]
    

    else:
        raise ValueError("Joint solutions not in range")

def get_dh_row_mat(arr):
        return np.array([[np.cos(arr[0]),  -np.sin(arr[0])*np.cos(arr[3]),    np.sin(arr[0])*np.sin(arr[3]),         arr[2]*np.cos(arr[0])],
                         [np.sin(arr[0]),   np.cos(arr[0])*np.cos(arr[3]),   -np.cos(arr[0])*np.sin(arr[3]),         arr[2]*np.sin(arr[0])],
                         [0,                np.sin(arr[3]),                   np.cos(arr[3]),                        arr[1]               ],
                         [0,                0,                                0,                                     1                    ]])

"""
get_int_mat(j):

Input: a 1 x 4 numpy array specifying the joint angles.
Output: a 4 x 4 x 4 numpy array of A matrices for specified joint angles (A1; A2; A3; A4).
Description: Utilizing get_dh_row_mat(), this method uses the DH table and joint
angles to calculate the intermediate transformation Ai for all rows at specific joint angles.
"""
def get_int_mat(j):
        return np.array([get_dh_row_mat([j[0],                               77,   0, -np.pi/2]),
                         get_dh_row_mat([j[1]-(np.pi/2 -np.arcsin(24/130)),   0, 130,        0]),
                         get_dh_row_mat([j[2]+(np.pi/2 -np.arcsin(24/130)),   0, 124,        0]),
                         get_dh_row_mat([j[3],                                0, 126,        0]),])

"""
get_acc_mat(j):

Input: a 1 x 4 numpy array specifying the joint angles.
Output: a 4 x 4 x 4 numpy array of T matrices (transforms from joint i to the base) for
specified joint angles (T01 ; T02 ; T 03 ; T 04 ).
Description: Using the A matrices from get_int_mat(), this method calculates the
accumulative transformations for all joints at specific joint angles.
"""

def get_acc_mat(j):
        a = get_int_mat(j)
        return np.array([a[0],
                        a[0] @ a[1],
                        a[0] @ a[1] @ a[2],
                        a[0] @ a[1] @ a[2] @ a[3]])
    
"""
get_fk(j):

Input: a 1 x 4 numpy array specifying the joint angles.
Output: a 4 x 4 numpy array representing the end-effector to base transformation (T 04)
Description: Using the A matrices from get_int_mat(), this method calculates the
forward kinematics of the robot as a 4 x 4 homogeneous transformation matrix repre-
senting the position and orientation of the end-effector frame with respect to the base
frame (i.e. Tbase-ee )
"""
    
def get_fk(j):
        a = get_int_mat(j)
        return a[0] @ a[1] @ a[2] @ a[3]
           
for waypoint in waypoints:
    print('Output of get_ik() for',waypoint)
    b = get_ik(waypoint)
    print(np.rad2deg(b))
    print()
    print('Output of get_fk()', waypoint)
    print(get_fk(b))
    print()