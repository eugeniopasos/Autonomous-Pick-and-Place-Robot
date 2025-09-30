import numpy as np

"""
get_dh_row_mat(arr):

Input: a 1 x 4 array corresponding to a row of the DH parameter table for a given joint.
Output: a 4 x 4 numpy array representing the homogeneous transformation matrix Ai .
Description: Calculates the intermediate transformation Ai for a DH parameter table row

"""

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


def get_current_fk():
    return get_fk(np.radians((get_joints_readings()[0])))

def get_ee_pos(j):
    a = get_current_fk()
    return np.array([a[0][3], a[1][3], a[2][3], np.rad2deg(-j[1] - j[2] - j[3]), np.rad2deg(j[0])])

tt1 = 1
print(-180 < tt1 < 180)
raise ValueError("Joint solutions not in range")
# print("Tbase-ee for [0, 0, 0, 0]")
# print(get_fk(np.radians([0, 0, 0, 0])))
# print()
# print("Tbase-ee for [15, -45, -60, 90]")

# print(get_fk(np.radians([15, -45, -60, 90])))
# print()
# print("Tbase-ee for [-90, 15, 30, -45]")
# print(get_fk(np.radians([-90, 15, 30, -45])))