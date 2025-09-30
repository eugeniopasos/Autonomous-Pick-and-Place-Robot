import sys
import os

# Add the 'classes' directory to the PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

import matplotlib.pyplot as plt
from Robot import Robot
from TrajPlanner import TrajPlanner
from AprilTags import AprilTags
import time

import numpy as np
import cv2
import lab8_prelab as pre8
from Realsense import Realsense

##################
# Initialization #

# Initialize Objects
camera = Realsense()
robot = Robot()
at = AprilTags()

# Load Camera Calibration Transformation Coefficients
camera_robot_transform = np.load('camera_robot_transform.npy')
intrinsics = camera.get_intrinsics()

# Set Trajectory Parameters
traj_time = 2    # Defines the trajectroy time (s)
points_num = 498 # Defines the number if intermediate waypoints for a trajectory

# Set Bin Locations
Z = 150
alpha = 0
red_bin = [0, -220, Z, alpha]      # upper left
orange_bin = [120, -220, Z, alpha] # lower left
yellow_bin = [120, 220, Z, alpha]  # lower right
blue_bin = [0, 220, Z, alpha]      # upper right
color_bin_position = []
#####################


#####################
# Main Control Loop #

while True:
    # Set Robot to Home Configuration
    robot.write_time(traj_time)
    robot.write_joints(np.rad2deg(robot.get_ik([25, -100, 150, -60])))
    time.sleep(traj_time)
    robot.write_gripper(True)

    # Get Camera Frame and Find Centroid of Detected Balls
    color_frame, _ = camera.get_frames()
    final_image, radius_list, centroid_list, color_list = pre8.detect_colored_spheres(color_frame)
    print(centroid_list, color_list)
    cv2.imshow("Image Window", final_image) # Shows image with highlighted balls and centroid
    cv2.waitKey(1)
    
    
    if len(centroid_list) != 0:
        # Extract ball positions from image
        R, t = at.get_ball_pose(centroid_list[0], intrinsics, radius_list[0])
        t = np.append(t,1)
        transformed_centroid = camera_robot_transform @ t
        final = camera_robot_transform @ t

        # Morph Setpoint to accound for error
        transformed_centroid[0] += 10
        final[0] += 10
        transformed_centroid[2] = 100
        transformed_centroid[3] = -90
        final[2] = 30
        final[3] = -90

        # Decide on bin location
        if color_list[0] == 'Red':
            color_bin_position = red_bin
        elif color_list[0] == 'Orange':
            color_bin_position = orange_bin
        elif color_list[0] == 'Blue':
            color_bin_position = blue_bin
        elif color_list[0] == 'Yellow':
            color_bin_position = yellow_bin
        
        #################################
        # Plan and Execute Trajectories #

        tj = TrajPlanner(np.array([[100, 0, 100, -90], transformed_centroid, final, transformed_centroid, color_bin_position]))
        trajectories = tj.get_quintic_traj(traj_time, points_num) # Use quintic trajectory to control initial and final position,
                                                                  # velocity and acceleration
        time_step = trajectories[2,0]-trajectories[1,0]
        robot.write_time(traj_time) 
        robot.write_joints(np.rad2deg(robot.get_ik(trajectories[0, 1:]))) # Initialize Position
        time.sleep(traj_time)                                             # Wait for trajectory completion 
        robot.write_time(time_step)
        start_time = time.time()
        gripper = 1
        

        # Iterate Through Trajectories
        for i in range(1, len(trajectories)):

            robot.write_joints(np.rad2deg(robot.get_ik(trajectories[i, 1:])))
            while time.time() - start_time < (i * time_step):
                count = 1
            if (gripper == 1) and (i == 1000):
                robot.write_gripper(False)

        robot.write_gripper(True)
        gripper = 0
        #################################

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
#####################