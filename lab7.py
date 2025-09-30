"""
Lab 6 Part 3: Validation of Camera-Robot Calibration
Tracks an AprilTag and transforms its position from camera to robot frame
"""

import sys
import os

# Add the 'classes' directory to the PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

from Robot import Robot
import time
import matplotlib.pyplot  as plt
import pickle
from TrajPlanner import TrajPlanner
import numpy as np
import cv2
import PIDController as PID1

from Realsense import Realsense
from AprilTags import AprilTags

# Robot Setup
traj_time = 4
tolerance = 10
robot = Robot()
robot.write_time(traj_time)
robot.write_motor_state(True)
robot.write_joints([0, 0, 0, 0])

time.sleep(traj_time)
robot.write_mode("velocity")

def main():
    try:
        # Initialize camera and detector
        camera = Realsense()
        detector = AprilTags()
        PID = PID1.PIDController()

        intrinsics = camera.get_intrinsics()
        
        # Load the saved calibration transform
        camera_robot_transform = np.load('camera_robot_transform.npy')
        print("Loaded camera-robot transformation matrix:")
        print(camera_robot_transform)
        
        # Constants
        TAG_SIZE = 40.0  # mm
        TAG_SIZE_METERS = TAG_SIZE / 1000.0
        PRINT_INTERVAL = 10  # frames
        prev_time = 0

        ee_pose_data = []
        t_data = []
        joint_data = []
        joint_vel_data = []
        ee_vel_data = []
        error_data = []
        PID_error_data = []

        while True:
            color_frame, _ = camera.get_frames()
            tags = detector.detect_tags(color_frame)
            detected = 0
            t_prime = [0, 0, 0, 0]
            if len(tags) != 0:
                detected = 1
            for tag in tags:
                detector.draw_tags(color_frame, tag)
                R_and_t = detector.get_tag_pose(tag.corners, intrinsics, TAG_SIZE)
                t = R_and_t[1]
                R = R_and_t[0]
                t = np.append(t,1)
                t_prime = camera_robot_transform @ t
            j = np.radians(robot.get_joints_readings())
            ee_pose = robot.get_ee_pos(j[0])
            t_prime[2] += 40
            print(t_prime)
            distance = t_prime[:3] - ee_pose[:3]
            if time.time() - prev_time > 0.02:
                PID_error = PID.compute_pid(distance)
                J = robot.get_jacobian(j[0])     
                Jinv = np.linalg.pinv(J[:3])
                j_velocities = Jinv @ PID_error
                if detected == 1:    
                    robot.write_velocities(j_velocities)    # Write joint values
                else:
                    robot.write_velocities([0, 0, 0, 0])
            ee_velocity = robot.get_forward_diff_kinematics(j[0],j[1] )
            PID_error_data.append(PID_error)
            joint_vel_data.append(j[1])
            joint_data.append(j[0])        
            t_data.append(time.time())
            ee_pose_data.append(robot.get_ee_pos(j[0]))
            ee_vel_data.append(ee_velocity)
            error_data.append(distance)
            
            
            cv2.imshow('Image', color_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        robot.write_velocities([0, 0, 0, 0])
        data_to_save = {
            'joint_angles': np.array(joint_data),
            'ee_positions': np.array(ee_pose_data),
            'timestamps': np.array(t_data),
            'joint_velocities':np.array(joint_vel_data),
            'ee_velocities': np.array(ee_vel_data),
            'error': np.array(error_data)}
        save_to_pickle(data_to_save, 'lab_7_stepresponse.pkl')
        print("Data Recorded")

# Save Data to pickle File
def save_to_pickle(data, filename):
    with open(filename, 'wb') as file:
        pickle.dump(data, file)

# Loading data from a pickle file
def load_from_pickle(filename):
    with open(filename, 'rb') as file:
        return pickle.load(file)


if __name__ == "__main__":
    main()