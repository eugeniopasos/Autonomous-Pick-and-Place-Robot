"""
Lab 6 Part 2: Camera-Robot Calibration
Implements camera-robot calibration using AprilTags and the Kabsch algorithm
"""

import sys
import os

# Add the 'classes' directory to the PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

import numpy as np
import cv2
import lab6_prelab as pre
import time

from Realsense import Realsense
from AprilTags import AprilTags

# def point_registration(points_camera, points_robot):
#     """
#     Implement Kabsch algorithm to find transformation between camera and robot frames.
    
#     Args:
#         points_camera: 4xN array of homogeneous points in camera frame
#         points_robot: 4xN array of homogeneous points in robot frame
        
#     Returns:
#         4x4 homogeneous transformation matrix from camera to robot frame
#     """
#     # Algorithm implemented for students
#     pass

def main():
    try:
        # Initialize cameras and detectors
        camera = Realsense()
        detector = AprilTags()
        intrinsics = camera.get_intrinsics()
               
        # AprilTag size in mm (adjust according to your tag)
        TAG_SIZE = 40.0
        TAG_SIZE_METERS = TAG_SIZE / 1000.0
        
        # Define known tag positions in robot frame (mm)
        robot_points = [
            [64, -90, 0],   # Add all your measured points here
            [64, -30, 0],
            [64, 30, 0],
            [64, 90, 0],
            [124, -90, 0],   # Add all your measured points here
            [124, -30, 0],
            [124, 30, 0],
            [124, 90, 0],
            [184, -90, 0],   # Add all your measured points here
            [184, -30, 0],
            [184, 30, 0],
            [184, 90, 0],
        ]
        
        # 1. Convert robot points to homogeneous coordinates (4xN)
        # YOUR CODE HERE
        for point in robot_points:
            point.append(1)
        
        robot_points = np.transpose(np.array(robot_points))

        # 2. Initialize camera points array with same size
        # YOUR CODE HERE
        points_camera = np.empty((12,3))

        # 3. Collect 3 to 5 measurements:
        #    - Get frames and detect tags
        #    - When all tags visible:
        #      * Get pose for each tag
        #      * Store positions in order
        #    - Average the measurements
        # YOUR CODE HERE
        t_list = []
        R_list = []
        dis_list = []
        or_list = []
        translation_set = np.zeros((3,12))
        for i in range(1):
            color_frame, _ = camera.get_frames()
            tags = detector.detect_tags(color_frame)
            if(len(tags) == 12):
                for tag in tags:
                    detector.draw_tags(color_frame, tag)
                    R_and_t = detector.get_tag_pose(tag.corners, intrinsics, TAG_SIZE)
                    t = R_and_t[1]
                    R = R_and_t[0]
                    t_list.append(t)
                    R_list.append(R)
                    distance = np.linalg.norm(t)
                    orientation = cv2.RQDecomp3x3(R)[0]
                    dis_list.append(distance)
                    or_list.append(orientation)
                    translation_set[:, tag.tag_id] = t[:,0] 
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            cv2.imshow('Image', color_frame)


        # 4. Calculate transformation using point_registration()
        # YOUR CODE HERE

        T = pre.point_registration(translation_set, robot_points[:3,:])
        print("Transformation Matrix: ")
        print(T)
        # 5. Calculate and print calibration error
        # YOUR CODE HERE
        
        # 6. Save transformation matrix
        # YOUR CODE HERE
        np.save('camera_robot_transform.npy',T)

    finally:
        camera.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()