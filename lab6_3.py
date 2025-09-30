"""
Lab 6 Part 3: Validation of Camera-Robot Calibration
Tracks an AprilTag and transforms its position from camera to robot frame
"""

import sys
import os

# Add the 'classes' directory to the PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

import numpy as np
import cv2

from Realsense import Realsense
from AprilTags import AprilTags

def main():
    try:
        # Initialize camera and detector
        camera = Realsense()
        detector = AprilTags()
        intrinsics = camera.get_intrinsics()
        
        # Load the saved calibration transform
        camera_robot_transform = np.load('camera_robot_transform.npy')
        print("Loaded camera-robot transformation matrix:")
        print(camera_robot_transform)
        
        # Constants
        TAG_SIZE = 40.0  # mm
        TAG_SIZE_METERS = TAG_SIZE / 1000.0
        PRINT_INTERVAL = 10  # frames
        counter = 1
        
        while True:
            # 1. Get camera frame
            # YOUR CODE HERE
            color_frame, _ = camera.get_frames()
            # 2. Detect AprilTags
            # YOUR CODE HERE
            tags = detector.detect_tags(color_frame)
            # 3. For each detected tag:
            #    - Get pose using get_tag_pose()
            #    - Create tag-to-camera transform
            #    - Convert to robot frame using camera_robot_transform
            #    - Print every PRINT_INTERVAL frames:
            #      * Camera frame coordinates
            #      * Robot frame coordinates
            # YOUR CODE HERE
            detected = 0
            if len(tags) != 0:
                detected = 1

            if (counter%1 == 0):
                for tag in tags:
                    detector.draw_tags(color_frame, tag)
                    R_and_t = detector.get_tag_pose(tag.corners, intrinsics, TAG_SIZE)
                    t = R_and_t[1]
                    R = R_and_t[0]
                    t = np.append(t,1)
                    t_prime = camera_robot_transform @ t
                    print('Tag position camera frame', t)
                    print('Tag position robot frame', t_prime)
                    print('Tracking Status', detected)
            # 4. Display frame with detections
            # YOUR CODE HERE
            cv2.imshow('Image', color_frame)
            # Exit when 'q' is pressed
            counter +=1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        camera.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()