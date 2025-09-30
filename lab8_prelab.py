import cv2
import os
import numpy as np

"""
Detect Colored Spheres and Return their Radii, Centroids, and Colors as well as
the centroids and circle outlines imprinted on the image
"""

def detect_colored_spheres(image):

    # Convert RGB image to HSV
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Filter image for specific colors
    red = cv2.inRange(image_hsv, (170, 100, 100), (179, 255, 255))
    orange = cv2.inRange(image_hsv, (5, 100, 50), (10, 255, 255))
    blue = cv2.inRange(image_hsv, (95, 70, 50), (130, 255, 255))
    yellow = cv2.inRange(image_hsv, (20, 90, 115), (30, 255, 255))
    kernel = np.ones((5,5), np.uint8)

    # Create mask for all colors
    mask = cv2.bitwise_or(cv2.bitwise_or(red, orange), cv2.bitwise_or(blue, yellow))
    mask_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    images = [red, orange, yellow, blue]
    colors = ["Red", "Orange", "Yellow", "Blue"]
    final_image = image.copy()
    color_list = []
    centroid_list = []
    radius_list = []

    # Find circles in each color filter
    for i,image in enumerate(images):

        # Create array of contours for each color
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:

            # Calculate area and perimeter to determine circularity
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = (4 * np.pi * area) / (perimeter ** 2) # Circularity equation

                # Filter by circularity index and ensure reasonable area
                if 0.5 <= circularity <= 1.2 and area > 200:

                    # Draw circle on image and label it
                    cv2.drawContours(final_image, [contour], -1, (0, 255, 0), 2)
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.putText(final_image, colors[i], (x, y - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    moments = cv2.moments(contour)

                    # Find and draw centroids of detected spheres
                    if moments["m00"] != 0:
                        cx = int(moments["m10"] / moments["m00"])
                        cy = int(moments["m01"] / moments["m00"])
                        centroid_list.append([cx, cy])
                        color_list.append(colors[i])
                        radius_list.append(np.sqrt(area/np.pi))

                        # Draw a red dot at the center
                        cv2.circle(final_image, (cx, cy), 2, (0, 0, 255), -1)

    return final_image, radius_list, centroid_list, color_list