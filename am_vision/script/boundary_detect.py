#! /usr/bin/python

'''
 Visual lawn boundary detector.
 Input: the RGB image and optionally the depth image.
 Output: the labelled boundary or the suggested command for drving away from boundary
         depth image required for the latter output.
'''

import cv2
import numpy as np


# Load the image.
src = cv2.imread("../samples/p3_color.png")
# Check if the image is properly loaded.
if not src.data:
    print("Image not loaded correctly.")
# Show source image
cv2.imshow('Source Image', src)
cv2.waitKey(0)

# Display the edges detected by Canny operator.
edges = cv2.Canny(src,100,200)
cv2.imshow('Canny Edge on Source Image', edges)
cv2.waitKey(0)

# src_blur = cv2.blur(src,(6,6))
# # src_blur = cv2.GaussianBlur(src,(5,5),0)
# # src_blur = cv2.bilateralFilter(src,9,75,75)
# src = src_blur
# cv2.imshow('Source Image Blurred', src_blur)
# cv2.waitKey(0)

# Convert to the HSV space.
src_hsv = cv2.cvtColor(src, cv2.COLOR_RGB2HSV)

# Produce the mask.
green_lower = np.array([0, 5, 50])
green_upper = np.array([100, 120, 200])

mask_green = cv2.inRange(src_hsv, green_lower, green_upper) 
cv2.imshow("Green Mask Before Morphology", mask_green)
cv2.waitKey(0)


# Define kernels for morphology
# kernel1 = np.array([[1,1,1,1,1],
#                     [1,1,1,1,1],
#                     [1,1,1,1,1],
#                     [1,1,1,1,1],
#                     [1,1,1,1,1]], np.uint8)
kernel2 = np.ones((2, 2), np.uint8)
kernel3 = np.ones((3, 3), np.uint8)
kernel4 = np.ones((4, 4), np.uint8)
kernel5 = np.ones((5, 5), np.uint8)


# Morphology part, could tune the parameter "iterations"
mask_green = cv2.erode(mask_green, kernel4, iterations = 3)
mask_green = cv2.dilate(mask_green, kernel3, iterations = 4)
# mask_green = cv2.erode(mask_green, kernel4, iterations = 1)
# mask_green = cv2.erode(mask_green, kernel2, iterations = 2)
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel2)
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel2)

cv2.imshow("Green Mask After Morphology", mask_green)
cv2.waitKey(0)

img_color_threshold = cv2.bitwise_and(src, src, mask = mask_green)

cv2.imshow("Within color threshold", img_color_threshold)
cv2.waitKey(0)


# cv2.bitwise_not(src, img_color_filtered, mask_green);

# cv2.imshow("Filtered by color threshold", img_color_filtered);
# cv2.waitKey(0);

# # Visualize the final image
# cv2.imshow("Final Result", dst);
# cv2.waitKey(0);

