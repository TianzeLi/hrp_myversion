#! /usr/bin/python

'''
 Non-learning visual lawn boundary detector.
 Input: RGB image and optionally the depth image.
 Output: labelled boundary or the suggested drving away direction.
 Pipeline (mainly): 
 	1. Color thresholding.
 	2. Morphology mask processing.
 	3. Filtering with requirements on contour.

 Note: 
 	1. Trade-off between robustness and efficiency.
 	2. The overall rigidity depends on all procedures in the pipeline.
 TODO: 
 	1. Automate color sampling.
 	2. 
 	3. Texture detection.
 	4. Integrate with ROS.

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

# # Display the edges detected by Canny operator.
# edges = cv2.Canny(src,100,200)
# cv2.imshow('Canny Edge on Source Image', edges)
# cv2.waitKey(0)

# Blur the image.
# src_blur = cv2.blur(src,(6,6))
# # src_blur = cv2.GaussianBlur(src,(5,5),0)
# # src_blur = cv2.bilateralFilter(src,9,75,75)
# src = src_blur
# cv2.imshow('Source Image Blurred', src_blur)
# cv2.waitKey(0)

# Convert to the HSV space.
src_hsv = cv2.cvtColor(src, cv2.COLOR_RGB2HSV)

# *********************************************************
# The color mask tunning can be automated by auto-sampling.
# Strategy: set the boundary to let in 95% of inliners. 
# Potential Automating block 1.
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


# ***************************************************
# The morphology tunning may not be easily automated.
# Strategy: find a set of universally optimal parameters.  
# Morphology part, could tune the parameter "iterations"
mask_green = cv2.erode(mask_green, kernel4, iterations = 3)
mask_green = cv2.dilate(mask_green, kernel3, iterations = 4)
# mask_green = cv2.erode(mask_green, kernel4, iterations = 1)
# mask_green = cv2.erode(mask_green, kernel2, iterations = 2)
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel3)
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel3)

cv2.imshow("Green Mask After Morphology", mask_green)
cv2.waitKey(0)

img_color_threshold = cv2.bitwise_and(src, src, mask = mask_green)

cv2.imshow("Within color threshold", img_color_threshold)
cv2.waitKey(0)

# Two threshold values, minVal and maxVal. 
# Any edges with intensity gradient more than maxVal are sure to be edges 
# and those below minVal are sure to be non-edges, so discarded. 
# Those who lie between these two thresholds are classified edges or 
# non-edges based on their connectivity. 
# If they are connected to sure-edge pixels, they are considered to be part of edges. 
# Otherwise, they are also discarded. 
edges = cv2.Canny(mask_green, 100, 200, L2gradient=True)
edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel4)
_, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# contours = cv2.findContours(mask_total.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

cv2.imshow('Canny Edge on Threshold Image', edges)
cv2.waitKey(0)

# Draw the contours before area check.
src_contour = cv2.drawContours(src.copy(), contours, -1, (0,255,0), 1)
cv2.imshow("Contours", src_contour)
cv2.waitKey(0)

# contour area restriction
# large_area = 2500
area_min = 900
length_min = 500

# Create the queue to contain the selected contours.
lawn_contour = []

if len(contours) > 0:
	i = -1
	for cnt in contours:
		i += 1
		area = cv2.contourArea(cnt)
		length = cv2.arcLength(cnt,True)

		# if (area < area_min):
		# 	continue

		if length < length_min:
			continue

		print("Got a contour!")
		lawn_contour.append(cnt)
		print('The area is %.3f' % area )
		print('The length is %.3f' % length)
		# Print the first child contour and the parent contour
		# print(hierarchy[0][i][2])
		if hierarchy[0][i][3] > 0:
			print('Has the parent contour no. %d' % hierarchy[0][i][3])
		print('\n')
else:
	print('No contour fouond before examining the shapes.')

# Draw all the passed contours
src_contour_dst = cv2.drawContours(src, lawn_contour, -1, (0,255,255), 5)
# Draw the contours after the area check.
cv2.imshow("Lawn Contours", src_contour_dst)
cv2.waitKey(0)



# cv2.bitwise_not(src, img_color_filtered, mask_green);

# cv2.imshow("Filtered by color threshold", img_color_filtered);
# cv2.waitKey(0);

# # Visualize the final image
# cv2.imshow("Final Result", dst);
# cv2.waitKey(0);

