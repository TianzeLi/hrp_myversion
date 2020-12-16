#! /usr/bin/python

'''
 Visual segmentation based on texture..
 Input: RGB image or IR image.
 Output: labelled boundary.
 Pipeline (mainly):
 	1. Sample the frequency. (Not practical) 
 	1. Use Gabor filter.
 	2. Same as in the color_seg.

 Note: 
 	1. 
 TODO: 
 	1. Could use variance in pic after Gabor as weight or threshold.
 	2. 

'''

import cv2
import numpy as np

wavelength_min = 2.5

# Load the image.
src = cv2.imread("../samples/p91_color.png")
src = cv2.resize(src, (self.fixed_width, self.fixed_height))

# Check if the image is properly loaded.
if not src.data:
    print("Image not loaded correctly.")
# Show source image
cv2.imshow('Source Image', src)
cv2.waitKey(0)

src_gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
cv2.imshow('Intensity Image', src_gray)
cv2.waitKey(0)

# cv2.getGaborKernel(ksize, sigma, theta, lambda, gamma, psi, ktype)
# For example, 
# 	g_kernel = cv2.getGaborKernel(\
# 			   (21, 21), 8.0, np.pi/4, 10.0, 0.5, 0, ktype=cv2.CV_32F)
# ksize - size of gabor filter (n, n)
# sigma - standard deviation of the gaussian function
# theta - orientation of the normal to the parallel stripes
# 		  for lawn, should be 0.0 rad.
# lambda - wavelength of the sinusoidal factor
# gamma - spatial aspect ratio
# psi - phase offset
# ktype - type and range of values that each pixel in the gabor kernel can hold

g_kernel = cv2.getGaborKernel((15, 15), 7.0, 3.14159/12*0, 2.5, 1.0, 0, ktype=cv2.CV_32F)

filtered_img = cv2.filter2D(src_gray, cv2.CV_8UC3, g_kernel)

cv2.imshow('Filtered Image', filtered_img)
cv2.waitKey(0)

h, w = g_kernel.shape[:2]
g_kernel = cv2.resize(g_kernel, (3*w, 3*h), interpolation=cv2.INTER_CUBIC)
cv2.imshow('gabor kernel (resized)', g_kernel)
cv2.waitKey(0)
cv2.destroyAllWindows()