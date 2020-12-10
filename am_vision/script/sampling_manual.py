#! /usr/bin/python

'''
 Manual select region interedted and outliers to analyze 
 distribution in HSV space.

 Contains three part:
 	Select foreground sample and background sample.
 	Color image analysis in HSV space.
 	Gray-scale image texture analysis in frequency space.

'''

import cv2
import numpy as np
from matplotlib import pyplot as plt

fig_num = 0

if __name__ == '__main__':
	# ************************************
	# Part one: Interested area selecting.
	# Read image.
	img = cv2.imread("../samples/p3_color.png")
	# cv2.imshow("Original Image", img)
	# cv2.waitKey(0)

	# Select region of interest to sample.
	print("Select the inliners.")
	roi_in = cv2.selectROI(img)

	# Crop image.
	img_in_sample = img[int(roi_in[1]):int(roi_in[1]+roi_in[3]), \
					int(roi_in[0]):int(roi_in[0]+roi_in[2])]

	# Display cropped image
	# cv2.imshow("Region of Interest", img_sample)
	# cv2.waitKey(0)

	# Select region not of interest to sample.
	print("Select the outliners.")
	roi_out = cv2.selectROI(img)

	# Crop image.
	img_out_sample = img[int(roi_out[1]):int(roi_out[1]+roi_out[3]), \
					int(roi_out[0]):int(roi_out[0]+roi_out[2])]

	# Display cropped image
	# cv2.imshow("Region of Interest", img_sample)
	# cv2.waitKey(0)

	
	# *************************
	# Part two: color analysis.
	in_sample_hsv = cv2.cvtColor(img_in_sample, cv2.COLOR_BGR2HSV)
	out_sample_hsv = cv2.cvtColor(img_out_sample, cv2.COLOR_BGR2HSV)
	
	# Use red, blue, green for hue, satruation and value.
	HSV = ('Hue', 'Satruation', 'Value')
	color_space = ('r', 'b', 'g')

	plt.figure(fig_num)
	fig_num += 1
	plt.subplot(2, 1, 2)
	for i, col in enumerate(color_space):
		hist_hsv = cv2.calcHist([in_sample_hsv], [i], None, [256], [0, 256])
		plt.plot(hist_hsv, color = col, label = HSV[i])
		plt.xlim([0,256])
	plt.legend()
	plt.xlabel('Value')
	plt.ylabel('Count')
	plt.title('Sampled lawn pixels distribution in HSV space')
	plt.grid(True)

	plt.subplot(2, 1, 1)
	for i, col in enumerate(color_space):
		hist_hsv = cv2.calcHist([out_sample_hsv], [i], None, [256], [0, 256])
		plt.plot(hist_hsv, color = col, label = HSV[i])
		plt.xlim([0,256])
	plt.legend()
	plt.xlabel('Value')
	plt.ylabel('Count')
	plt.title('Sampled outliner pixels distribution in HSV space')
	plt.grid(True)

	plt.tight_layout()
	plt.show()

	
	# *****************************
	# Part three: frequency analysis.
	in_sample_gray = cv2.cvtColor(img_in_sample, cv2.COLOR_BGR2GRAY)
	out_sample_gray = cv2.cvtColor(img_out_sample, cv2.COLOR_BGR2GRAY)

	f_in = np.fft.fft2(in_sample_gray)
	fshift_in = np.fft.fftshift(f_in)
	magnitude_spectrum_in = 20*np.log(np.abs(fshift_in))

	f_out = np.fft.fft2(out_sample_gray)
	fshift_out = np.fft.fftshift(f_out)
	magnitude_spectrum_out = 20*np.log(np.abs(fshift_out))

	hist_in_gray = cv2.calcHist([in_sample_gray], [0], None, [256], [0, 256])
	hist_out_gray = cv2.calcHist([out_sample_gray], [0], None, [256], [0, 256])

	plt.figure(fig_num)
	fig_num += 1
	plt.plot(hist_in_gray, color = 'g', label = 'In-hist')
	plt.xlim([0,256])
	plt.plot(hist_out_gray, color = 'r', label = 'Out-hist')
	plt.xlim([0,256])
	plt.legend()
	plt.xlabel('Value')
	plt.ylabel('Count')
	plt.title('Frequency distribution in intensity space')
	plt.grid(True)
	# plt.show()

	plt.figure(fig_num)
	fig_num += 1
	plt.subplot(221),plt.imshow(magnitude_spectrum_in, cmap = 'gray')
	plt.title('Inliner magnitude spectrum'), plt.xticks([]), plt.yticks([])
	plt.subplot(222),plt.imshow(magnitude_spectrum_out, cmap = 'gray')
	plt.title('Outliner magnitude spectrum'), plt.xticks([]), plt.yticks([])
	plt.show()

	gray_lower = np.array([200])
	gray_upper = np.array([500])
	mask_gray_in = cv2.inRange(magnitude_spectrum_in, gray_lower, gray_upper)
	mask_gray_out = cv2.inRange(magnitude_spectrum_out, gray_lower, gray_upper)
	
	in_gray_threshold = cv2.bitwise_and(magnitude_spectrum_in, \
							magnitude_spectrum_in, mask = mask_gray_in)

	out_gray_threshold = cv2.bitwise_and(magnitude_spectrum_out, \
							magnitude_spectrum_out, mask = mask_gray_out)

	plt.subplot(223),plt.imshow(in_gray_threshold, cmap = 'gray')
	plt.title('Inliner magnitude spectrum within threshold'), plt.xticks([]), plt.yticks([])
	plt.subplot(224),plt.imshow(out_gray_threshold, cmap = 'gray')
	plt.title('Outliner magnitude spectrum within threshold'), plt.xticks([]), plt.yticks([])
	plt.show()

	cv2.waitKey(0)
