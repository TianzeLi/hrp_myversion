#! /usr/bin/python
import cv2
import numpy as np
from matplotlib import pyplot as plt

if __name__ == '__main__':
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

	in_sample_hsv = cv2.cvtColor(img_in_sample, cv2.COLOR_BGR2HSV)
	out_sample_hsv = cv2.cvtColor(img_out_sample, cv2.COLOR_BGR2HSV)
	
	# Use red, blue, green for hue, satruation and value.
	HSV = ('Hue', 'Satruation', 'Value')
	color_space = ('r', 'b', 'g')

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
	cv2.waitKey(0)


