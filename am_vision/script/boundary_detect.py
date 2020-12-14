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
 	0. Load each feature detector into a function and form main pipeline. 
 	1. Automate color sampling.
 		(Still need draw box on img and tune parameters.)
 	2. 
 	3. Add texture features.
 	4. Add spatial adjacency as features.
 	5. Potentially add depth into feature.
 	6. Clustering.
 	7. Integrate with ROS.
 	8. Potentially feed back the warning level and retreating suggestion. 

'''

import cv2
import numpy as np


class BoundaryDetectNode():
	""" Detect the boundary of the lawn in the image. 

	"""

	def __init__(self):
		# ROS parameters.


		# Standalone parameters.
		self.fixed_width = 640
		self.fixed_height = 480

		self.left_sample_w = 150
		self.left_sample_x0 = self.fixed_width/4
		self.left_sample_y0 = self.fixed_height - self.fixed_height/4
		self.left_sample_h = self.fixed_height/4


		self.right_sample_w = 150
		self.right_sample_x0 = self.fixed_width*3/4 - self.right_sample_w
		self.right_sample_y0 = self.fixed_height - self.fixed_height/4
		self.right_sample_h = self.fixed_height/4

		self.select_precentage = 0.90

		# Pipeline starts here.
		self.src = self.image_load()
		self.color_lower_bound, self.color_upper_bound = \
				 self.color_sample(self.src)
		self.green_mask, self.img_threshold = self.color_mask(self.src)
		self.color_contour(self.green_mask, self.src)



	def image_load(self):
		""" Load the Image and reshape to the uniform size. 

		"""

		# Load the image.
		src = cv2.imread("../samples/p3_color.png")
		# Check if the image is properly loaded.
		if not src.data:
		    print("Image not loaded correctly.")
		
		src = cv2.resize(src, (self.fixed_width, self.fixed_height))
		# Show formated size source image
		cv2.imshow('Source Image', src)
		cv2.waitKey(0)
		return src

		# # Display the edges detected by Canny operator.
		# edges = cv2.Canny(src,100,200)
		# cv2.imshow('Canny Edge on Source Image', edges)
		# cv2.waitKey(0)


	def image_blur(self, src):
		""" Blur the image

		"""
		src_blur = cv2.blur(src,(6,6))
		# src_blur = cv2.GaussianBlur(src,(5,5),0)
		# src_blur = cv2.bilateralFilter(src,9,75,75)
		src = src_blur
		cv2.imshow('Source Image Blurred', src_blur)
		cv2.waitKey(0)


	def color_sample(self, src):
		""" Sample the color histgram from the area that supposed to be 
			part of the lawn. 
		
		"""

		img_left_sample = src[int(self.left_sample_y0):int(self.left_sample_y0 \
						+ self.left_sample_h), int(self.left_sample_x0): \
						int(self.left_sample_x0 + self.left_sample_w)]

		img_right_sample = src[int(self.right_sample_y0):int(self.right_sample_y0 \
						+ self.right_sample_h), int(self.right_sample_x0): \
						int(self.right_sample_x0 + self.right_sample_w)]

		cv2.imshow('Left sampled image.', img_left_sample)
		cv2.imshow('Right sampled image.', img_right_sample)
		cv2.waitKey(0)

		left_sample_hsv = cv2.cvtColor(img_left_sample, cv2.COLOR_BGR2HSV)
		right_sample_hsv = cv2.cvtColor(img_right_sample, cv2.COLOR_BGR2HSV)
		
		# Use red, blue, green for hue, satruation and value.
		HSV = ('Hue', 'Satruation', 'Value')
		color_space = ('r', 'b', 'g')

		#  Create array to store the bounds.
		color_lower_bound = np.array([0, 0, 0])
		color_upper_bound = np.array([255, 255, 255])
		threshold_count = (1 - self.select_precentage) * self.left_sample_w \
							* self.left_sample_h

		# plt.figure(fig_num)
		# fig_num += 1
		for i, col in enumerate(color_space):
			tmp_count = 0
			j = 0
			left_hist_hsv = cv2.calcHist([left_sample_hsv], [i], None, [256], [0, 256])
			
			while (tmp_count < 0.5*threshold_count):
				tmp_count += left_hist_hsv[j]
				j += 1
			color_lower_bound[i] = j

			j = 0
			while (tmp_count < 0.5*threshold_count):
				tmp_count += left_hist_hsv[255-j]
				j += 1			

			color_upper_bound[i] = 255 - j 

		# 	plt.plot(hist_hsv, color = col, label = HSV[i])
		# 	plt.xlim([0,256])
		# plt.legend()
		# plt.xlabel('Value')
		# plt.ylabel('Count')
		# plt.title('Sampled lawn pixels distribution in HSV space')
		# plt.grid(True)

		for i, col in enumerate(color_space):
			tmp_count = 0
			j = 0
			right_hist_hsv = cv2.calcHist([right_sample_hsv], [i], None, [256], [0, 256])
			
			while (tmp_count < 0.5*threshold_count):
				tmp_count += left_hist_hsv[j]
				j += 1
			if (j < color_lower_bound[i]):
				color_lower_bound[i] = j

			j = 0
			while (tmp_count < 0.5*threshold_count):
				tmp_count += right_hist_hsv[255-j]
				j += 1			
			if ((255 - j) > color_lower_bound[i]):
				color_upper_bound[i] = 255 - j 

		return color_lower_bound, color_upper_bound

	def color_mask(self, src):
		""" Make the green mask.

		"""

		# Convert to the HSV space.
		src_hsv = cv2.cvtColor(src, cv2.COLOR_RGB2HSV)

		# *********************************************************
		# The color mask tunning can be automated by auto-sampling.
		# Strategy: set the boundary to let in 95% of inliners. 
		# Potential Automating block 1.
		# Produce the mask.

		# green_lower = np.array([0, 5, 50])
		# green_upper = np.array([100, 120, 200])
		green_lower = self.color_lower_bound
		green_upper = self.color_upper_bound

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

		return mask_green, img_color_threshold


	def color_contour(self, mask_green, src):
		""" Draw contours out of the mask and source image.
		
		"""

		# Two threshold values, minVal and maxVal. 
		# Any edges with intensity gradient more than maxVal are sure to be edges 
		# and those below minVal are sure to be non-edges, so discarded. 
		# Those who lie between these two thresholds are classified edges or 
		# non-edges based on their connectivity. 
		# If they are connected to sure-edge pixels, they are considered to be part of edges. 
		# Otherwise, they are also discarded. 
		
		# kernel1 = np.array([[1,1,1,1,1],
		#                     [1,1,1,1,1],
		#                     [1,1,1,1,1],
		#                     [1,1,1,1,1],
		#                     [1,1,1,1,1]], np.uint8)
		kernel2 = np.ones((2, 2), np.uint8)
		kernel3 = np.ones((3, 3), np.uint8)
		kernel4 = np.ones((4, 4), np.uint8)
		kernel5 = np.ones((5, 5), np.uint8)

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



if __name__ == '__main__':
    # rospy.init_node('boundary_detect')
    BoundaryDetectNode()
    # rospy.spin()
