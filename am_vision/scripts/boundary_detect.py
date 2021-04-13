#! /usr/bin/python

"""
 Lawn boundary detector based on explicit image processing methods.
 Input: 
 	RGB image and optionally the depth image.
 Output: 
 	Labelled boundary and optionally the suggested drving away direction.
 
 Pipeline: 
 	1. Color thresholding
 		1.1 Morphology mask processing.
 		1.2 Filtering with requirements on contour.
 	2. Texture feature detecting.
 		2.1 Filter with multi gabor filters.
 		2.2 Only accept feature image with proper variance.
 	3. Depth feature
 		3.1 Directly make use of the depth image. 
 	4. Clustering using Kmeans.

 Note: 
 	1. Trade-off between robustness and efficiency.
 	2. The overall rigidity depends on all procedures in the pipeline.
 TODO: 
 	2. Consider if to set range limit for depth images.
 	6. Integrate with ROS.
 	7. Optionally feed back the warning level and retreating suggestion. 
 	8. Add superpixel generating. (See DigitalImageProcessing P775)
 	9. Maybe try GraphCut clustering algorithm.
 	10. Use histogram equalization or other image enhancement
 	    methods when the lighting condition is extreme.
 	11. Add connectivity to the sample as a criention.
 	12. Pre-process into super-pixels.

"""

import cv2
import numpy as np
import logging
import matplotlib.pyplot as plt
from matplotlib.colors import NoNorm

# Create logger with 'log'
logger = logging.getLogger('BoundaryDetect')
logger.setLevel(logging.DEBUG)
# Examples
# logger.debug('This is a debug message')
# logger.info('This is an info message')
# logger.warning('This is a warning message')
# logger.error('This is an error message')
# logger.critical('This is a critical message')

# Enable colored text in terminal.
def prRed(prt): print("\033[91m {}\033[00m" .format(prt))
def prGreen(prt): print("\033[92m {}\033[00m" .format(prt))
def prYellow(prt): print("\033[93m {}\033[00m" .format(prt))
def logYellow(prt): logger.debug("\033[93m {}\033[00m" .format(prt))
def prLightPurple(prt): print("\033[94m {}\033[00m" .format(prt))
def prPurple(prt): print("\033[95m {}\033[00m" .format(prt))
def prCyan(prt): print("\033[96m {}\033[00m" .format(prt))
def logCyan(prt): logger.debug("\033[96m {}\033[00m" .format(prt))
def prLightGray(prt): print("\033[97m {}\033[00m" .format(prt))
def prBlack(prt): print("\033[98m {}\033[00m" .format(prt))


class BoundaryDetectNode():
	""" Detect the boundary of the lawn in the image. 

	"""

	def __init__(self):
		# ROS parameters.(TODO)


		# Standalone init parameters

		# Specify the location of the images to load.
		src_path = "../samples/p2_color.png"
		src_depth_path = "../samples/p2_depth.png"

		# Select the features for clustering.
		self.use_color_feature = True
		self.use_loc_feature = True
		self.use_texture_feature = True
		self.use_depth_feature = False

		# Kmeans parameters
		self.K = 2
				
		# Define the precentage of accepting points for color seg.
		self.select_precentage = 0.75

		# Unify the size of the image and the depth image.
		self.fixed_width = 640
		self.fixed_height = 480

		# Define the sampling area on the left.
		self.left_sample_w = 150
		self.left_sample_x0 = self.fixed_width/4
		self.left_sample_y0 = self.fixed_height - self.fixed_height/6
		self.left_sample_h = self.fixed_height/6

		# Define the sampling area on the right.
		self.right_sample_w = 150
		self.right_sample_x0 = self.fixed_width*3/4 - self.right_sample_w
		self.right_sample_y0 = self.fixed_height - self.fixed_height/6
		self.right_sample_h = self.fixed_height/6

		# Define the range for normalizing, i.e. the weights in kmeans.
		self.feature_range_color = 10
		self.feature_range_loc = 17
		self.feature_range_texture = 9
		self.feature_range_depth = 20

		# Color parameter
		self.blur_before_sampling = True

		# Markers
		self.feature_mat_empty = True
		self.num_feature = 0

		# Visualizing options
		self.show_gabor_filters = False
		# To count the plt numbers.
		self.fig_num = 1


		# Pipeline starts here.
		self.src = self.image_load(src_path)
		self.src_gray = cv2.cvtColor(self.src, cv2.COLOR_BGR2GRAY)
		
		if (self.use_color_feature):

			if(self.blur_before_sampling):
				src_color = cv2.GaussianBlur(self.src.copy(),(5,5),0)
			else:
				src_color = self.src.copy()

			img_sample = cv2.rectangle(src_color, (self.left_sample_x0, self.left_sample_y0), \
										(self.left_sample_x0 + self.left_sample_w,\
										self.left_sample_y0 + self.left_sample_h), (0, 255, 255), 3)
			img_sample = cv2.rectangle(img_sample, (self.right_sample_x0, self.right_sample_y0), \
										(self.right_sample_x0 + self.right_sample_w,\
										self.right_sample_y0 + self.right_sample_h), (0, 255, 255), 3)
			cv2.imshow('Source image with color sampling', img_sample)
			cv2.waitKey(0)

			color_bound_low, color_bound_up = self.color_sample(self.src, self.select_precentage)
			self.green_mask, self.img_threshold = self.color_mask(src_color, color_bound_low, color_bound_up)
			# self.find_contour(self.green_mask, self.src.copy())

			num_color_feature = 1
			self.num_feature += num_color_feature
			if (self.feature_mat_empty):
				self.feature_mat = self.green_mask.reshape((-1, num_color_feature))
				self.feature_mat = self.feature_normalize(self.feature_mat, self.feature_range_color)
				self.feature_mat_empty = False
			else:
				self.green_mask = self.feature_normalize(self.green_mask, self.feature_range_color)
				self.feature_append(self.feature_mat, self.green_mask, num_color_feature)
				logging.debug('The color feature matrix has the shape {}'.format(self.feature_mat.shape))
			
			if (self.use_loc_feature):
				loc_img_x = np.zeros(self.src_gray.shape)
				loc_img_y = np.zeros(self.src_gray.shape)
				
				for i in range(self.fixed_width):
					loc_img_x[:, i-1] = i - 1

				for i in range(self.fixed_height):
					loc_img_y[i-1, :] = i - 1

				loc_img_x = self.feature_normalize(loc_img_x, self.feature_range_loc)
				loc_img_y = self.feature_normalize(loc_img_y, self.feature_range_loc)
				self.feature_append(self.feature_mat, loc_img_x, 1)
				self.feature_append(self.feature_mat, loc_img_y, 1)
				self.num_feature += 2

		
		if (self.use_texture_feature):
			if (self.feature_mat_empty):
				self.num_texture_feature = self.texture_seg(self.src_gray)
				if (self.num_texture_feature > 0):
					self.feature_mat_empty = False
					self.num_feature += self.num_texture_feature
				else:
					logging.warning("No capable features in the texture segmentation.")

			else:
				self.num_texture_feature = self.texture_seg(self.src_gray)
				self.num_feature += self.num_texture_feature

			# self.feature_mat = self.feature_append(self.texture_feature, self.num_texture_feature)

		if (self.use_depth_feature):
			self.depth_img = self.image_load(src_depth_path)
			self.depth_img = cv2.cvtColor(self.depth_img, cv2.COLOR_BGR2GRAY)
			num_depth_feature = 1
			self.num_feature += num_depth_feature

			if (self.feature_mat_empty):
				self.feature_mat = self.depth_img.reshape((-1, num_depth_feature))
				self.feature_mat = self.feature_normalize(self.feature_mat, self.feature_range_depth)
				self.feature_mat_empty = False			
			else:
				self.depth_img = self.feature_normalize(self.depth_img, self.feature_range_depth)
				self.feature_append(self.feature_mat, self.depth_img, num_depth_feature)
				logging.debug(self.feature_mat.shape)


		self.feature_cluster = self.kmeans_seg(self.feature_mat, self.K)
		self.feature_cluster = cv2.normalize(self.feature_cluster, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
		self.feature_cluster = self.feature_cluster.astype(np.uint8)
		self.find_contour(self.feature_cluster, self.src.copy())	


	def image_load(self, loc):
		""" Load the Image and reshape to the uniform size. 
			
			Input: 
				The image locations. 
			Output:
				The image shaped into the unified size.

		"""

		src = cv2.imread(loc)
		if not src.data:
		    logging.error("Image not loaded correctly.")
		
		src = cv2.resize(src, (self.fixed_width, self.fixed_height))

		img_name = 'Source Image: ' + loc
		cv2.imshow(img_name, src)
		cv2.waitKey(0)

		return src


	def image_blur(self, src):
		""" Blur the image.

			Input: 
				The source image.
			Output: 
				The blurred image.
		"""

		src_blur = cv2.blur(src,(6,6))
		# src_blur = cv2.GaussianBlur(src,(5,5),0)
		# src_blur = cv2.bilateralFilter(src,9,75,75)
		src = src_blur
		# cv2.imshow('Source Image Blurred', src_blur)
		# cv2.waitKey(0)
		return src


	def left_crop(self, img):
		""" Extract the image in the selected region on the left side.

		"""

		return img[int(self.left_sample_y0):int(self.left_sample_y0 \
						+ self.left_sample_h), int(self.left_sample_x0): \
						int(self.left_sample_x0 + self.left_sample_w)]

	def right_crop(self, img):
		""" Extract the image in the selected region on the left side.
		
		"""

		return img[int(self.right_sample_y0):int(self.right_sample_y0 \
						+ self.right_sample_h), int(self.right_sample_x0): \
						int(self.right_sample_x0 + self.right_sample_w)]


	def color_sample(self, src, accept_ratio):
		""" Find the distribution in the color space by checking the 
			histgram from the area that supposed to be part of the lawn. 

			Arguments:
				src - The sampled image.
				accept_ratio - the ratio to accept.
			Return:
				The bound value in the HSV space.
		"""

		img_left_sample = self.left_crop(src)
		img_right_sample = self.right_crop(src)
		left_sample_hsv = cv2.cvtColor(img_left_sample, cv2.COLOR_BGR2HSV)
		right_sample_hsv = cv2.cvtColor(img_right_sample, cv2.COLOR_BGR2HSV)

		# Use red, blue, green for hue, satruation and value.
		HSV = ('Hue', 'Satruation', 'Value')
		color_space = ('r', 'b', 'g')

		# Array to store the bounds.
		# H - [0, 180]; S - [0, 255]; V - [0, 255]
		color_lower_bound = np.array([0, 0, 0])
		color_upper_bound = np.array([180, 255, 255])
		threshold_count = (1 - accept_ratio)*self.left_sample_w*self.left_sample_h

		# Find the bound that (1 - accept_ratio) of the points are within.
		# Keep out 2.5% on the low side and 2.5% on the high side if 5% is kept out. 
		for i, col in enumerate(color_space):
			tmp_count = 0
			j = 0
			left_hist_hsv = cv2.calcHist([left_sample_hsv], [i], None, [256], [0, 256])
			right_hist_hsv = cv2.calcHist([right_sample_hsv], [i], None, [256], [0, 256])
			
			while (tmp_count < 0.5*threshold_count):
				tmp_count += left_hist_hsv[j]
				j += 1
			color_lower_bound[i] = j

			while (tmp_count < 0.5*threshold_count):
				tmp_count += right_hist_hsv[j]
				j += 1
			if (j < color_lower_bound[i]):
				color_lower_bound[i] = j

			j = 0
			while (tmp_count < 0.5*threshold_count):
				tmp_count += left_hist_hsv[255-j]
				j += 1			
			color_upper_bound[i] = 255 - j

			j = 0
			while (tmp_count < 0.5*threshold_count):
				tmp_count += right_hist_hsv[255-j]
				j += 1			
			if ((255 - j) > color_lower_bound[i]):
				color_upper_bound[i] = 255 - j  
		
		# # Plot the histgram.
		# plt.figure(fig_num)
		# fig_num += 1
		# 	plt.plot(hist_hsv, color = col, label = HSV[i])
		# 	plt.xlim([0,256])
		# plt.legend()
		# plt.xlabel('Value')
		# plt.ylabel('Count')
		# plt.title('Sampled lawn pixels distribution in HSV space')
		# plt.grid(True)

		return color_lower_bound, color_upper_bound


	def color_mask(self, src, green_lower, green_upper):
		""" Filter the image with green mask and then morphology operations.
			
			Argument: 
				src - source image
				green_lower - the lower bound for lawn color 
				green_upper - the upper bound for lawn color
			Return:
				The labelled image for accepted regions.
		"""

		# Convert to the HSV space.
		src_hsv = cv2.cvtColor(src, cv2.COLOR_RGB2HSV)

		# # Manually set bounds.
		# green_lower = np.array([0, 5, 50])
		# green_upper = np.array([100, 120, 200])

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


		# The morphology tunning may not be easily automated.
		# Strategy: find a set of universally optimal parameters.  
		# Could tune the parameter "iterations"
		mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel5)
		# mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel3)

		mask_green = cv2.erode(mask_green, kernel3, iterations = 3)
		mask_green = cv2.dilate(mask_green, kernel3, iterations = 2)
		mask_green = cv2.erode(mask_green, kernel3, iterations = 2)
		mask_green = cv2.dilate(mask_green, kernel3, iterations = 2)

		# mask_green = cv2.erode(mask_green, kernel4, iterations = 1)
		# mask_green = cv2.erode(mask_green, kernel2, iterations = 2)
		# mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel4)
		# mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel4)

		cv2.imshow("Green Mask After Morphology", mask_green)
		cv2.waitKey(0)

		img_color_threshold = cv2.bitwise_and(src, src, mask = mask_green)

		cv2.imshow("Within color threshold", img_color_threshold)
		cv2.waitKey(0)

		return mask_green, img_color_threshold


	def find_contour(self, mask, src):
		""" Draw contours defined by the mask on the source image.
			Also remove the not desirable areas by examining their contours.

			Input: 
				1. The color mask obtained from the former step.
				2. The image to apply the mask.
		
		"""

		# For the edge detecotr cv2.Canny, there are: 
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


		# The morphology tunning may not be easily automated.
		# Strategy: find a set of universally optimal parameters.  
		# Could tune the parameter "iterations"
		mask = cv2.dilate(mask, kernel4, iterations = 3)
		mask = cv2.erode(mask, kernel4, iterations = 4)
		# mask = cv2.erode(mask, kernel4, iterations = 1)
		# mask = cv2.erode(mask, kernel2, iterations = 2)
		mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5)
		# mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel4)


		edges = cv2.Canny(mask, 100, 200, L2gradient=True)
		edges = cv2.dilate(edges, kernel3, iterations = 1)
		edges = cv2.erode(edges, kernel2, iterations = 2)

		edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel5)

		# edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel5)
		# edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel5)


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

				logger.debug("Got a contour!")
				lawn_contour.append(cnt)
				logger.debug('\tThe area is %.3f' % area )
				logger.debug('\tThe length is %.3f' % length)
				# Print the first child contour and the parent contour
				# logger.debug(hierarchy[0][i][2])
				if hierarchy[0][i][3] > 0:
					logger.debug('Has the parent contour no. %d' % hierarchy[0][i][3])
				logger.debug('\n')
		else:
			logger.info('No contour fouond before examining the shapes.')

		# Draw all the passed contours
		src_contour_dst = cv2.drawContours(src, lawn_contour, -1, (0,255,255), 5)
		# Draw the contours after the area check.
		cv2.imshow("Lawn Contours", src_contour_dst)
		cv2.waitKey(0)

		# Display the filtered out areas.
		# cv2.bitwise_not(src, img_color_filtered, mask_green);
		# cv2.imshow("Removed by color threshold", img_color_filtered);
		# cv2.waitKey(0);

		# # Visualize the final image
		# cv2.imshow("Final Result", dst);
		# cv2.waitKey(0);
		return src_contour_dst

	def texture_seg(self, src_gray):
		""" Genetate texture feature images. 

			Input:
				The gray scaled image.

		"""

		# Store the number of features contained.
		num_texture_feature = 0

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
		# g_kernel = cv2.getGaborKernel((15, 15), 7.0, 3.14159/12*0, 2.5, 1.0, 0, ktype=cv2.CV_32F)
		ksize = 15
		sigma = 4.0
		theta_max = np.pi
		theta_seq = np.pi/6
		theta_num = int(np.floor(theta_max/theta_seq)+1)
		wavelength_max = ksize/2
		wavelength_min = 2.5
		wavelength_num = 6
		wavelength_seq = (wavelength_max - wavelength_min)/wavelength_num
		gamma = 1.0
		psi = 0.0

		# Stored for plotting the gaber filters in the spatial domain.
		gabor_kernels = []
		theta_arr = []
		wavelength_arr = []

		# Iterate over both wavelength and theta.
		for j in range(theta_num):
			theta = j*theta_seq
			theta_arr.append( np.degrees(theta) )
			for k in range(wavelength_num):
				wavelength = wavelength_min + k*wavelength_seq
				if (j == 0):
					wavelength_arr.append(wavelength)				
				g_kernel = cv2.getGaborKernel((ksize, ksize), sigma, theta, wavelength, gamma, psi, ktype=cv2.CV_32F)
				filtered_img = cv2.filter2D(src_gray, cv2.CV_8UC3, g_kernel)
				
				# Stored for plotting the gaber filters in the spatial domain.
				gabor_kernels.append(g_kernel)

				# gray_bound_low, gray_bound_up = self.color_sample(filtered_img, self.select_precentage)
				# gray_mask, self.gray_threshold = self.color_mask(self.src, gray_bound_low, gray_bound_up)

				# filtered_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)

				logger.debug('Theta = {}, wavelength = {}.'.format(theta, wavelength))

				if (self.texture_feature_valid(filtered_img) == 0):
					if (self.feature_mat_empty):
						filtered_img = self.feature_normalize(filtered_img, self.feature_range_texture)
						self.feature_mat = filtered_img.reshape((-1, 1))
						self.feature_mat_empty = False
						num_texture_feature = 1
					else:
						filtered_img = self.feature_normalize(filtered_img, self.feature_range_texture)
						self.feature_append(self.feature_mat, filtered_img, 1)
						logger.debug('\tThe feature matrix now has the shape {}.'.format(self.feature_mat.shape))
					num_texture_feature += 1

				# cv2.imshow('Filtered Image', filtered_img)
				# cv2.imshow('Filtered Image Thresholded', gray_mask)
				# cv2.waitKey(0)

		# To plot the Gabor filters.
		if(self.show_gabor_filters):
			# plt.figure(self.fig_num)
			# self.fig_num += 1
			i = 0

			# fig, axs = plt.subplots(ncols=wavelength_num, nrows=theta_num, gridspec_kw={'hspace': 0.0, 'wspace': 0.0})
			fig, axs = plt.subplots( nrows=theta_num, ncols=wavelength_num, figsize=(6, 6) )
			plt.setp(axs.flat, xticks=[], yticks=[])

			print(theta_arr)
			print(wavelength_arr)
			# plt.setp(axes.flat, xticks=[], yticks=[])
			for ax in axs.flat:
				k = gabor_kernels[i]
				h, w = k.shape[:2]
				k = cv2.resize(k, (3*w, 3*h), interpolation=cv2.INTER_CUBIC)
				# ax.axis('off')
				ax.imshow(k, cmap='gray', norm=NoNorm())
				i += 1

			for ax, wl in zip(axs[-1], wavelength_arr):
			    ax.set_xlabel('{0}'.format(wl))
			for ax, angle in zip(axs[:, 0], theta_arr):
			    ax.set_ylabel( '{0}'.format(angle) )

			# for ax, ve in zip(axes[0], [0.1, 1, 10]):
			#     ax.set_title('{0}'.format(ve), size=18)
			# for ax, mode in zip(axes[:, 0], ['Hillshade', 'hsv', 'overlay', 'soft']):
			#     ax.set_ylabel(mode, size=18)
			plt.suptitle('Gabor filters in the spatial domain') # or plt.suptitle('Main title')
			
			fig.text(0.5, 0.04, 'Wavelength (pixel)', ha='center')
			fig.text(0.04, 0.5, 'Angle (degree)', va='center', rotation='vertical')
			# fig.tight_layout()

			plt.show()

		return num_texture_feature


	def texture_feature_valid(self, array):
		""" Check if the texture segmentation instance is valid in the 
			sense that:
				1. The variance of the feature image is high.
				2. The variance of the feature in left sample is low.
				3. The variance of the feature in right sample is low.
			Input: 
				The feature image after the gabor filter.
			Output: 
				True of False

			Note: 
				The variance is the average of the squared deviations
				from the mean, i.e., var = mean(abs(x - x.mean())**2).
		"""

		invalid_whole = 1
		invalid_left = 1
		invalid_right = 1

		array_left_sample = self.left_crop(array)
		array_right_sample = self.right_crop(array)

		texture_stdvar_threshold_high = 50
		texture_stdvar_threshold_low = 10

		logger.debug('\tThe overall std variance = {}'.format(np.std(array)))
		logger.debug('\tThe left std variance = {}'.format(np.std(array_left_sample)))
		logger.debug('\tThe right std variance = {}'.format(np.std(array_right_sample)))

		if (np.std(array) > texture_stdvar_threshold_high):
			invalid_whole = 0
		
		if (np.std(array_left_sample) < texture_stdvar_threshold_low):
			invalid_left = 0
		
		if (np.std(array_right_sample) < texture_stdvar_threshold_low):
			invalid_right = 0
		
		if (invalid_whole + invalid_left + invalid_right):
			logYellow('\tThis parameter set does not pass the threshold.')
		else:
			logCyan('\tThis parameter set passes the threshold.')


		return invalid_whole + invalid_left + invalid_right



	def feature_append(self, feature_mat, in_mat, num_feature):
		""" Append new features in preparation for clustering.
			Input:
				1. The new matrix carring the feature.
				2. Number of features beared in the coming matrix. 

			Note:
				The coming image is reshaped into an array. 
		"""

		feature_mat_new = in_mat.reshape((-1, num_feature))
		logger.debug('\tThe new feature matrix now has the shape {}.'.format(feature_mat_new.shape))
		logger.debug('\tThe feature matrix now has the shape {}.'.format(feature_mat.shape))
		self.feature_mat = np.append(feature_mat, feature_mat_new, axis = 1) 
		# self.feature_mat = np.vstack((self.feature_mat, feature_mat_new))

	def feature_normalize(self, feature_array, feature_range):
		""" Normalize the feature_array.
			Encapsulize here for convenience.
		"""

		feature_array = cv2.normalize(feature_array, None, alpha=0, beta=feature_range, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
		return feature_array


	def kmeans_seg(self, feature_mat, K):
		""" Use Kmeans to cluster on the given features.

			Input: 
				feature_mat - the image with available features.
				K - the number of the clusters.

		"""

		# Reshape the image into M*N
		# M-number of pixels; N-number of features.
		Z = self.feature_mat
		Z = np.float32(Z)

		# Define criteria, number of clusters(K) and apply kmeans()
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
		ret, label, center = cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_PP_CENTERS)
		
		# Now convert back into uint8, and make original image
		center = np.uint8(center)
		res = center[label.flatten()]
		logger.debug(Z.shape)
		logger.info("Total number of features: %d" % self.num_feature)
		res2 = label.reshape((self.fixed_height, self.fixed_width))
		plt.imshow(res2, interpolation='nearest')
		# plt.plot(res2)
		plt.show()
		# cv2.imshow('Kmeans Result', res2)
		# cv2.waitKey(0)

		return res2



if __name__ == '__main__':
    # rospy.init_node('boundary_detect')
    BoundaryDetectNode()
    # rospy.spin()
