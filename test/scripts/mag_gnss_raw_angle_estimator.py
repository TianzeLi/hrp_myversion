#! /usr/bin/python3

"""
 Use magnetometer and GNSS to estimate the yaw angle.

 Subscribe to raw GNSS and magnetometer message. Magnetometer is used for initial
 yaw angle prediction, and the GNSS estimates the heading when the difference 
 in position of the robot is sufficiently significant.

 Unit: yaw angle, rad

"""

import rospy
import numpy as np
import math


class MagGNSSYawEstimator():
    
    def __init__(self):

        self.degree2meter = 111320
        
        self.use_mag = False
        self.use_gnss = False
        self.fuse_mag_gnss = False

        mag_topic_name = "mag"
        acc_topic_name = "acc"
        gnss_topic_name = "/GPSfix"

        self.pub_topic_name = "yaw_mag_gnss"

        self.gnss_queue = []
        self.gnss_queue_length = 10        
        self.mag_queue_length = 20
        self.mag_queue = []

        if (self.use_mag):
            self.mag_yaw_stdev_threshold = 0.1

            # Subscribe to magnetometer and accelerometer for pre-processing.
            rospy.Subscriber(mag_topic_name, message_type, self.mag_callback)
            rospy.Subscriber(acc_topic_name, message_type, self.acc_callback)

        if (self.use_gnss):
            self.gnss_yaw_stdev_threshold = 0.05

            # Subscribe to gnss topic, say, /GPSfix
            rospy.Subscriber(gnss_topic_name, NavSatFix, self.gpsfix_callback)

        self.yaw_pub = rospy.Publisher((pub_topic_name), PoseWithCovarianceStamped, queue_size=10)
    

    def gpsfix_callback(self, msg):
      
        # to_pub.header.frame_id = "odom"
        # to_pub.header.frame_id = "base_link"

        # to_pub.header.stamp = rospy.Time.now()

        if (self.do_estimate_initial):
          if (self.initial_stack_number > self.number_count):
            self.initial_latitude += msg.latitude
            self.initial_longitude += msg.longitude
            self.number_count += 1
          else:
            self.initial_latitude = self.initial_latitude / self.initial_stack_number
            self.initial_longitude = self.initial_longitude / self.initial_stack_number
            self.do_estimate_initial = False

        if not self.do_estimate_initial:
          x,y = self.compute_local_corrdinate(msg.latitude, msg.longitude)
          to_pub.pose.pose.position.x = x * self.x_scale_factor
          to_pub.pose.pose.position.y = y * self.y_scale_factor
          # to_pub.pose.pose.covariance[0] = msg.position_covariance[0]
          # to_pub.pose.pose.covariance[9] = msg.position_covariance[4]
          to_pub.pose.covariance[0] = self.longitude_stdev
          to_pub.pose.covariance[9] = self.latitude_stdev
          self.pose_pub.publish(to_pub)

          yaw_var = np.var(self.yaw_queue)

          if (var < self.gnss_yaw_stdev_threshold**2):
              yaw_mean = sum(yaw_queue) / len(yaw_queue)

              to_pub = self.yaw2pose(self, yaw_mean)
              to_pub.header = msg.header
              to_pub.header.frame_id = "map"
              to_pub.pose.covariance[35] = yaw_var
              self.yaw_pub.publish(to_pub)


    def yaw2pose(self, yaw):
        # Transfer from yaw to quaternion here. 
        yaw_estimated = PoseWithCovarianceStamped()
        # q_mag is the quaternion that rotates the Global frame (North West Up) into
        # the intermediary frame. q1_mag and q2_mag are defined as 0.
        gamma = lx*lx + ly*ly;  
        beta = sqrt(gamma + lx*sqrt(gamma));
        q0_mag = beta / (sqrt(2.0 * gamma));  
        q3_mag = ly / (sqrt(2.0) * beta); 

      retuen q


    def normalize_vector(self, ax, ay, az):
        factor = sqrt(ax*ax + ay*ay + az*az)
        return ax/factor, ay/factor, az/factor

    def acc_callback(self, msg):
        self.ax = 
        self.ay = 
        self.az = 

    def mag_callback(self, msg):

        mx, my, mz = msg.

        if not (self.mag_done):
            yaw = calculate_mag_yaw(ax, ay, az, mx, my, mz)
            yaw_queue.append(yaw)
            if (len(yaw_queue) > self.mag_queue_length):
                yaw_queue.pop([0])

            yaw_var = np.var(self.yaw_queue)
            if (var < self.gnss_yaw_stdev_threshold**2):
                yaw_mean = sum(yaw_queue) / len(yaw_queue)


            self.yaw_pub.publish(to_pub)
            self.mag_done = True



    def calculate_mag_yaw(self, ax, ay, az, mx, my, mz):
    """ Compute the initial global yaw angle from mag and acc. 
        Code converted from CPT filter in the imu_tools package.
    """
        # Normalize acceleration vector.
        ax, ay, az = self.normalize_vector(ax, ay, az);
        if (az >= 0):
            q0_acc =  sqrt((az+1) * 0.5)	
            q1_acc = -ay / (2.0*q0_acc)
            q2_acc =  ax / (2.0*q0_acc)
            q3_acc = 0
        else: 
            X = sqrt((1-az) * 0.5)
            q0_acc = -ay / (2.0*X)
            q1_acc = X
            q2_acc = 0
            q3_acc = ax / (2.0 * X)
        
        # [lx, ly, lz] is the magnetic field reading, rotated into the intermediary
        # frame by the inverse of q_acc.
        # l = R(q_acc)^-1 m
        lx = ((q0_acc*q0_acc + q1_acc*q1_acc - q2_acc*q2_acc)*mx + 
            2.0 * (q1_acc*q2_acc)*my - 2.0 * (q0_acc*q2_acc)*mz)
        ly = (2.0 * (q1_acc*q2_acc)*mx + (q0_acc*q0_acc - q1_acc*q1_acc + 
            q2_acc*q2_acc)*my + 2.0 * (q0_acc*q1_acc)*mz)

        # math.atan2(y, x) Return atan(y / x), in radians. 
        # The result is between -pi and pi. 
        yaw = math.atan2(ly, lx)



if __name__ == '__main__':
    rospy.init_node('mag_gnss_yaw_estimator')
    MagGNSSYawEstimator()
    rospy.spin()