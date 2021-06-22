#! /usr/bin/python3

"""
 Use magnetometer and GNSS to estimate the yaw angle.

 Subscribe to raw GNSS and magnetometer message. Magnetometer is used for initial
 yaw angle prediction, and the GNSS estimates the heading when the difference 
 in position of the robot is sufficiently significant.

 For the continuity in angle, we append quaternion in the queue to compute variance. 

 Global frame: East-North-Up
 Unit: yaw angle, rad; length, meter

"""

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from sensor_msgs.msg import NavSatFix, Imu, MagneticField

# rospy.logdebug(msg, *args)
# rospy.logwarn(msg, *args)
# rospy.loginfo(msg, *args)
# rospy.logerr(msg, *args)
# rospy.logfatal(msg, *args)

class MagGNSSYawEstimator():
    
    def __init__(self):

        # Independent constants 
        self.degree2meter = 111320
        # Fundamental switch
        self.use_mag = False
        self.use_gnss = False
        # General configurations
        mag_topic_name = "/imu_left/imu/mag" # Need to adjust
        acc_topic_name = "/imu_left/imu/data/processed" 
        gnss_topic_name = "/GPSfix"
        self.pub_topic_name = "yaw_mag_gnss"
        self.yaw_pub = rospy.Publisher((pub_topic_name), PoseWithCovarianceStamped, queue_size=10)
        # Mag and acc specific configurations
        if (self.use_mag):
            self.mag_max_dev = 0.1
            self.mag_yaw_var = 0.0025

            self.acc_available = False
            rospy.Subscriber(mag_topic_name, MagneticField, self.mag_callback)
            rospy.Subscriber(acc_topic_name, Imu, self.acc_callback)
        # GNSS specific configuration
        if (self.use_gnss):
            # yaw angle difference
            self.gnss_yaw_stdev_threshold = 0.05
            # translational diff 
            self.trans_min_diff = 0.5

            self.last_longitude = -1.0 
            self.last_latitude = -1.0 
            self.first_fix = True
            rospy.Subscriber(gnss_topic_name, NavSatFix, self.gpsfix_callback)

        # Necessary members
        self.gnss_diff_queue = []
        self.gnss_queue_length = 10        
        self.mag_queue = []  
        self.mag_queue_length = 20

    def gpsfix_callback(self, msg):

        to_publish = False
        lati = msg.latitude
        longi = msg.longitude

        if self.first_fix:
            self.last_longitude = longi
            self.last_latitude = lati
            self.first_fix = False
        else:
            x_diff = self.degree2meter*(longi - self.last_longitude)
            y_diff = self.degree2meter*(lati - self.last_latitude)
            self.last_longitude = longi
            self.last_latitude = lati
            # The translational difference is calculated by consecutive two poses.
            if (sqrt(x_diff**2 + y_diff**2) > self.trans_min_diff):
                self.gnss_diff_queue.append([x_diff, y_diff])

            if (len(self.gnss_diff_queue) >= self.gnss_queue_length): 
                rospy.logdebug("Sufficient measurement in gnss queue")
                rospy.logdebug(gnss_diff_queue)                
                x_mean, y_mean, dev = self.compute_diff_mean_dev(self.gnss_diff_queue)
                rospy.logdebug("The means and dev are %f, %f, %f.", % x_mean, y_mean, dev) 
                self.gnss_diff_queue.pop([0])
                if (dev < self.gnss_max_dev**2):
                    to_publish = True
                    q_mean = self.xy2quaternion(x_mean, y_mean)

            if to_publish:
                yaw_estimated = PoseWithCovarianceStamped()  
                yaw_estimated.header = msg.header
                yaw_estimated.header.frame_id = "map"
                yaw_estimated.pose.pose.quaternion.x = q_mean[0]
                yaw_estimated.pose.pose.quaternion.y = q_mean[1]
                yaw_estimated.pose.pose.quaternion.z = q_mean[2]
                yaw_estimated.pose.pose.quaternion.w = q_mean[3]
                yaw_estimated.pose.covariance[35] = dev
                self.yaw_pub.publish(yaw_estimated)

    def compute_diff_mean_dev(self, queue):
        """ Compute the mean direction and the deviation w.r.t. it. """
        theta_sum = 0.0
        for x, y in queue:
            x_sum += x
            y_sum += y
        mean = [x_sum/len(queue), y_sum/len(queue)]

        for pair in queue:
            product = (np.dot(mean, pair)/sqrt(mean[0]**2 + mean[1]**2)
                    /sqrt(pair[0]**2 + pair[1]**2))
            theta_sum += math.acos(product)
        dev = theta_sum/len(queue)

        return mean[0], mean[1], dev
                
    def xy2quaternion(self, x, y):
        """ Transfer from x and y projection to quaternion here. """        
        gamma = x*x + y*y  
        beta = sqrt(gamma + x*sqrt(gamma))
        q0 = beta / (sqrt(2.0 * gamma))  
        q3 = y / (sqrt(2.0) * beta)
        q_enu = Quaternion(q0, 0.0, 0.0, q3)

        return q_enu

    def normalize_vector(self, ax, ay, az):
        factor = sqrt(ax*ax + ay*ay + az*az)
        return ax/factor, ay/factor, az/factor

    def acc_callback(self, msg):
        if not (self.mag_done):
            self.ax = msg.linear_acceleration.x
            self.ay = msg.linear_acceleration.y
            self.az = msg.linear_acceleration.z
            self.acc_available = True

    def mag_callback(self, msg):
        ready_to_publish = False
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z

        if self.acc_available and (not (self.mag_done)):
            lx, ly = calculate_q_mag(self.ax, self.ay, self.az, mx, my, mz)
            self.mag_queue.append([lx, ly])
            if (len(self.mag_queue) >= self.mag_queue_length):    
                lx_mean, ly_mean, dev = self.compute_diff_mean_var(self.mag_queue)
                self.mag_queue.pop([0])
            if (dev < self.mag_max_dev**2):
                # Here need to transfer from NWU to ENU
                q_mag_mean = self.xy2quaternion(-ly_mean, lx_mean)
                ready_to_publish = True

            if ready_to_publish:
                yaw_estimated = PoseWithCovarianceStamped()  
                yaw_estimated.header = msg.header
                yaw_estimated.header.frame_id = "map"
                yaw_estimated.pose.pose.quaternion.x = q_mag_mean[0]
                yaw_estimated.pose.pose.quaternion.y = q_mag_mean[1]
                yaw_estimated.pose.pose.quaternion.z = q_mag_mean[2]
                yaw_estimated.pose.pose.quaternion.w = q_mag_mean[3]
                yaw_estimated.pose.covariance[35] = self.mag_yaw_var

                self.yaw_pub.publish(yaw_estimated)
                self.mag_done = True

    def calculate_q_mag(self, ax, ay, az, mx, my, mz):
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
        # frame by the inverse of q_acc. l = R(q_acc)^-1 m
        lx = ((q0_acc*q0_acc + q1_acc*q1_acc - q2_acc*q2_acc)*mx + 
            2.0 * (q1_acc*q2_acc)*my - 2.0 * (q0_acc*q2_acc)*mz)
        ly = (2.0 * (q1_acc*q2_acc)*mx + (q0_acc*q0_acc - q1_acc*q1_acc + 
            q2_acc*q2_acc)*my + 2.0 * (q0_acc*q1_acc)*mz)

        return lx, ly


if __name__ == '__main__':
    rospy.init_node('mag_gnss_yaw_estimator', log_level=rospy.DEBUG)
    MagGNSSYawEstimator()
    rospy.spin()