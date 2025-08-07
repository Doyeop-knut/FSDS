#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#####################
# 수집 대상 데이터 : 
# time, ax, ay, az, roll, pitch, yaw,
# x, y, z,
# motor rpm, wheel angle
#
#####################
import rospy, rospkg
import tf
import os
import threading
import csv
from std_msgs.msg import Float32MultiArray, Float64
from sensor_msgs.msg import Imu, NavSatFix
from ublox_msgs.msg import NavSAT
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from pyproj import Proj
from math import pi, sqrt
from datetime import datetime
import numpy as np
import time
class DataLogger:
    def __init__(self):
        rospy.init_node("Data_Logger", anonymous=True)
        self.gss_sub = rospy.Subscriber("/fsds/gss",TwistWithCovarianceStamped, self.gss_callback)
        self.gps_sub = rospy.Subscriber("/fsds/gps",NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/fsds/imu",Imu, self.imu_callback)

        ## gss
        self.gss_linear_x , self.gss_linear_y, self.gss_linear_z = 0,0,0
        self.gss_angular_x , self.gss_angular_y, self.gss_angular_z = 0,0,0

        ## gps
        self.lat, self.lon, self.alt = 0,0,0

        ## imu
        self.ax, self.ay, self.az = 0,0,0
        self.roll, self.pitch, self.yaw = 0,0,0
        self.r_rate, self.p_rate, self.y_rate = 0,0,0

        self.initial_time = rospy.get_time()

        self.today = datetime.today().strftime("%m%d%H%M")
        rp = rospkg.RosPack()
        package_path = rp.get_path('tutorial')
        log_dir = os.path.join(package_path,"log")
        os.makedirs(log_dir, exist_ok=True)
        self.buffer = []
        self.buffer_lock = threading.Lock()
        
        self.data_log_file = os.path.join(log_dir,f"data_log_{self.today}.csv")
        self.rate = rospy.Rate(100)
        with open(self.data_log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["time[sec]","ax[m/s^2]","ay[m/s^2]","az[m/s^2]","roll_rate[deg/s]","pitch_rate[deg/s]","yaw_rate[deg/s]","roll[deg]","pitch[deg]","yaw[deg]","lat[deg]","lon[deg]","alt[m]","gss_linear_x[m/s]","gss_linear_y[m/s]","gss_linear_z[m/s]","gss_angular_x[rad/s]","gss_angular_y[rad/s]","gss_angular_z[rad/s]"])
        
        threading.Thread(target=self.write_data_log, daemon=True).start()
        while not rospy.is_shutdown():
            # print("Logging in progress ...")
            self.write_data_log()
            # rate.sleep()
            # self.data_logger()
            # print(self.cov_mat)
        rospy.spin()
        
    def gss_callback(self,data):
        gss_linear = data.twist.twist.linear
        gss_angular = data.twist.twist.angular

        self.gss_linear_x = gss_linear.x
        self.gss_linear_y = gss_linear.y
        self.gss_linear_z = gss_linear.z

        self.gss_angular_x = gss_angular.x
        self.gss_angular_y = gss_angular.y
        self.gss_angular_z = gss_angular.z


    def gps_callback(self,data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.alt = data.altitude

    def imu_callback(self,data):
        ## Linear Acceleration
        self.ax = data.linear_acceleration.x
        self.ay = data.linear_acceleration.y
        self.az = data.linear_acceleration.z
        
        ## angle
        q = data.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        r, p, y = euler_from_quaternion(quaternion)
        self.roll = r * 180.0 / pi
        self.pitch = p * 180.0 / pi
        self.yaw = y * 180.0/pi

        ## angular velocity
        self.r_rate = data.angular_velocity.x
        self.p_rate = data.angular_velocity.y
        self.y_rate = data.angular_velocity.z

    def data_logger(self):
        # t_old = 0
        t = rospy.get_time() - self.initial_time
        # print(t)
        # self.enc_log_data = np.append(self.enc_log_data,[[t,real_steer,desire_steer,pwm,vel,state,current_vel,con_sp,desired_vel]],axis=0)
        self.buffer.append([round(t,1),self.ax,self.ay,self.az,self.roll,self.pitch,self.yaw,self.r_rate,self.p_rate,self.y_rate,self.lat,self.lon,self.alt,self.gss_linear_x,self.gss_linear_y,self.gss_linear_z,self.gss_angular_x,self.gss_angular_y,self.gss_angular_z])
        self.rate.sleep()
        print(self.buffer)
    def write_data_log(self):
        while not rospy.is_shutdown():
            self.data_logger()
            print("LOGGING")
            with self.buffer_lock:
                buffer_copy = self.buffer[:]
                self.buffer = []
                if buffer_copy:
                    with open(self.data_log_file, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerows(buffer_copy)
if __name__ == '__main__':
    try:
        
        EncoderLog = DataLogger()
    except rospy.ROSInterruptException:
        pass