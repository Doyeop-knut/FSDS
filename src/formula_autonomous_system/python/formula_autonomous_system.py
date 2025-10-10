#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file formula_autonomous_system.py
@author Jiwon Seok (jiwonseok@hanyang.ac.kr)
@brief Formula Student Driverless Autonomous System - Python Implementation
@version 0.1
@date 2025-07-25

@copyright Copyright (c) 2025
"""

import rospy
import numpy as np
import cv2
import math
import random
from enum import Enum
from typing import List, Tuple, Optional
import time
from collections import deque, namedtuple

import tf2_ros
from geometry_msgs.msg import TransformStamped, Point

# ROS
from std_msgs.msg import String
from fs_msgs.msg import ControlCommand
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, Image, Imu, NavSatFix, PointField
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

# 3D LiDAR
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.interpolate import splprep, splev
from scipy.spatial import Delaunay

# Data Logger
import os
import csv
import datetime
 
# ==================== Enums ====================
class AutonomousMode(Enum):
    AS_OFF = 0
    AS_READY = 1
    AS_DRIVING = 2

class AutonomousEvent(Enum):
    SYSTEM_INIT = 0
    SYSTEM_READY = 1
    GO_SIGNAL = 2
# ==================== Main System ====================

class FormulaAutonomousSystem:
    def __init__(self):
        self.is_initialized = False
        self.x_min, self.x_max = 0,0
        self.y_min, self.y_max = 0,0
        self.z_min, self.z_max = 0,0
        self.ransac_iter = 0
        self.ransac_distance = 0
        self.dbscan_eps = float()
        self.dbscan_points = 0
        self.prev_x, self.prev_y, self.prev_z = 0,0,0
        
        # ==================== 데이터 로거 추가 ====================
        self.data_logger = DataLogger(
        log_directory="/home/user/fsds_ws/src/tutorial/log",
        session_name=datetime.datetime.now().strftime("%Y%m%d_%H%M%S"),
        max_lidar_points=50  # 필요시 이 값을 조절
        )
        # =========================================================
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Publishers for visualization
        self.path_publisher = rospy.Publisher("/centerline_path", Marker, queue_size=1)
        self.map_cone_publisher = rospy.Publisher("/map_cones", MarkerArray, queue_size=1)
        self.triangulation_publisher = rospy.Publisher("/delaunay_triangulation", Marker, queue_size=1)

        # System Components
        self.gps_util = GPSIMUProcessor()
        self.state_machine = StateMachine()
        self.lidar_util = LiDARProcessor()
        self.camera_util = CameraProcessor()
        self.track_map = TrackMap()
        self.path_planner = PathPlanner()
        
    def init(self):
        """Initialize the system"""
        self.is_initialized = True
        return True

    def get_parameters(self):
        """Get parameters from ROS parameter server"""
        self.x_min, self.x_max = rospy.get_param("/perception/lidar_roi_extraction/x_min") , rospy.get_param("/perception/lidar_roi_extraction/x_max")
        self.y_min, self.y_max = rospy.get_param("/perception/lidar_roi_extraction/y_min") , rospy.get_param("/perception/lidar_roi_extraction/y_max")
        self.z_min, self.z_max = rospy.get_param("/perception/lidar_roi_extraction/z_min") , rospy.get_param("/perception/lidar_roi_extraction/z_max")
        self.ransac_iter = rospy.get_param("/perception/lidar_ground_removal/ransac_iterations")
        self.ransac_distance = rospy.get_param("/perception/lidar_ground_removal/ransac_distance_threshold")
        self.dbscan_eps = rospy.get_param("/perception/lidar_clustering/dbscan_eps")
        self.dbscan_points = rospy.get_param("/perception/lidar_clustering/dbscan_min_points")
        self.left_tx, self.left_ty, self.left_tz = rospy.get_param("/perception/camera_extrinsics/translation_x"), rospy.get_param("/perception/camera_extrinsics/translation_y"),rospy.get_param("/perception/camera_extrinsics/translation_z")
        self.left_rr, self.left_rp, self.left_ry = rospy.get_param("/perception/camera_extrinsics/rotation_roll"), rospy.get_param("/perception/camera_extrinsics/rotation_pitch"), rospy.get_param("/perception/camera_extrinsics/rotation_yaw")
        self.right_tx, self.right_ty, self.right_tz = rospy.get_param("/perception/camera_right_extrinsics/translation_x"), rospy.get_param("/perception/camera_right_extrinsics/translation_y"),rospy.get_param("/perception/camera_right_extrinsics/translation_z")
        self.right_rr, self.right_rp, self.right_ry = rospy.get_param("/perception/camera_right_extrinsics/rotation_roll"), rospy.get_param("/perception/camera_right_extrinsics/rotation_pitch"), rospy.get_param("/perception/camera_right_extrinsics/rotation_yaw")
        return True

    def run(self, lidar_msg, camera1_msg, camera2_msg, imu_msg, gps_msg, go_signal_msg):
        """Run the autonomous system (Pythonic version)
        
        Returns:
            tuple: (success, control_command, autonomous_mode)
                - success (bool): 처리 성공 여부
                - control_command (ControlCommand): 제어 명령
                - autonomous_mode (String): 자율주행 모드 상태
        """
        # print(self.state_machine.state)

         # 시스템 초기화 확인
        if not self.is_initialized:
            rospy.logwarn_throttle(1.0, "FormulaAutonomousSystem: Not initialized")
            return False
        self.get_parameters()

        autonomous_mode = String()
        autonomous_mode.data = "AS_OFF"
        self.state_machine.inject_system_init()

        acc, gyro, orientation = self.get_imu_data(imu_msg)
        imu_data = [acc[0], acc[1], gyro[2]]
        roll,pitch,yaw = self.gps_util.Quat_to_Euler(orientation)
        lat, lon, alt = self.get_gps_data(gps_msg)
        # vtL = self.lidar_util.vehicle_to_lidar_Transform()
        # print(vtL)
        gps_data = self.gps_util.gps_to_local(lat, lon)
        self.gps_util.updateIMU(imu_data, yaw, imu_msg.header.stamp.to_sec())
        self.gps_util.updateGPS(gps_data,gps_msg.header.stamp.to_sec())
        vehicle_state = self.gps_util.state

        # ==================== TF Publisher ====================
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = vehicle_state[0]
        t.transform.translation.y = vehicle_state[1]
        t.transform.translation.z = 0.0

        # Convert yaw to quaternion
        veh_yaw = vehicle_state[2]
        q_z = math.sin(veh_yaw / 2.0)
        q_w = math.cos(veh_yaw / 2.0)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        
        self.tf_broadcaster.sendTransform(t)
        # =====================================================

        rospy.loginfo_throttle(1.0,f"v = {round(math.sqrt(vehicle_state[3]**2 + vehicle_state[4]**2),4)} m/s")
        # print(f"state = {self.gps_util.state}, yaw = {math.degrees(self.gps_util.state[2])}")
        ## LiDAR Processed
        # print(f"parameters = {self.dbscan_eps, self.dbscan_points, self.ransac_distance, self.ransac_iter, self.x_min, self.x_max}")
        points=self.get_lidar_point_cloud(lidar_msg)
        filtered = self.lidar_util.filtering_points(points, (self.x_min, self.x_max), (self.y_min, self.y_max), (self.z_min, self.z_max))
        removal =  self.lidar_util.ransac_plane_removal(filtered, threshold=self.ransac_distance, max_trials=self.ransac_iter)
        # print(self.dbscan_eps)
        cluster = self.lidar_util.cluster_points(removal, eps=self.dbscan_eps, min_samples=self.dbscan_points)
        # print(f"cluster = {cluster}")
        # print(f"gps_data = {gps_data}")
        # print(f"x = {self.gps_util.state[0]}, y = {self.gps_util.state[1]}, yaw = {math.degrees(self.gps_util.state[2])}")
        # print(cluster*math.sin(yaw))
       
        image1 = self.get_camera_image(camera1_msg)
        image2 = self.get_camera_image(camera2_msg)
        cam_mat = self.camera_util.cam_matrix()
        cam1_transform = self.camera_util.transform_matrix(-self.left_tx, -self.left_ty, -self.left_tz, self.left_rr, self.left_rp, self.left_ry)
        cam2_transform = self.camera_util.transform_matrix(-self.right_tx, -self.right_ty, -self.right_tz, self.right_rr, self.right_rp, self.right_ry)
        # print(cam2_transform)
        image1 = self.camera_util.preprocessImage(image1)
        image2 = self.camera_util.preprocessImage(image2)
        
        # --- Time Synchronization Compensation ---
        # Compensate for vehicle motion between LiDAR scan time and Camera image time
        # This is a first-order correction for high-speed alignment issues.
        try:
            # Note: A negative dt means the lidar message is newer than the camera, which is unusual but possible.
            # The compensation will move the points backward in that case, which is correct.
            dt_cam_lidar = camera1_msg.header.stamp.to_sec() - lidar_msg.header.stamp.to_sec()
            vx = vehicle_state[3] # Longitudinal velocity
            compensation_dist = vx * dt_cam_lidar
            
            compensated_cluster = cluster.copy()
            compensated_cluster[:, 0] += compensation_dist # Add distance to the x-component (forward)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Could not perform time compensation: {e}")
            compensated_cluster = cluster
        # --- End Compensation ---

        # print(color)
        cam1_pts = self.camera_util.projectToCam(compensated_cluster, cam1_transform)
        cam2_pts = self.camera_util.projectToCam(compensated_cluster, cam2_transform)
        # color = self.camera_util.detectConeColor(cam1_pts, image1)

        if cluster.size > 0:
            # Transform cluster points to map frame
            veh_x = self.gps_util.state[0]
            veh_y = self.gps_util.state[1]
            veh_yaw = self.gps_util.state[2]  # Assumes radians

            cos_yaw = math.cos(veh_yaw)
            sin_yaw = math.sin(veh_yaw)
            rot_matrix = np.array([[cos_yaw, -sin_yaw],
                                   [sin_yaw,  cos_yaw]])
            # Prepare a list to store cones with color information
            cones_with_color = []
            left, right = [], []
            # Iterate through each 3D cluster point and its corresponding 2D projected point
            for i, cone_3d_veh_frame in enumerate(cluster):
                # Project 3D cone to camera 1 image plane
                # Note: cam1_pts is a list of projected points, need to get the i-th one
                if i < len(cam1_pts):
                    cone_2d_img_frame = cam1_pts[i]
                    detected_color_left = self.camera_util.detectConeColor(cone_2d_img_frame, image1, debug_image=image1)
                    detected_color_right = self.camera_util.detectConeColor(cam2_pts[i], image2, debug_image=image2)
                    
                    color_id = 0 # Default to unknown
                    if detected_color_left == "blue" or detected_color_right == "blue":
                        # print("Blue cone detected")
                        color_id = 1
                    elif detected_color_left == "yellow" or detected_color_right == "yellow":
                        # print("Yellow cone detected")
                        color_id = 2
                    elif detected_color_left == "orange" or detected_color_right == "orange":
                        # print("Orange cone detected")
                        color_id = 3
                    
                    # Transform 3D cone from vehicle frame to map frame
                    # print(f"3D Cone (Vehicle Frame): {cone_3d_veh_frame}, Color ID: {color_id}")
                    cone_3d_map_frame_xy = np.dot(cone_3d_veh_frame[:2], rot_matrix.T) + np.array([veh_x, veh_y])
                    cone_3d_map_frame_z = cone_3d_veh_frame[2] # Z-coordinate remains the same
                    
                    cones_with_color.append([cone_3d_map_frame_xy[0], cone_3d_map_frame_xy[1], cone_3d_map_frame_z, color_id])
                    if color_id == 1:
                        left.append([cone_3d_map_frame_xy[0], cone_3d_map_frame_xy[1]])
                    elif color_id == 2:
                        right.append([cone_3d_map_frame_xy[0], cone_3d_map_frame_xy[1]])
            
            global_clusters = np.array(cones_with_color)
        else:
            global_clusters = np.empty((0, 4)) # Ensure global_clusters is always a 2D array with 4 columns

        # ==================== Map & Path ===================
        # Update map with new cone observations
        self.track_map.update(global_clusters)

        # Plan path using the map
        path, tri, tri_points = self.path_planner.plan_path(self.track_map.get_cones(), vehicle_state[:2])
        
                    # Visualize Map and Path
        self.publish_map_cones()
        self.publish_triangulation(tri, tri_points)
        if path is not None:
            self.publish_path(path)
        # =====================================================
        img1 = self.camera_util.visualization(cam1_pts, image1)
        img2 = self.camera_util.visualization(cam2_pts, image2)
        cv2.imshow("image1", img1)
        cv2.imshow("image2", img2)
        # cv2.imshow("Camera1", self.get_camera_image(camera1_msg))
        # cv2.imshow("Camera2", self.get_camera_image(camera2_msg))
        cv2.waitKey(1)
        
        # print(f"left = {left}, right = {right}")

        ## LiDAR Cone mean point calculate
        # min_left, min_right = math.inf, math.inf
        # for p in left:
        #     # print(f"distance  = {math.sqrt(r[0]**2+r[1]**2)}")
        #     distance = math.sqrt(p[0]**2 + p[1] **2)
        #     if min_left > distance:
        #         min_left = distance
        #         left_point = [p[0],p[1]]
        #     # print(f"minimum_distance_left = {min_left}")
        # for r in right:
        #     # print(f"distance  = {math.sqrt(r[0]**2+r[1]**2)}")
        #     distance = math.sqrt(r[0]**2 + r[1] **2)
        #     if min_right > distance:
        #         min_right = distance
        #         right_point = [r[0],r[1]]
        #     # print(f"minimum_distance_right  = {min_right}")

        # mean_point = [(left_point[0] + right_point[0]) / 2, (left_point[1] + right_point[1])/ 2]
        # # print(mean_point)
        self.lidar_util.publish_point_cloud(cluster)
        # print(len(cluster))
        

        ## GO_SIGNAL
        if go_signal_msg.mission != "None" and go_signal_msg.mission != "":
            self.state_machine.inject_go_signal(go_signal_msg.mission, go_signal_msg.track)
        autonomous_mode.data = self.state_machine.get_current_state_string()

        # filtered_points = LiDARProcessor().filtering_points(np.array([[x,y,z]]), (1.0, 20.0), (-10.0, 10.0), (-0.5, 0.5))
        # print("Filtered Points:", filtered_points)
        ## GPS velocity
        # Control
        control_command_msg = ControlCommand()
        # print(self.state_machine.current_state == AutonomousMode.AS_DRIVING)#, len(waypoints))
        if self.state_machine.current_state == AutonomousMode.AS_DRIVING and path is not None and len(path) > 0:
            throttle, steer, brake = 0,0,0
            # TODO: Implement MPC controller here using the 'path'
            # print(throttle, steer, brake)
            control_command_msg.throttle = throttle
            control_command_msg.steering = steer
            control_command_msg.brake = brake
        # print(go_signal_msg)

        # ==================== Data Logger (Test) ====================
        self.data_logger.log_entry(
            autonomous_mode=autonomous_mode.data,
            control_command=control_command_msg,
            imu_acc=acc,
            imu_gyro=gyro,
            state= vehicle_state,
            camera1_image=img1,
            camera2_image=img2,
            lidar_points=global_clusters,
            map_cones=self.track_map.get_cones()
        )
        # =========================================================
        
        # plt.axis([-50,50,-20,200])
        # if len(cluster) == 0:
        #     pass
        
        # plt.scatter(x=cluster[:,0] + offset[0] ,y=cluster[:,1]+ offset[1])
        # plt.pause(0.001)
        
        # plt.clf()
        # # print(go_signal_msg.mission, go_signal_msg.track)

        return True, control_command_msg, autonomous_mode

    def publish_map_cones(self):
        marker_array = MarkerArray()
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        # Add markers for each cone in the map
        for cone in self.track_map.get_cones():
            marker = Marker()
            marker.header = header
            marker.ns = "map_cones"
            marker.id = cone['id']
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = cone['x']
            marker.pose.position.y = cone['y']
            marker.pose.position.z = cone.get('z', 0.0) # Use z if available
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.5
            marker.color.a = 0.8
            if cone['color_id'] == 1: # Blue
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif cone['color_id'] == 2: # Yellow
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif cone['color_id'] == 3: # Orange
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            else: # Unknown
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
            marker_array.markers.append(marker)
        
        self.map_cone_publisher.publish(marker_array)

    def publish_path(self, path):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "centerline_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Line width
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for point in path:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.1 # slightly above ground
            marker.points.append(p)

        self.path_publisher.publish(marker)

    def publish_triangulation(self, tri, points):
        if tri is None or points is None:
            # Clear previous markers if triangulation is not available
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"
            marker.ns = "delaunay_mesh"
            marker.id = 0
            marker.action = Marker.DELETEALL
            self.triangulation_publisher.publish(marker)
            return

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "delaunay_mesh"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # Line width
        marker.color.a = 0.4
        marker.color.r = 0.6
        marker.color.g = 0.6
        marker.color.b = 0.6 # Gray color for the mesh

        # Use the same max edge length from the path planner for consistency
        max_len_sq = self.path_planner.max_edge_length ** 2

        # tri.simplices contains the indices of the points forming each triangle
        for simplex in tri.simplices:
            # Add the 3 edges of the triangle to the line list
            for i in range(3):
                p1_idx = simplex[i]
                p2_idx = simplex[(i + 1) % 3]
                
                p1_coords = points[p1_idx]
                p2_coords = points[p2_idx]

                # Filter out long edges to clean up the visualization
                dist_sq = (p1_coords[0] - p2_coords[0])**2 + (p1_coords[1] - p2_coords[1])**2
                if dist_sq < max_len_sq:
                    p1 = Point()
                    p1.x = p1_coords[0]
                    p1.y = p1_coords[1]
                    p1.z = 0.0
                    
                    p2 = Point()
                    p2.x = p2_coords[0]
                    p2.y = p2_coords[1]
                    p2.z = 0.0

                    marker.points.append(p1)
                    marker.points.append(p2)
                
        self.triangulation_publisher.publish(marker)

    def get_lidar_point_cloud(self, msg):
        """Convert ROS PointCloud2 message to point cloud"""
        pointcloud = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[:3]
            pointcloud.append([x, y, z])
            
        return np.array(pointcloud)

    def get_camera_image(self, msg):
        """Convert ROS Image message to OpenCV Mat"""
        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            return cv_image
        except Exception as e:
            rospy.logerr(f"cv_bridge exception: {e}")
            return None

    def get_imu_data(self, msg):
        """Extract IMU data from ROS message"""
        # Extract orientation
        orientation = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        
        # Extract acceleration
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        
        # Extract angular velocity
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        
        return acc, gyro, orientation
    
    def get_gps_data(self, msg):
        """Extract GPS data from ROS message"""
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        return latitude, longitude, altitude
class CameraProcessor:
    def __init__(self):
        self.fx, self.fy = rospy.get_param("/perception/camera_intrinsics/focal_length_x"), rospy.get_param("/perception/camera_intrinsics/focal_length_y")
        self.px, self.py = rospy.get_param("/perception/camera_intrinsics/principal_point_x"), rospy.get_param("/perception/camera_intrinsics/principal_point_y")
        self.preprocess = rospy.get_param("/perception/camera_image_processing/enable_preprocessing")
        self.sigma = rospy.get_param("/perception/camera_image_processing/gaussian_blur_sigma")
        self.bilateral = rospy.get_param("/perception/camera_image_processing/bilateral_filter_diameter")
        
        self.hsv_window_size = rospy.get_param("/perception/camera_hsv_window_size/window_size", 15)

        self.hsv_yellow_min = np.array([rospy.get_param("/perception/camera_hsv_yellow/hue_min"), rospy.get_param("/perception/camera_hsv_yellow/saturation_min"), rospy.get_param("/perception/camera_hsv_yellow/value_min")])
        self.hsv_yellow_max = np.array([rospy.get_param("/perception/camera_hsv_yellow/hue_max"), 255, 255])

        self.hsv_blue_min = np.array([rospy.get_param("/perception/camera_hsv_blue/hue_min"), rospy.get_param("/perception/camera_hsv_blue/saturation_min"), rospy.get_param("/perception/camera_hsv_blue/value_min")])
        self.hsv_blue_max = np.array([rospy.get_param("/perception/camera_hsv_blue/hue_max"), 255, 255])

        self.hsv_orange_min = np.array([rospy.get_param("/perception/camera_hsv_orange/hue_min"), rospy.get_param("/perception/camera_hsv_orange/saturation_min"), rospy.get_param("/perception/camera_hsv_orange/value_min")])
        self.hsv_orange_max = np.array([rospy.get_param("/perception/camera_hsv_orange/hue_max"), 255, 255])

    def cam_matrix(self):
        camera_matrix = [
            [self.fx, 0, self.px],
            [0,self.fy, self.py],
            [0, 0, 1]
        ]
        return camera_matrix
    
    def transform_matrix(self, x, y, z, r, p, yaw):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(yaw), math.sin(yaw)

        T = [
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, x,],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, y],
            [-sp,   cp*sr,            cp*cr,            z],
            [0,    0,                0,                1]
        ]
        return T
    def preprocessImage(self, rgb_image):
        if not self.preprocess:
            return rgb_image
        processed_image = rgb_image.copy()
        if self.sigma > 0 :
            kernel_size = (2* self.sigma *3 + 1)
            if kernel_size % 2 == 0 : kernel_size += 1
            # print(kernel_size)
            processed = cv2.GaussianBlur(processed_image,(0, 0), self.sigma)
        
        if self.bilateral > 0:
            processed = cv2.bilateralFilter(processed_image, self.bilateral, 80,80)
        return processed
    
    def projectToCam(self, points, transform):
        projected_points = []
        
        # The transform matrix represents T_camera_from_vehicle.
        # It should be used directly without transposition.
        T = np.array(transform)
        rotation = T[:3, :3]
        translation = T[:3, 3] # Translation is the last column of the original matrix

        for point in points:
            # point is a 3D point in the vehicle (odom) frame
            cone_point_in_base = np.array([point[0], point[1], point[2]])
            
            # Apply the transformation: p_camera = R * p_vehicle + t
            cone_point_in_cam = np.dot(rotation, cone_point_in_base) + translation

            # The point is now in the camera's ROS-standard coordinate system (X-fwd, Y-left, Z-up)
            # We need to check if the point is in front of the camera before proceeding.
            # In the ROS convention for cameras, the X axis points forward.
            if cone_point_in_cam[0] <= 0:
                continue

            # Convert from ROS camera coordinates (X-fwd, Y-left, Z-up)
            # to standard computer vision/image coordinates (Z-fwd, X-right, Y-down)
            z_cv = cone_point_in_cam[0]  # ROS X -> CV Z
            x_cv = -cone_point_in_cam[1] # ROS Y -> CV X (Y-left = -X-right)
            y_cv = -cone_point_in_cam[2] # ROS Z -> CV Y (Z-up   = -Y-down) <--- Corrected sign

            # Perform pinhole projection
            # u = fx * (X/Z) + px
            # v = fy * (Y/Z) + py
            u = self.fx * (x_cv / z_cv) + self.px
            v = self.fy * (y_cv / z_cv) + self.py
            
            projected_points.append((u, v))
        return projected_points


        # print(f"base = {cone_point_in_base}, cam = {cone_point_in_cam}")
        # return (u,v)
    def visualization(self, points, rgb_image):
        viz = rgb_image.copy()
        image_size = rgb_image.shape
        for point in points:
            if 0 <= point[0] < image_size[1] and 0 <= point[1] < image_size[0]:
                projected = (int(point[0]), int(point[1]))
                cv2.circle(viz, projected, 10, (0, 0, 0), 2)
        return viz
    
    def detectConeColor(self, cone_point_img, rgb_image, debug_image=None):
        # cone_point_img is expected to be a single (u, v) tuple or list
        if cone_point_img is None or not isinstance(cone_point_img, (tuple, list)) or len(cone_point_img) != 2:
            return "unknown"
        # cv2.imshow("debug", cv2.cvtColor(rgb_image,cv2.COLOR_BGR2HSV))
        u, v = int(cone_point_img[0]), int(cone_point_img[1])
        
        # Define ROI around the cone
        half_window = self.hsv_window_size // 2
        x_min = max(0, u - half_window)
        x_max = min(rgb_image.shape[1], u + half_window)
        y_min = max(0, v - half_window)
        y_max = min(rgb_image.shape[0], v + half_window)

        # Check if ROI is valid
        if x_max <= x_min or y_max <= y_min:
            if debug_image is not None and (0 <= u < rgb_image.shape[1] and 0 <= v < rgb_image.shape[0]):
                cv2.circle(debug_image, (u, v), 8, (0, 0, 255), -1) # Draw red dot for invalid ROI
            return "unknown"

        # Draw the ROI on the debug image if provided
        if debug_image is not None:
            cv2.rectangle(debug_image, (x_min, y_min), (x_max, y_max), (0, 255, 255), 1)

        roi = rgb_image[y_min:y_max, x_min:x_max]

        if roi.size == 0: # Check if ROI is empty
            return "unknown"

        # Convert ROI to HSV
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Detect colors
        color_counts = {
            "yellow": 0,
            "blue": 0,
            "orange": 0
        }

        # Yellow
        mask_yellow = cv2.inRange(hsv_roi, self.hsv_yellow_min, self.hsv_yellow_max)
        color_counts["yellow"] = cv2.countNonZero(mask_yellow)

        # Blue
        mask_blue = cv2.inRange(hsv_roi, self.hsv_blue_min, self.hsv_blue_max)
        color_counts["blue"] = cv2.countNonZero(mask_blue)

        # Orange
        mask_orange = cv2.inRange(hsv_roi, self.hsv_orange_min, self.hsv_orange_max)
        color_counts["orange"] = cv2.countNonZero(mask_orange)

        # Determine dominant color
        max_count = 0
        dominant_color = "unknown"
        for color, count in color_counts.items():
            if count > max_count:
                max_count = count
                dominant_color = color
        
        # A threshold can be added here to avoid detecting noise as a color
        # For example, if max_count is too low, return "unknown"
        if max_count < (roi.size * 0.1): # e.g., at least 10% of ROI pixels must be of a color
            dominant_color = "unknown"

        if debug_image is not None and dominant_color != "unknown":
             # Put text for the detected color
             cv2.putText(debug_image, dominant_color, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)

        return dominant_color
        

class LiDARProcessor:
    def __init__(self):
        self.lidar_publisher = rospy.Publisher("/processed_lidar", PointCloud2, queue_size=1)
        self.marker_publisher = rospy.Publisher("/cluster_indices", MarkerArray, queue_size=1)
        self.last_marker_count = 0
        self.x_min, self.x_max = rospy.get_param("/perception/lidar_roi_extraction/x_min") , rospy.get_param("/perception/lidar_roi_extraction/x_max")
        self.y_min, self.y_max = rospy.get_param("/perception/lidar_roi_extraction/y_min") , rospy.get_param("/perception/lidar_roi_extraction/y_max")
        self.z_min, self.z_max = rospy.get_param("/perception/lidar_roi_extraction/z_min") , rospy.get_param("/perception/lidar_roi_extraction/z_max")
        self.ransac_iter = rospy.get_param("/perception/lidar_ground_removal/ransac_iterations")
        self.ransac_distance = rospy.get_param("/perception/lidar_ground_removal/ransac_distance_threshold")
        self.dbscan_eps = rospy.get_param("/perception/lidar_clustering/dbscan_eps")
        self.dbscan_points = rospy.get_param("/perception/lidar_clustering/dbscan_min_points")

        self.trans_x,self.trans_y,self.trans_z = rospy.get_param("/perception/lidar_extrinsics/translation_x"), rospy.get_param("/perception/lidar_extrinsics/translation_y"), rospy.get_param("/perception/lidar_extrinsics/translation_z")
        self.rot_r, self.rot_p, self.rot_yaw = rospy.get_param("/perception/lidar_extrinsics/rotation_roll"), rospy.get_param("/perception/lidar_extrinsics/rotation_pitch"), rospy.get_param("/perception/lidar_extrinsics/rotation_yaw")
        # self.gps_util = GPSIMUProcessor()
    def vehicle_to_lidar_Transform(self, x, y, z, r, p, yaw):
        veh_to_LiDAR = [
            [math.cos(yaw)* math.cos(p), math.cos(yaw)*math.sin(p)*math.sin(r) - math.sin(yaw)*math.cos(r), math.cos(yaw)*math.sin(p)*math.cos(r)+math.sin(yaw)*math.sin(r), x],
            [math.sin(yaw)* math.cos(p), math.sin(yaw)*math.sin(p)*math.sin(r) + math.cos(yaw)*math.cos(r), math.sin(yaw)*math.sin(p)*math.cos(r)- math.cos(yaw)*math.sin(r),y],
            [-math.sin(p) , math.cos(p)* math.sin(r), math.cos(p)*math.cos(r), z],
            [0,0,0,1]
        ]
        return veh_to_LiDAR

    def filtering_points(self, points: np.ndarray, x_range: Tuple[float, float], y_range: Tuple[float, float], z_range: Tuple[float, float]) -> np.ndarray:
        """Filter points within specified ranges"""
        mask = (
            (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
            (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
            (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
        )
        return points[mask]
    
    def ransac_plane_removal(self, points: np.ndarray, threshold: float = 0.05, max_trials: int = 100) -> np.ndarray:
        """Remove ground plane using RANSAC"""
        if points is None or len(points) < 10: # RANSAC을 위해 최소 포인트 수 확보
            return np.array([])

        X = points[:, 0:2] # x, y 좌표
        y = points[:, 2]   # z 좌표
        
        # RANSAC Regressor 모델 생성
        ransac = RANSACRegressor(
            residual_threshold=threshold,
            random_state=0
        )
        ransac.fit(X, y)
        
        # inlier_mask_는 지면(inlier)에 해당하는 포인트는 True, 아니면 False
        inlier_mask = ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)

        # 지면과 객체(콘 후보) 포인트를 분리
        ground_points = points[inlier_mask]
        object_points = points[outlier_mask]
        return object_points

    def cluster_points(self, points: np.ndarray, eps: float = 0.5, min_samples: int = 5) -> np.ndarray:
        """Cluster points using DBSCAN"""
        if points is None or len(points) == 0:
            return np.array([])
        
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = db.labels_
        unique_labels = set(labels)
        
        mat=self.vehicle_to_lidar_Transform(self.trans_x,self.trans_y,self.trans_z,self.rot_r,self.rot_p,self.rot_yaw)
        # rot_mat = self.vehicle_to_lidar_Transform(self.gps_util.state[0], self.gps_util.state[1],0,0,0, self.gps_util.state[2])
        # print(f"GPS = {self.gps_util.state[0], self.gps_util.state[1]}, yaw = {math.degrees(self.gps_util.state[2])}")
        # print(f"rotation matrix = {rot_mat}, translation matrix = {mat}")

        clusters = []
        for label in unique_labels:
            if label == -1:
                continue
            cluster = points[labels == label]
            center = np.mean(cluster, axis=0)
            center_4d = np.array([center[0],center[1],center[2],1])
            
            transform_lidar = np.dot(mat, center_4d)
            # print(f"before = {transform_lidar}")
            # print(f"original = {center}, transformed = {rotated_lidar}")
            # print(f"transformed = {transform_lidar}")
            # if math.sqrt(transform_lidar[0]**2 + transform_lidar[1]**2) < 5.0:
            clusters.append(transform_lidar[:3])  # Append transformed x, y, z   
                # print(f"transformed = {transform_lidar}")     
            # if math.sqrt(center[0]**2 + center[1]**2) < 5.0:
                # print(f"original = {center}")
            # print(center)

        return np.array(clusters)
    
        

    def left_right_split(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Split points into left and right based on y-coordinate"""
        left_points = points[points[:, 1] > 0]
        right_points = points[points[:, 1] <= 0]
        return left_points, right_points
    
    ### For Debugging: Publish Processed Point Cloud ###
    def publish_point_cloud(self, points: np.ndarray):
        """Publish processed point cloud and their indices as markers"""

        clusters = points

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "odom"
        
        # Publish point cloud
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        point_cloud_msg = pc2.create_cloud(header, fields, clusters)
        self.lidar_publisher.publish(point_cloud_msg)

        # Publish markers for indices
        marker_array = MarkerArray()
        
        # Add text markers for each cluster
        for i, point in enumerate(clusters):
            marker = Marker()
            marker.header = header
            marker.ns = "cluster_indices"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2] + 0.5  # Offset text above the point
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.5  # Text size
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.text = str(i)
            marker_array.markers.append(marker)

        # Add delete markers for old markers that are no longer present
        for i in range(len(clusters), self.last_marker_count):
            marker = Marker()
            marker.header = header
            marker.ns = "cluster_indices"
            marker.id = i
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self.last_marker_count = len(clusters)
        if len(marker_array.markers) > 0:
            self.marker_publisher.publish(marker_array)

# ==================== Map & Path Planner ===================

class TrackMap:
    def __init__(self):
        self.cones = []  # 지도에 저장된 콘 리스트. 예: [{'id': 0, 'x': 10.2, 'y': 3.1, 'color_id': 1, 'covariance': np.eye(2)*0.1}]
        self.next_cone_id = 0
        self.association_threshold = 1.5  # 1.5미터 안에 있으면 같은 콘으로 간주

    def update(self, new_cones_observations):
        """
        새로 감지된 콘(observations)을 기반으로 지도를 업데이트합니다.
        new_cones_observations: [[x, y, z, color_id], ...] 형태의 numpy 배열
        """
        if not self.cones: # 지도가 비어있으면
            for cone_obs in new_cones_observations:
                self._add_new_cone(cone_obs)
            return

        if new_cones_observations.size == 0:
            return # 새로운 관측이 없으면 업데이트 안함

        # 데이터 연관 수행
        # 1. 지도에 있는 콘들과 새로 관측된 콘들 간의 거리 행렬 계산
        map_cone_positions = np.array([[c['x'], c['y']] for c in self.cones])
        obs_cone_positions = new_cones_observations[:, :2]
        distance_matrix = cdist(map_cone_positions, obs_cone_positions)

        matched_obs_indices = set()
        matched_map_indices = set()

        # 2. 지도 상의 각 콘에 대해 가장 가까운 관측 콘을 찾음
        for map_idx, map_cone in enumerate(self.cones):
            if map_idx in matched_map_indices:
                continue

            # 같은 색상의 콘만 대상으로 함
            possible_matches_mask = (new_cones_observations[:, 3] == map_cone['color_id'])
            if not np.any(possible_matches_mask):
                continue

            distances_to_map_cone = distance_matrix[map_idx, possible_matches_mask]
            obs_indices_for_color = np.where(possible_matches_mask)[0]
            
            if not distances_to_map_cone.size:
                continue

            best_match_local_idx = np.argmin(distances_to_map_cone)
            min_dist = distances_to_map_cone[best_match_local_idx]

            # 3. 거리가 임계값보다 작으면 매칭 성공
            if min_dist < self.association_threshold:
                obs_idx = obs_indices_for_color[best_match_local_idx]
                
                if obs_idx not in matched_obs_indices:
                    # 4. 매칭된 콘 정보 업데이트 (예: 칼만 필터 업데이트 또는 이동 평균)
                    self._update_cone(map_idx, new_cones_observations[obs_idx])
                    matched_obs_indices.add(obs_idx)
                    matched_map_indices.add(map_idx)

        # 5. 매칭되지 않은 관측 콘은 새로운 콘으로 지도에 추가
        for obs_idx, cone_obs in enumerate(new_cones_observations):
            if obs_idx not in matched_obs_indices:
                self._add_new_cone(cone_obs)

    def _add_new_cone(self, cone_obs):
        new_cone = {
            'id': self.next_cone_id,
            'x': cone_obs[0],
            'y': cone_obs[1],
            'z': cone_obs[2],
            'color_id': int(cone_obs[3]),
            'covariance': np.eye(2) * 0.5  # 초기 불확실성은 높게 설정
        }
        self.cones.append(new_cone)
        self.next_cone_id += 1

    def _update_cone(self, map_idx, cone_obs):
        # 간단한 이동 평균으로 위치 업데이트
        alpha = 0.5 
        self.cones[map_idx]['x'] = alpha * self.cones[map_idx]['x'] + (1 - alpha) * cone_obs[0]
        self.cones[map_idx]['y'] = alpha * self.cones[map_idx]['y'] + (1 - alpha) * cone_obs[1]
        # TODO: 칼만 필터 식으로 Covariance 업데이트

    def get_cones(self):
        return self.cones

class PathPlanner:
    def __init__(self):
        self.max_edge_length = 7.0 # A reasonable track width, meters

    def plan_path(self, cones, current_car_pos):
        """
        지도 상의 콘들을 기반으로 Delaunay Triangulation을 이용해 주행 경로를 생성합니다.
        cones: TrackMap의 self.cones 리스트
        current_car_pos: 차량의 현재 위치 [x, y]
        Returns:
            (path, tri, all_points) or (None, None, None)
        """
        # 1. Get blue (1) and yellow (2) cones
        blue_cones = [c for c in cones if c['color_id'] == 1]
        yellow_cones = [c for c in cones if c['color_id'] == 2]

        if len(blue_cones) < 2 or len(yellow_cones) < 2:
            return None, None, None # Not enough cones to define a path

        # 2. Prepare points for triangulation
        all_points = np.array([[c['x'], c['y']] for c in blue_cones] + [[c['x'], c['y']] for c in yellow_cones])
        if len(all_points) < 3:
            return None, None, None # Triangulation requires at least 3 points

        # Create a mapping from point index back to cone color
        # 1 for blue, 2 for yellow
        num_blue = len(blue_cones)
        colors = np.array([1] * num_blue + [2] * len(yellow_cones))

        # 3. Perform Delaunay Triangulation
        try:
            tri = Delaunay(all_points)
        except Exception as e:
            rospy.logwarn(f"Delaunay triangulation failed: {e}")
            return None, None, None

        # 4. Find centerline edges (connecting blue and yellow cones)
        midpoints = []
        for simplex in tri.simplices:
            # A simplex is a triangle, defined by indices of 3 points
            for i in range(3):
                p1_idx = simplex[i]
                p2_idx = simplex[(i + 1) % 3]
                
                color1 = colors[p1_idx]
                color2 = colors[p2_idx]

                # Check if the edge connects a blue and a yellow cone
                if color1 != color2:
                    p1 = all_points[p1_idx]
                    p2 = all_points[p2_idx]
                    
                    # Filter out unrealistically long edges
                    edge_length = np.linalg.norm(p1 - p2)
                    if edge_length < self.max_edge_length:
                        midpoint = (p1 + p2) / 2.0
                        midpoints.append(midpoint)
        
        if len(midpoints) < 3: # Not enough midpoints to create a spline
            return None, tri, all_points

        # 5. Sort midpoints by distance from the car and smooth with a spline
        midpoints = np.array(sorted(midpoints, key=lambda p: np.hypot(p[0]-current_car_pos[0], p[1]-current_car_pos[1])))
        
        # Remove duplicate midpoints that might arise from shared edges
        unique_midpoints, indices = np.unique(midpoints, axis=0, return_index=True)
        midpoints = unique_midpoints[np.argsort(indices)]

        if len(midpoints) < 3:
            return None, tri, all_points

        k = min(2, len(midpoints)-1)
        if k < 1: return None, tri, all_points

        tck, u = splprep([midpoints[:, 0], midpoints[:, 1]], s=0.5, k=k) # s: smoothing factor
        
        u_new = np.linspace(u.min(), u.max(), 50) # Create 50 points for the path
        x_new, y_new = splev(u_new, tck)

        path = np.vstack((x_new, y_new)).T
        return path, tri, all_points

# ==================== Utility Classes ====================

class GPSIMUProcessor:
    def __init__(self):
        self.origin_set = rospy.get_param("/localization/localization/use_user_defined_ref_wgs84_position",False)
        if self.origin_set:
            self.origin_lat = rospy.get_param("/localization/localization/ref_wgs84_latitude", 0.0)
            self.origin_lon = rospy.get_param("/localization/localization/ref_wgs84_longitude", 0.0)
            self.origin_alt = rospy.get_param("/localization/localization/ref_wgs84_altitude", 0.0)
        else: self.origin_lat, self.origin_lon, self.origin_alt = 0,0,0
        self.alpha = rospy.get_param("/localization/localization/alpha_velocity", 0.0)
        self.R = 6378137.0  # WGS84 타원체의 반경 (미터 단위)
        self.prev_time = 0.0
        self.prev_gps_time = 0.0
        self.prev_x, self.prev_y, self.prev_z = 0, 0, 0
        self.yaw_filter_alpha = rospy.get_param("/localization/localization/yaw_filter_alpha", 0.05)
        self.yaw_initialized = False
        # Initialize state vector [x, y, yaw, vx, vy, yawrate, ax, ay]
        self.state = [0,0,0,0,0,0,0,0]

    ## Set origin GPS coordinates (relative to this point)
    def set_origin(self, lat: float, lon: float, alt: float):
        self.origin_lat = lat
        self.origin_lon = lon
        self.origin_alt = alt

    def gps_to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        if not self.origin_set:
            raise ValueError("Origin GPS coordinates not set.")
        
        # print(self.origin_lat, self.origin_lon, self.origin_alt)
        # print(lat,lon,alt)
        
        d_lat = math.radians(lat - self.origin_lat)
        d_lon = math.radians(lon - self.origin_lon)
        
        x = d_lon * self.R * math.cos(math.radians(self.origin_lat))
        y = d_lat * self.R
        
        return np.array([x, y])
    def updateIMU(self, imu_input, yaw_from_imu, current_time):
        # On the first run, initialize the yaw directly to avoid starting from 0
        if not self.yaw_initialized:
            self.state[2] = yaw_from_imu
            self.yaw_initialized = True
            self.prev_time = current_time
            return

        # Set current inputs (ax, ay, yawrate)
        self.state[6] = imu_input[0] # ax
        self.state[7] = imu_input[1] # ay
        self.state[5] = imu_input[2] # yawrate

        dt = current_time - self.prev_time
        if dt > 0.0:
            # Predict state (including yaw from gyro)
            predicted_state = self.predictState(self.state, dt)
            
            # The predicted_state[2] is now (old_yaw + yawrate * dt)
            # This is the high-frequency estimate from the gyro.
            
            # Correct the predicted yaw with the low-frequency measurement from the IMU's absolute orientation
            # This is the complementary filter step.
            fused_yaw = (1 - self.yaw_filter_alpha) * predicted_state[2] + self.yaw_filter_alpha * yaw_from_imu
            
            # To handle angle wrapping, a more robust solution would handle the -pi to pi jump.
            # For now, this simple fusion will greatly improve stability.
            
            predicted_state[2] = fused_yaw
            self.state = predicted_state

        self.prev_time = current_time

    def updateGPS(self, gps_msg, current_time):
        dt = current_time - self.prev_time
        self.state[0], self.state[1] = gps_msg[0], gps_msg[1]
        if dt > 0.0 and current_time > np.finfo(float).eps:
            self.state = self.predictState(self.state, dt)
            self.prev_time = current_time
        dt_gps = current_time - self.prev_gps_time
        if dt_gps > 0.0 and current_time > np.finfo(float).eps:
            dx,dy = self.state[0] - self.prev_x , self.state[1] - self.prev_y
            self.prev_x, self.prev_y = self.state[0], self.state[1]
            vx,vy = dx/dt_gps, dy/dt_gps
            self.state[3], self.state[4] = self.alpha * self.state[3] + (1-self.alpha) * (vx * math.cos(-self.state[2]) - vy * math.sin(-self.state[2])), self.alpha * self.state[4] + (1-self.alpha) * (vx * math.sin(-self.state[2])+ vy * math.cos(-self.state[2]))
        self.prev_gps_time = current_time
    
    def Quat_to_Euler(self,quaternion):
        """
        Converts a quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = quaternion[0], quaternion[1], quaternion[2], quaternion[3]

        # roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        # yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw
    
    
    def predictState(self, state, dt):
        x, y, yaw, vx, vy, yawrate, ax, ay = state

        # Use a mid-point integration for better accuracy
        yaw_middle = yaw + (yawrate * dt / 2.0)
        
        # Calculate displacement in the vehicle's body frame
        disp_body_x = vx * dt + 0.5 * ax * dt * dt
        disp_body_y = vy * dt + 0.5 * ay * dt * dt

        # Rotate the body-frame displacement to the map frame
        cos_yaw_mid = math.cos(yaw_middle)
        sin_yaw_mid = math.sin(yaw_middle)
        
        disp_map_x = disp_body_x * cos_yaw_mid - disp_body_y * sin_yaw_mid
        disp_map_y = disp_body_x * sin_yaw_mid + disp_body_y * cos_yaw_mid

        # Update state
        new_x = x + disp_map_x
        new_y = y + disp_map_y
        new_yaw = yaw + yawrate * dt
        new_vx = vx + ax * dt
        new_vy = vy + ay * dt

        # Note: yawrate, ax, ay are assumed to be constant over the interval dt
        new_state = [new_x, new_y, new_yaw, new_vx, new_vy, yawrate, ax, ay]
        return new_state


class DataLogger:
    """
    주행 데이터를 체계적으로 저장하는 클래스 (최종 추천안).
    - 메타데이터: `log.csv`에 IMU, GPS, 제어값 및 실제 LiDAR 포인트 개수 기록
    - LiDAR: `lidar.csv`에 고정된 최대 너비로 포인트 좌표 기록
    - 카메라: `cameraX.avi` 동영상 파일로 저장
    """
    def __init__(self, log_directory: str, session_name: str, video_fps: float = 10.0, max_lidar_points: int = 500):
        self.session_path = os.path.join(log_directory, session_name)
        os.makedirs(self.session_path, exist_ok=True)

        # 1. 메타데이터 CSV 설정
        self.csv_path = os.path.join(self.session_path, "log.csv")
        # 헤더에 'lidar_point_count' 필드 추가
        self.csv_header = [
            'timestamp', 'frame_id', 'autonomous_mode',
            'control_steering', 'control_throttle', 'control_brake',
            'imu_acc_x', 'imu_acc_y', 'imu_acc_z',
            'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
            'gps_latitude', 'gps_longitude',
            'yaw', 'vehicle_vx', "vehicle_vy", 'vehicle_yawrate', 'vehicle_ax', 'vehicle_ay',
            'lidar_point_count'  # <--- 추가된 필드
        ]
        self.metadata_csv_file = open(self.csv_path, 'w', newline='')
        self.metadata_csv_writer = csv.DictWriter(self.metadata_csv_file, fieldnames=self.csv_header)
        self.metadata_csv_writer.writeheader()

        # 2. 비디오 녹화 설정
        self.video_paths = {'cam1': os.path.join(self.session_path, "camera1.avi"), 'cam2': os.path.join(self.session_path, "camera2.avi")}
        self.video_writers = {'cam1': None, 'cam2': None}
        self.video_fps = video_fps
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')

        # 3. LiDAR CSV 설정 (고정 너비 방식)
        self.max_lidar_points = max_lidar_points
        self.lidar_csv_path = os.path.join(self.session_path, "lidar.csv")
        self.lidar_csv_file = open(self.lidar_csv_path, 'w', newline='')
        self.lidar_csv_writer = csv.writer(self.lidar_csv_file)
        lidar_header = ['frame_id']
        for i in range(self.max_lidar_points):
            lidar_header.extend([f'p{i}_x', f'p{i}_y'])
        self.lidar_csv_writer.writerow(lidar_header)

        # 4. Map Cones CSV 설정
        self.map_cones_csv_path = os.path.join(self.session_path, "map_cones.csv")
        self.map_cones_csv_file = open(self.map_cones_csv_path, 'w', newline='')
        self.map_cones_csv_writer = csv.writer(self.map_cones_csv_file)
        self.map_cones_csv_writer.writerow(['frame_id', 'cone_id', 'color_id', 'x', 'y', 'z'])

        self.frame_count = 0
        rospy.loginfo(f"DataLogger initialized. Saving logs to: {self.session_path}")

    def log_entry(self, autonomous_mode: str, control_command: ControlCommand,
                  imu_acc: list, imu_gyro: list, state: list,
                  camera1_image: np.ndarray, camera2_image: np.ndarray, lidar_points: np.ndarray,
                  map_cones: list):
        timestamp = rospy.Time.now().to_sec()

        # 각 프레임의 실제 LiDAR 포인트 개수 계산
        point_count = len(lidar_points) if lidar_points is not None else 0  # <--- 실제 포인트 개수 계산

        # 메타데이터 로깅 (point_count 포함)
        log_row = {
            'timestamp': timestamp, 'frame_id': self.frame_count, 'autonomous_mode': autonomous_mode,
            'control_steering': control_command.steering, 'control_throttle': control_command.throttle, 'control_brake': control_command.brake,
            'imu_acc_x': imu_acc[0], 'imu_acc_y': imu_acc[1], 'imu_acc_z': imu_acc[2],
            'imu_gyro_x': imu_gyro[0], 'imu_gyro_y': imu_gyro[1], 'imu_gyro_z': imu_gyro[2],
            'gps_latitude': state[0], 'gps_longitude': state[1],
            'yaw' : state[2], 'vehicle_vx' : state[3], 'vehicle_vy' : state[4], 'vehicle_yawrate' : state[5], 'vehicle_ax' : state[6], 'vehicle_ay' : state[7],
            'lidar_point_count': point_count  # <--- 포인트 개수 추가
        }
        self.metadata_csv_writer.writerow(log_row)

        # 카메라 데이터 로깅
        images = {'cam1': camera1_image, 'cam2': camera2_image}
        for cam_id, img in images.items():
            if img is None: continue
            if self.video_writers[cam_id] is None:
                h, w, _ = img.shape
                self.video_writers[cam_id] = cv2.VideoWriter(self.video_paths[cam_id], self.fourcc, self.video_fps, (w, h))
            self.video_writers[cam_id].write(img)

        # LiDAR 데이터 로깅 (고정 너비 + 패딩)
        lidar_row = [self.frame_count]
        if point_count > 0:
            points_flat = lidar_points[:self.max_lidar_points, :2].flatten().tolist()
            lidar_row.extend(points_flat)
        
        expected_len = 1 + self.max_lidar_points * 2
        padding_len = expected_len - len(lidar_row)
        if padding_len > 0:
            lidar_row.extend([''] * padding_len)
        self.lidar_csv_writer.writerow(lidar_row)

        # Map Cones 데이터 로깅
        if map_cones:
            for cone in map_cones:
                cone_row = [
                    self.frame_count,
                    cone['id'],
                    cone['color_id'],
                    cone['x'],
                    cone['y'],
                    cone['z']
                ]
                self.map_cones_csv_writer.writerow(cone_row)

        self.frame_count += 1

    def close(self):
        """프로그램 종료 시 호출되어 모든 파일 핸들을 안전하게 닫습니다."""
        self.metadata_csv_file.close()
        rospy.loginfo(f"Successfully saved metadata to {self.csv_path}")

        for cam_id, writer in self.video_writers.items():
            if writer is not None:
                writer.release()
                rospy.loginfo(f"Successfully saved video to {self.video_paths[cam_id]}")

        self.lidar_csv_file.close()
        rospy.loginfo(f"Successfully saved LiDAR data to {self.lidar_csv_path}")

        self.map_cones_csv_file.close()
        rospy.loginfo(f"Successfully saved map cone data to {self.map_cones_csv_path}")

StateTransitionResult = namedtuple(
    'StateTransitionResult', 
    ['success', 'from_state', 'to_state', 'reason']
)

class StateMachine:
    """
    차량의 자율주행 시스템 상태를 관리하는 상태 머신 클래스입니다.
    """
    def __init__(self):
        self.current_state = AutonomousMode.AS_OFF
        self.previous_state = AutonomousMode.AS_OFF
        self.state_entry_time = time.monotonic()
        self.last_update_time = time.monotonic()
        
        self.current_mission = ""
        self.mission_track = ""
        self.mission_active = False
        
        self.valid_transitions = {}
        self._initialize_valid_transitions()
        
        print("StateMachine: Initialized in AS_OFF state")

    def _initialize_valid_transitions(self):
        self.valid_transitions.clear()
        
        # AS_OFF -> AS_READY
        self.valid_transitions[(AutonomousMode.AS_OFF, AutonomousMode.AS_READY)] = True
        
        # AS_READY -> AS_DRIVING, AS_OFF
        self.valid_transitions[(AutonomousMode.AS_READY, AutonomousMode.AS_DRIVING)] = True
        self.valid_transitions[(AutonomousMode.AS_READY, AutonomousMode.AS_OFF)] = True
        
        # AS_DRIVING -> AS_OFF
        self.valid_transitions[(AutonomousMode.AS_DRIVING, AutonomousMode.AS_OFF)] = True

    def is_valid_transition(self, from_state: AutonomousMode, to_state: AutonomousMode) -> bool:
        return (from_state, to_state) in self.valid_transitions

    def process_event(self, event: AutonomousEvent) -> StateTransitionResult:
        target_state = self.current_state
        reason = self._event_to_string(event)
        
        if event == AutonomousEvent.SYSTEM_INIT:
            if self.current_state == AutonomousMode.AS_OFF:
                target_state = AutonomousMode.AS_READY
        elif event == AutonomousEvent.GO_SIGNAL:
            if self.current_state == AutonomousMode.AS_READY:
                target_state = AutonomousMode.AS_DRIVING
                self.mission_active = True
        else:
            return StateTransitionResult(False, self.current_state, self.current_state,
                                         f"Unknown event: {reason}")
        
        # 상태가 변경되어야 하는 경우
        if target_state != self.current_state:
            if self._perform_state_transition(target_state, reason):
                return StateTransitionResult(True, self.previous_state, self.current_state, reason)
            else:
                return StateTransitionResult(False, self.current_state, self.current_state,
                                             f"Transition failed: {reason}")
        
        # 상태 변경이 필요 없는 경우
        return StateTransitionResult(True, self.current_state, self.current_state, "No transition needed")

    def _perform_state_transition(self, new_state: AutonomousMode, reason: str) -> bool:
        if not self.is_valid_transition(self.current_state, new_state):
            print(f"StateMachine: Invalid transition from {self._state_to_string(self.current_state)} "
                  f"to {self._state_to_string(new_state)}")
            return False
        
        exit_success = self._exit_state(self.current_state)
        if not exit_success:
            print(f"StateMachine: Failed to exit state {self._state_to_string(self.current_state)}")
            return False
        
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_entry_time = time.monotonic()
        
        enter_success = self._enter_state(new_state)
        
        self._log_state_transition(self.previous_state, self.current_state, reason)
        
        return enter_success

    def _enter_state(self, state: AutonomousMode) -> bool:
        if state == AutonomousMode.AS_OFF: return self._enter_as_off()
        if state == AutonomousMode.AS_READY: return self._enter_as_ready()
        if state == AutonomousMode.AS_DRIVING: return self._enter_as_driving()
        return False
        
    def _exit_state(self, state: AutonomousMode) -> bool:
        if state == AutonomousMode.AS_OFF: return self._exit_as_off()
        if state == AutonomousMode.AS_READY: return self._exit_as_ready()
        if state == AutonomousMode.AS_DRIVING: return self._exit_as_driving()
        return True # 기본적으로 성공

    def _enter_as_off(self) -> bool:
        print("StateMachine: Entering AS_OFF state")
        self.mission_active = False
        return True

    def _enter_as_ready(self) -> bool:
        print("StateMachine: Entering AS_READY state")
        return True

    def _enter_as_driving(self) -> bool:
        print("StateMachine: Entering AS_DRIVING state")
        self.mission_active = True
        return True

    def _exit_as_off(self) -> bool: return True
    def _exit_as_ready(self) -> bool: return True
    def _exit_as_driving(self) -> bool: return True

    def inject_system_init(self):
        self.process_event(AutonomousEvent.SYSTEM_INIT)

    def inject_go_signal(self, mission: str, track: str):
        self.current_mission = mission
        self.mission_track = track
        self.process_event(AutonomousEvent.GO_SIGNAL)

    def print_state_info(self):
        print("=== State Machine Status ===")
        print(f"Current State: {self.get_current_state_string()}")
        print(f"Previous State: {self._state_to_string(self.previous_state)}")
        print(f"Time in State: {self.get_time_in_current_state():.3f} seconds")
        active_str = "Yes" if self.mission_active else "No"
        print(f"Mission: {self.current_mission} (Active: {active_str})")
        print("==========================")

    def get_time_in_current_state(self) -> float:
        return time.monotonic() - self.state_entry_time

    def get_current_state_string(self) -> str:
        return self._state_to_string(self.current_state)

    @staticmethod
    def _state_to_string(state: AutonomousMode) -> str:
        return state.name if state in AutonomousMode else "UNKNOWN"

    @staticmethod
    def _event_to_string(event: AutonomousEvent) -> str:
        return event.name if event in AutonomousEvent else "UNKNOWN_EVENT"

    def _log_state_transition(self, from_state: AutonomousMode, to_state: AutonomousMode, reason: str):
        print(f"StateMachine: {self._state_to_string(from_state)} -> "
              f"{self._state_to_string(to_state)} (Reason: {reason})")

class Control:
    def __init__(self):
        pass

    def compute_control(self, current_state, target_state):
        # 제어 알고리즘 구현
        pass