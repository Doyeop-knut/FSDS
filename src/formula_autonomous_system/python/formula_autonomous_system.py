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
import threading

# ROS
from std_msgs.msg import String
from fs_msgs.msg import ControlCommand
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, Image, Imu, NavSatFix, PointField
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped

# 3D LiDAR
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
import matplotlib.pyplot as plt

# Tracking
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist

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
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
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

        # State machine: Autonomous mode
        self.gps_util = GPSIMUProcessor()
        self.state_machine = StateMachine()
        self.lidar_util = LiDARProcessor()
        self.camera_util = CameraProcessor()
        self.map_cones_publisher = rospy.Publisher("/map_cones", MarkerArray, queue_size=10)
        self.tracker = ConeTracker()

        rospy.on_shutdown(self.shutdown_hook)
        
    def shutdown_hook(self):
        """Handles node shutdown procedures."""
        rospy.loginfo("Shutdown hook called. Saving final data...")
        final_map = self.tracker.get_all_tracks_for_saving()
        self.data_logger.close(cone_map=final_map)
        cv2.destroyAllWindows()
        
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
        vtL = self.lidar_util.vehicle_to_lidar_Transform()
        # print(vtL)
        gps_data = self.gps_util.gps_to_local(lat, lon)
        self.gps_util.updateIMU(imu_data, yaw, imu_msg.header.stamp.secs)
        self.gps_util.updateGPS(gps_data,gps_msg.header.stamp.secs)
        rospy.loginfo_throttle(1.0,f"v = {math.sqrt(self.gps_util.state[3]**2 + self.gps_util.state[4]**2)} m/s")

        # ==================== TF Publisher ====================
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "fsds/FSCar"
        t.transform.translation.x = self.gps_util.state[0]
        t.transform.translation.y = self.gps_util.state[1]
        t.transform.translation.z = 0.0
        
        yaw = self.gps_util.state[2]
        half_yaw = yaw / 2.0
        q_z = math.sin(half_yaw)
        q_w = math.cos(half_yaw)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        
        self.tf_broadcaster.sendTransform(t)
        # =====================================================

        ## LiDAR Processed
        points=self.get_lidar_point_cloud(lidar_msg)
        filtered = self.lidar_util.filtering_points(points, (self.x_min, self.x_max), (self.y_min, self.y_max), (self.z_min, self.z_max))
        removal =  self.lidar_util.ransac_plane_removal(filtered, threshold=self.ransac_distance, max_trials=self.ransac_iter)
        cluster = self.lidar_util.cluster_points(removal, eps=self.dbscan_eps, min_samples=self.dbscan_points)

        # ==================== Map Building & Tracking =====================
        if cluster.size > 0:
            # Transform cluster points to map frame
            veh_x = self.gps_util.state[0]
            veh_y = self.gps_util.state[1]
            veh_yaw = self.gps_util.state[2]  # Assumes radians

            cos_yaw = math.cos(veh_yaw)
            sin_yaw = math.sin(veh_yaw)
            rot_matrix = np.array([[cos_yaw, -sin_yaw],
                                   [sin_yaw,  cos_yaw]])
            map_frame_points = np.dot(cluster[:, :2], rot_matrix.T) + np.array([veh_x, veh_y])
            global_clusters = np.hstack([map_frame_points, cluster[:, 2, np.newaxis]])

            # Update tracker
            self.tracker.update(global_clusters)

        # Publish the active tracks for visualization
        active_tracks = self.tracker.get_active_tracks()
        if active_tracks.size > 0:
            self.publish_map_cones(active_tracks)
        # =====================================================================
        
        left, right = self.lidar_util.left_right_split(np.array(cluster))
        image1 = self.get_camera_image(camera1_msg)
        image2 = self.get_camera_image(camera2_msg)
        cam_mat = self.camera_util.cam_matrix()
        cam1_transform = self.camera_util.transform_matrix(self.left_tx, self.left_ty, self.left_tz, self.left_rr, self.left_rp, self.left_ry)
        cam2_transform = self.camera_util.transform_matrix(self.right_tx, self.right_ty, self.right_tz, self.right_rr, self.right_rp, self.right_ry)

        image1 = self.camera_util.preprocessImage(image1)
        image2 = self.camera_util.preprocessImage(image2)
        cam1_pts = self.camera_util.projectToCam(cluster, cam1_transform)
        cam2_pts = self.camera_util.projectToCam(cluster,cam2_transform)
        img1 = self.camera_util.visualization(cam1_pts, image1)
        img2 = self.camera_util.visualization(cam2_pts, image2)
        cv2.imshow("image1", img1)
        cv2.imshow("image2", img2)
        cv2.waitKey(1)

        # Publish cluster centers in vehicle frame (for debugging)
        self.lidar_util.publish_point_cloud(cluster)

        ## GO_SIGNAL
        if go_signal_msg.mission != "None" and go_signal_msg.mission != "":
            self.state_machine.inject_go_signal(go_signal_msg.mission, go_signal_msg.track)
        autonomous_mode.data = self.state_machine.get_current_state_string()

        # Control
        control_command_msg = ControlCommand()

        # ==================== Data Logger (Test) ====================
        self.data_logger.log_entry(
            autonomous_mode=autonomous_mode.data,
            control_command=control_command_msg,
            imu_acc=acc,
            imu_gyro=gyro,
            state= self.gps_util.state,
            camera1_image=img1,
            camera2_image=img2,
            lidar_points=global_clusters  # Log globally transformed points
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

    def publish_map_cones(self, cones):
        marker_array = MarkerArray()

        # First, publish a DELETEALL to clear old markers
        delete_marker = Marker()
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.header.frame_id = "map"
        delete_marker.ns = "map_cones"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Also clear labels
        delete_label_marker = Marker()
        delete_label_marker.header = delete_marker.header
        delete_label_marker.ns = "map_cone_labels"
        delete_label_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_label_marker)

        # Now, add all current cones
        for cone in cones:
            cone_id = int(cone[0])
            cone_pos = cone[1:]

            # Cube Marker
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"
            marker.ns = "map_cones"
            marker.id = cone_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = cone_pos[0]
            marker.pose.position.y = cone_pos[1]
            marker.pose.position.z = cone_pos[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration()
            marker_array.markers.append(marker)

            # Text Marker for ID
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "map_cone_labels"
            text_marker.id = cone_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = cone_pos[0]
            text_marker.pose.position.y = cone_pos[1]
            text_marker.pose.position.z = cone_pos[2] + 0.5  # Offset text
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.4 # Text size
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = str(cone_id)
            text_marker.lifetime = rospy.Duration()
            marker_array.markers.append(text_marker)

        self.map_cones_publisher.publish(marker_array)

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
        Trans = np.array(transform).T
        rotation = Trans[:3,:3]
        translation = Trans[3,:3]

        for point in points:
            cone_point_in_base = np.array([point[0], point[1], point[2]]).T
            cone_point_in_cam = np.dot(rotation, cone_point_in_base) + translation

            if cone_point_in_cam[0] <= 0:
                continue

            x_cam = -cone_point_in_cam[1]
            y_cam = cone_point_in_cam[2]
            z_cam = cone_point_in_cam[0]

            x_img = x_cam / z_cam
            y_img = y_cam / z_cam

            u = self.fx * x_img + self.px
            v = self.fy * y_img + self.py
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
    
    def detectConeColor(self, cone, rgb_image):
        pass
        

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

    def vehicle_to_lidar_Transform(self):

        # print(self.trans_x)
        veh_to_LiDAR = [
            [math.cos(self.rot_yaw)* math.cos(self.rot_p), math.cos(self.rot_yaw)*math.sin(self.rot_p)*math.sin(self.rot_r) - math.sin(self.rot_yaw)*math.cos(self.rot_r), math.cos(self.rot_yaw)*math.sin(self.rot_p)*math.cos(self.rot_r)+math.sin(self.rot_yaw)*math.sin(self.rot_r), self.trans_x],
            [math.sin(self.rot_yaw)* math.cos(self.rot_p), math.sin(self.rot_yaw)*math.sin(self.rot_p)*math.sin(self.rot_r) + math.cos(self.rot_yaw)*math.cos(self.rot_r), math.sin(self.rot_yaw)*math.sin(self.rot_p)*math.cos(self.rot_r)- math.cos(self.rot_yaw)*math.sin(self.rot_r), self.trans_y],
            [-math.sin(self.rot_p) , math.cos(self.rot_p)* math.sin(self.rot_r), math.cos(self.rot_p)*math.cos(self.rot_r), self.trans_z],
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
        
        clusters = []
        for label in unique_labels:
            if label == -1:
                continue
            cluster = points[labels == label]
            center = np.mean(cluster, axis=0)
            center_4d = np.array([center[0],center[1],center[2],1])
            mat=self.vehicle_to_lidar_Transform()
            transform_lidar = np.dot(mat, center_4d)
            # print(f"transformed = {transform_lidar}")
            if math.sqrt(transform_lidar[0]**2 + transform_lidar[1]**2) < 5.0:
                clusters.append(transform_lidar[:3])  # Append transformed x, y, z
        
        return np.array(clusters)
    
    def left_right_split(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Split points into left and right based on y-coordinate"""
        left_points = points[points[:, 1] > 0]
        right_points = points[points[:, 1] <= 0]
        return left_points, right_points
    
    ### For Debugging: Publish Processed Point Cloud ###
    def publish_point_cloud(self, points: np.ndarray):
        """Publish processed point cloud and their indices as markers"""

    
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "fsds/FSCar"
        
        # Publish point cloud
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        point_cloud_msg = pc2.create_cloud(header, fields, points)
        self.lidar_publisher.publish(point_cloud_msg)

        # Publish markers for indices
        marker_array = MarkerArray()
        
        # Add text markers for each cluster
        for i, point in enumerate(points):
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
        for i in range(len(points), self.last_marker_count):
            marker = Marker()
            marker.header = header
            marker.ns = "cluster_indices"
            marker.id = i
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self.last_marker_count = len(points)
        if len(marker_array.markers) > 0:
            self.marker_publisher.publish(marker_array)

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
        # Initialize state vector [x, y, yaw, vx, vy, yawrate, ax, ay]
        self.state = [0,0,0,0,0,0,0,0]

    def gps_to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        # print(self.origin_set)
        if not self.origin_set:
            raise ValueError("Origin GPS coordinates not set.")
                
        d_lat = math.radians(lat - self.origin_lat)
        d_lon = math.radians(lon - self.origin_lon)
        
        x = d_lon * self.R * math.cos(math.radians(self.origin_lat))
        y = d_lat * self.R
        
        return np.array([x, y])
    def updateIMU(self, imu_input, yaw, current_time):
        self.state[5], self.state[6], self.state[7] = imu_input[0], imu_input[1], imu_input[2]
        self.state[2] = yaw

        dt = current_time - self.prev_time
        if dt> 0.0 :
            self.state = self.predictState(self.state, dt)
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
        yaw = math.atan2(2*(quaternion[3]*quaternion[0]+quaternion[1]*quaternion[2]),(1-2*(quaternion[0]**2+quaternion[1]**2)))
        pitch = -math.pi/2 + 2 * math.atan2(math.sqrt(1+2*(quaternion[3]*quaternion[1]-quaternion[0]*quaternion[2])),math.sqrt(1-2*(quaternion[3]*quaternion[1]-quaternion[0]*quaternion[2])))
        roll = math.atan2(2*(quaternion[3]*quaternion[2]+quaternion[0]*quaternion[1]),(1-2*(quaternion[1]**2+quaternion[2]**2)))
        return roll,pitch,yaw
    
    
    def predictState(self, state, dt):
        x = state[0]
        y = state[1]
        yaw = state[2]
        vx = state[3]
        vy = state[4]
        yawrate = state[5]
        ax = state[6]
        ay = state[7]

        yaw_middle = yaw + (yawrate * dt / 2)
        new_x =  x + vx * math.cos(yaw_middle) * dt + 0.5 * ax * math.cos(yaw_middle) * dt * dt
        new_y = y + vx * math.sin(yaw_middle) * dt + 0.5 * ax * math.sin(yaw_middle) * dt * dt
        new_yaw = yaw + yawrate * dt
        new_vx = vx + ax * dt
        new_vy = vy + ay * dt

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

        # Thread-safety
        self.file_lock = threading.Lock()
        self.is_closed = False

        # 1. 메타데이터 CSV 설정
        self.csv_path = os.path.join(self.session_path, "log.csv")
        self.csv_header = [
            'timestamp', 'frame_id', 'autonomous_mode',
            'control_steering', 'control_throttle', 'control_brake',
            'imu_acc_x', 'imu_acc_y', 'imu_acc_z',
            'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
            'gps_latitude', 'gps_longitude',
            'yaw', 'vehicle_vx', "vehicle_vy", 'vehicle_yawrate', 'vehicle_ax', 'vehicle_ay',
            'lidar_point_count'
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

        self.frame_count = 0
        rospy.loginfo(f"DataLogger initialized. Saving logs to: {self.session_path}")

    def log_entry(self, autonomous_mode: str, control_command: ControlCommand,
                  imu_acc: list, imu_gyro: list, state: list,
                  camera1_image: np.ndarray, camera2_image: np.ndarray, lidar_points: np.ndarray):

        with self.file_lock:
            if self.is_closed:
                return

            timestamp = rospy.Time.now().to_sec()
            point_count = len(lidar_points) if lidar_points is not None else 0

            log_row = {
                'timestamp': timestamp, 'frame_id': self.frame_count, 'autonomous_mode': autonomous_mode,
                'control_steering': control_command.steering, 'control_throttle': control_command.throttle, 'control_brake': control_command.brake,
                'imu_acc_x': imu_acc[0], 'imu_acc_y': imu_acc[1], 'imu_acc_z': imu_acc[2],
                'imu_gyro_x': imu_gyro[0], 'imu_gyro_y': imu_gyro[1], 'imu_gyro_z': imu_gyro[2],
                'gps_latitude': state[0], 'gps_longitude': state[1],
                'yaw' : state[2], 'vehicle_vx' : state[3], 'vehicle_vy' : state[4], 'vehicle_yawrate' : state[5], 'vehicle_ax' : state[6], 'vehicle_ay' : state[7],
                'lidar_point_count': point_count
            }
            self.metadata_csv_writer.writerow(log_row)

            images = {'cam1': camera1_image, 'cam2': camera2_image}
            for cam_id, img in images.items():
                if img is None: continue
                if self.video_writers[cam_id] is None:
                    h, w, _ = img.shape
                    self.video_writers[cam_id] = cv2.VideoWriter(self.video_paths[cam_id], self.fourcc, self.video_fps, (w, h))
                self.video_writers[cam_id].write(img)

            lidar_row = [self.frame_count]
            if point_count > 0:
                points_flat = lidar_points[:self.max_lidar_points, :2].flatten().tolist()
                lidar_row.extend(points_flat)
            
            expected_len = 1 + self.max_lidar_points * 2
            padding_len = expected_len - len(lidar_row)
            if padding_len > 0:
                lidar_row.extend([''] * padding_len)
            self.lidar_csv_writer.writerow(lidar_row)

            self.frame_count += 1

    def close(self, cone_map=None):
        """프로그램 종료 시 호출되어 모든 파일 핸들을 안전하게 닫고, 맵 데이터를 저장합니다."""
        with self.file_lock:
            if self.is_closed:
                return
            
            self.metadata_csv_file.close()
            self.lidar_csv_file.close()

            for cam_id, writer in self.video_writers.items():
                if writer is not None:
                    writer.release()
            
            self.is_closed = True

        rospy.loginfo(f"Successfully saved metadata to {self.csv_path}")
        rospy.loginfo(f"Successfully saved LiDAR data to {self.lidar_csv_path}")

        # Save the final cone map
        if cone_map is not None and cone_map.size > 0:
            map_csv_path = os.path.join(self.session_path, "map.csv")
            try:
                with open(map_csv_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['id', 'x', 'y', 'z'])
                    writer.writerows(cone_map)
                rospy.loginfo(f"Successfully saved cone map to {map_csv_path}")
            except IOError as e:
                rospy.logerr(f"Failed to save cone map: {e}")

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

# ==================== Kalman Tracker ====================

class KalmanConeTracker:
    """A class for a single tracked cone using a Kalman Filter."""
    def __init__(self, detection, track_id):
        # State: [x, y, z, vx, vy, vz]
        # Measurement: [x, y, z]
        self.id = track_id
        self.kf = cv2.KalmanFilter(6, 3)
        self.kf.transitionMatrix = np.array([[1, 0, 0, 1, 0, 0],
                                              [0, 1, 0, 0, 1, 0],
                                              [0, 0, 1, 0, 0, 1],
                                              [0, 0, 0, 1, 0, 0],
                                              [0, 0, 0, 0, 1, 0],
                                              [0, 0, 0, 0, 0, 1]], np.float32)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0],
                                               [0, 1, 0, 0, 0, 0],
                                               [0, 0, 1, 0, 0, 0]], np.float32)
        
        # Initial state
        self.kf.statePost = np.array([detection[0], detection[1], detection[2], 0, 0, 0], np.float32).reshape(6, 1)
        
        # Process noise covariance
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 0.1
        self.kf.processNoiseCov[3:, 3:] *= 10.0 # Higher uncertainty for velocity

        # Measurement noise covariance
        self.kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 0.5

        # Error covariance
        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 1

        self.time_since_update = 0
        self.hits = 1

    def predict(self):
        """Predict the next state."""
        return self.kf.predict()

    def update(self, detection):
        """Update the state with a new measurement."""
        self.kf.correct(np.array(detection, dtype=np.float32).reshape(3, 1))
        self.time_since_update = 0
        self.hits += 1

    @property
    def state(self):
        return self.kf.statePost.flatten()


class ConeTracker:
    """Manages multiple KalmanConeTracker objects and a persistent map."""
    def __init__(self, dist_thresh=1.5, max_age=5, min_hits_for_confirmation=2):
        self.dist_thresh = dist_thresh
        self.max_age = max_age
        self.min_hits_for_confirmation = min_hits_for_confirmation
        self.next_track_id = 0
        self.tracks = []  # Active tracks for real-time association
        self.map_landmarks = {}  # Persistent map of confirmed cones {id: state}

    def update(self, detections):
        """
        Update tracks with new detections.
        
        detections: np.array of shape (N, 3) for (x, y, z)
        """
        # 1. Predict next state for all active tracks
        if len(self.tracks) > 0:
            predicted_positions = np.array([t.predict()[:3].flatten() for t in self.tracks])
        else:
            predicted_positions = np.empty((0, 3))

        # 2. Associate detections with predictions
        if len(detections) > 0 and len(predicted_positions) > 0:
            cost_matrix = cdist(predicted_positions, detections)
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            matched_indices = []
            for r, c in zip(row_ind, col_ind):
                if cost_matrix[r, c] < self.dist_thresh:
                    matched_indices.append((r, c))
            
            matched_track_indices = [r for r, c in matched_indices]
            matched_det_indices = [c for r, c in matched_indices]
        else:
            matched_track_indices = []
            matched_det_indices = []

        # 3. Update matched tracks and populate the persistent map
        for r, c in zip(matched_track_indices, matched_det_indices):
            track = self.tracks[r]
            track.update(detections[c])
            # If track is confirmed, add/update it in the persistent map
            if track.hits >= self.min_hits_for_confirmation:
                self.map_landmarks[track.id] = track.state

        # 4. Create new tracks for unmatched detections
        unmatched_det_indices = set(range(len(detections))) - set(matched_det_indices)
        for i in unmatched_det_indices:
            new_track = KalmanConeTracker(detections[i], self.next_track_id)
            self.tracks.append(new_track)
            self.next_track_id += 1

        # 5. Manage active track lifecycle (remove stale tracks from active list)
        updated_tracks = []
        for track in self.tracks:
            if track.time_since_update <= self.max_age:
                updated_tracks.append(track)
            track.time_since_update += 1
        self.tracks = updated_tracks

    def get_active_tracks(self):
        """Return all confirmed landmarks from the map for visualization."""
        map_data = []
        for track_id, state in self.map_landmarks.items():
            map_data.append([track_id, state[0], state[1], state[2]])
        return np.array(map_data)

    def get_all_tracks_for_saving(self):
        """Return all confirmed landmarks from the map for saving."""
        # This now returns the same data as get_active_tracks
        saved_tracks = []
        for track_id, state in self.map_landmarks.items():
            saved_tracks.append([track_id, state[0], state[1], state[2]])
        return np.array(saved_tracks)