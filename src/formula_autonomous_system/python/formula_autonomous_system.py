#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file formula_autonomous_system.py
@author Jiwon Seok (jiwonseok@hanyang.ac.kr)
@brief Formula Student Driverless Autonomous System - Python Implementation
@version 0.6
@date 2025-10-05

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
        
        self.cone_map = [] # Use a list to store and update cone positions

        self.data_logger = DataLogger(
            log_directory="/home/user/fsds_ws/src/tutorial/log",
            session_name=datetime.datetime.now().strftime("%Y%m%d_%H%M%S"),
            max_lidar_points=50
        )

        self.gps_util = GPSIMUProcessor()
        self.state_machine = StateMachine()
        self.lidar_util = LiDARProcessor()
        
    def cleanup(self):
        """Gracefully shutdown the node and save final data."""
        print("Shutting down...")
        cv2.destroyAllWindows()

        # Save the final accumulated cone map
        map_path = os.path.join(self.data_logger.session_path, "final_map.csv")
        try:
            with open(map_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y'])  # Header
                if self.cone_map:
                    # Convert list of np.arrays to list of lists for writerows
                    writer.writerows([cone.tolist() for cone in self.cone_map])
            rospy.loginfo(f"Final map with {len(self.cone_map)} cones saved to {map_path}")
        except Exception as e:
            rospy.logerr(f"Error saving final map: {e}")

    def init(self):
        self.is_initialized = True
        rospy.on_shutdown(self.cleanup)
        return True

    def get_parameters(self):
        self.x_min, self.x_max = rospy.get_param("/perception/lidar_roi_extraction/x_min"), rospy.get_param("/perception/lidar_roi_extraction/x_max")
        self.y_min, self.y_max = rospy.get_param("/perception/lidar_roi_extraction/y_min"), rospy.get_param("/perception/lidar_roi_extraction/y_max")
        self.z_min, self.z_max = rospy.get_param("/perception/lidar_roi_extraction/z_min"), rospy.get_param("/perception/lidar_roi_extraction/z_max")
        self.ransac_iter = rospy.get_param("/perception/lidar_ground_removal/ransac_iterations")
        self.ransac_distance = rospy.get_param("/perception/lidar_ground_removal/ransac_distance_threshold")
        self.dbscan_eps = rospy.get_param("/perception/lidar_clustering/dbscan_eps")
        self.dbscan_points = rospy.get_param("/perception/lidar_clustering/dbscan_min_points")
        return True

    def run(self, lidar_msg, camera1_msg, camera2_msg, imu_msg, gps_msg, go_signal_msg):
        if not self.is_initialized:
            rospy.logwarn_throttle(1.0, "FormulaAutonomousSystem: Not initialized")
            return False
        self.get_parameters()

        autonomous_mode = String(data="AS_OFF")
        self.state_machine.inject_system_init()

        cv2.imshow("Camera1", self.get_camera_image(camera1_msg))
        cv2.imshow("Camera2", self.get_camera_image(camera2_msg))
        acc, gyro, orientation = self.get_imu_data(imu_msg)
        imu_data = [acc[0], acc[1], gyro[2]]
        roll, pitch, yaw = self.gps_util.Quat_to_Euler(orientation)
        lat, lon, alt = self.get_gps_data(gps_msg)
<<<<<<< HEAD
        
=======
        vtL = self.lidar_util.vehicle_to_lidar_Transform()
        # print(vtL)
>>>>>>> 2d2ee6a... [ConeDetection] 251005 @Doyeop-knut | LiDAR 회전/이동 변환행렬 생성 및 적용
        gps_data = self.gps_util.gps_to_local(lat, lon)
        self.gps_util.updateIMU(imu_data, yaw, imu_msg.header.stamp.secs)
        self.gps_util.updateGPS(gps_data, gps_msg.header.stamp.secs)
        rospy.loginfo_throttle(1.0, f"v = {math.sqrt(self.gps_util.state[3]**2 + self.gps_util.state[4]**2)} m/s")
        image1 = self.get_camera_image(camera1_msg)
        image2 = self.get_camera_image(camera2_msg)
        cv2.waitKey(1)

        points = self.get_lidar_point_cloud(lidar_msg)
        filtered = self.lidar_util.filtering_points(points, (self.x_min, self.x_max), (self.y_min, self.y_max), (self.z_min, self.z_max))
        removal = self.lidar_util.ransac_plane_removal(filtered, threshold=self.ransac_distance, max_trials=self.ransac_iter)
        cluster = self.lidar_util.cluster_points(removal, eps=self.dbscan_eps, min_samples=self.dbscan_points)
<<<<<<< HEAD
=======
        # print(cluster*math.sin(yaw))
        left, right = self.lidar_util.left_right_split(np.array(cluster))

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
        
>>>>>>> 2d2ee6a... [ConeDetection] 251005 @Doyeop-knut | LiDAR 회전/이동 변환행렬 생성 및 적용

        if go_signal_msg.mission != "None" and go_signal_msg.mission != "":
            self.state_machine.inject_go_signal(go_signal_msg.mission, go_signal_msg.track)
        autonomous_mode.data = self.state_machine.get_current_state_string()

        control_command_msg = ControlCommand()

        # Map Update with Data Association
        vehicle_x, vehicle_y, vehicle_yaw_rad = self.gps_util.state[0], self.gps_util.state[1], self.gps_util.state[2]
        if cluster.size > 0:
            cos_yaw = math.cos(vehicle_yaw_rad)
            sin_yaw = math.sin(vehicle_yaw_rad)
            rot_mat = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
            cluster_xy = cluster[:, :2]
            cluster_map_frame = (rot_mat @ cluster_xy.T).T + np.array([vehicle_x, vehicle_y])
            
            association_radius = 1.0  # meters

            for new_cone in cluster_map_frame:
                found_match = False
                if self.cone_map:
                    distances = np.sqrt(np.sum((np.array(self.cone_map) - new_cone)**2, axis=1))
                    closest_idx = np.argmin(distances)
                    if distances[closest_idx] < association_radius:
                        # A close cone already exists, so we do nothing and discard the new measurement.
                        found_match = True
                
                if not found_match:
                    self.cone_map.append(new_cone)

        # Transform accumulated map to vehicle frame for RViz visualization
        cone_map_local_frame = []
        if self.cone_map:
            map_points = np.array(self.cone_map)
            translated_points = map_points - np.array([vehicle_x, vehicle_y])
            inv_rot_mat = np.array([[math.cos(-vehicle_yaw_rad), -math.sin(-vehicle_yaw_rad)],
                                    [math.sin(-vehicle_yaw_rad),  math.cos(-vehicle_yaw_rad)]])
            local_points_2d = (inv_rot_mat @ translated_points.T).T
            cone_map_local_frame = np.hstack([local_points_2d, np.zeros((local_points_2d.shape[0], 1))])
        
        self.lidar_util.publish_point_cloud(np.array(cone_map_local_frame))

        # Logging
        self.data_logger.log_entry(
<<<<<<< HEAD
            autonomous_mode=autonomous_mode.data, control_command=control_command_msg,
            imu_acc=acc, imu_gyro=gyro, state=self.gps_util.state,
            camera1_image=image1, camera2_image=image2, lidar_points=np.array(self.cone_map)
=======
            autonomous_mode=autonomous_mode.data,
            control_command=control_command_msg,
            imu_acc=acc,
            imu_gyro=gyro,
            state= self.gps_util.state,
            camera1_image=image1,
            camera2_image=image2,
            lidar_points=cluster[:,:2]
>>>>>>> 2d2ee6a... [ConeDetection] 251005 @Doyeop-knut | LiDAR 회전/이동 변환행렬 생성 및 적용
        )

        # # --- Delaunay Triangulation and Visualization ---
        # ... (user commented out code is preserved)

        return True, control_command_msg, autonomous_mode

    def get_lidar_point_cloud(self, msg):
        pointcloud = [p[:3] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
        return np.array(pointcloud)

    def get_camera_image(self, msg):
        try:
            return CvBridge().imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"cv_bridge exception: {e}")
            return None

    def get_imu_data(self, msg):
        orientation = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        return acc, gyro, orientation
    
    def get_gps_data(self, msg):
        return msg.latitude, msg.longitude, msg.altitude

# ==================== Utility Classes ====================
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
<<<<<<< HEAD
=======

        # print(self.trans_x)
>>>>>>> 2d2ee6a... [ConeDetection] 251005 @Doyeop-knut | LiDAR 회전/이동 변환행렬 생성 및 적용
        veh_to_LiDAR = [
            [math.cos(self.rot_yaw)* math.cos(self.rot_p), math.cos(self.rot_yaw)*math.sin(self.rot_p)*math.sin(self.rot_r) - math.sin(self.rot_yaw)*math.cos(self.rot_r), math.cos(self.rot_yaw)*math.sin(self.rot_p)*math.cos(self.rot_r)+math.sin(self.rot_yaw)*math.sin(self.rot_r), self.trans_x],
            [math.sin(self.rot_yaw)* math.cos(self.rot_p), math.sin(self.rot_yaw)*math.sin(self.rot_p)*math.sin(self.rot_r) + math.cos(self.rot_yaw)*math.cos(self.rot_r), math.sin(self.rot_yaw)*math.sin(self.rot_p)*math.cos(self.rot_r)- math.cos(self.rot_yaw)*math.sin(self.rot_r), self.trans_y],
            [-math.sin(self.rot_p) , math.cos(self.rot_p)* math.sin(self.rot_r), math.cos(self.rot_p)*math.cos(self.rot_r), self.trans_z],
            [0,0,0,1]
        ]
        return veh_to_LiDAR

    def filtering_points(self, points: np.ndarray, x_range: Tuple[float, float], y_range: Tuple[float, float], z_range: Tuple[float, float]) -> np.ndarray:
        mask = (
            (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
            (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
            (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
        )
        return points[mask]
    
    def ransac_plane_removal(self, points: np.ndarray, threshold: float = 0.05, max_trials: int = 100) -> np.ndarray:
        if points is None or len(points) < 10:
            return np.array([])
        X = points[:, 0:2]
        y = points[:, 2]
        ransac = RANSACRegressor(residual_threshold=threshold, random_state=0)
        ransac.fit(X, y)
        return points[np.logical_not(ransac.inlier_mask_)]

    def cluster_points(self, points: np.ndarray, eps: float = 0.5, min_samples: int = 5) -> np.ndarray:
        if points is None or len(points) == 0:
            return np.array([])
        
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = db.labels_
        
        clusters = []
<<<<<<< HEAD
        for label in set(labels):
            if label == -1: continue
            cluster_points = points[labels == label]
            center = np.mean(cluster_points, axis=0)
            center_4d = np.array([center[0], center[1], center[2], 1])
            mat = self.vehicle_to_lidar_Transform()
            transform_lidar = np.dot(mat, center_4d)
            clusters.append(transform_lidar[:3])
=======
        for label in unique_labels:
            if label == -1:
                continue
            cluster = points[labels == label]
            center = np.mean(cluster, axis=0)
            center_4d = np.array([center[0],center[1],center[2],1])
            mat=self.vehicle_to_lidar_Transform()
            transform_lidar = np.dot(mat, center_4d)
            # print(f"transformed = {transform_lidar}")
            clusters.append(transform_lidar[:3])  # Append transformed x, y, z
>>>>>>> 2d2ee6a... [ConeDetection] 251005 @Doyeop-knut | LiDAR 회전/이동 변환행렬 생성 및 적용
        
        return np.array(clusters)
    
    def publish_point_cloud(self, points: np.ndarray):
        header = rospy.Header(stamp=rospy.Time.now(), frame_id="fsds/FSCar")
        fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1)]
        point_cloud_msg = pc2.create_cloud(header, fields, points)
        self.lidar_publisher.publish(point_cloud_msg)

        marker_array = MarkerArray()
        for i, point in enumerate(points):
            marker = Marker(header=header, ns="cluster_indices", id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = point[0], point[1], point[2] + 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.5
            marker.color.a, marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 1.0, 1.0
            marker.text = str(i)
            marker_array.markers.append(marker)

        # Clear old markers that are no longer in the updated full map
        for i in range(len(points), self.last_marker_count):
            marker = Marker(header=header, ns="cluster_indices", id=i, action=Marker.DELETE)
            marker_array.markers.append(marker)

        self.last_marker_count = len(points)
        if marker_array.markers:
            self.marker_publisher.publish(marker_array)

class GPSIMUProcessor:
    def __init__(self):
        self.origin_set = rospy.get_param("/localization/localization/use_user_defined_ref_wgs84_position",False)
        if self.origin_set:
            self.origin_lat = rospy.get_param("/localization/localization/ref_wgs84_latitude", 0.0)
            self.origin_lon = rospy.get_param("/localization/localization/ref_wgs84_longitude", 0.0)
            self.origin_alt = rospy.get_param("/localization/localization/ref_wgs84_altitude", 0.0)
        else: self.origin_lat, self.origin_lon, self.origin_alt = 0,0,0
        self.alpha = rospy.get_param("/localization/localization/alpha_velocity", 0.0)
        self.R = 6378137.0
        self.prev_time = 0.0
        self.prev_gps_time = 0.0
        self.prev_x, self.prev_y = 0, 0
        self.state = [0,0,0,0,0,0,0,0]

    def gps_to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        if not self.origin_set: raise ValueError("Origin GPS not set.")
        d_lat = math.radians(lat - self.origin_lat)
        d_lon = math.radians(lon - self.origin_lon)
        x = d_lon * self.R * math.cos(math.radians(self.origin_lat))
        y = d_lat * self.R
        return np.array([x, y])

    def updateIMU(self, imu_input, yaw, current_time):
        self.state[5], self.state[6], self.state[7] = imu_input[0], imu_input[1], imu_input[2]
        self.state[2] = yaw
        dt = current_time - self.prev_time
        if dt > 0.0:
            self.state = self.predictState(self.state, dt)
        self.prev_time = current_time

    def updateGPS(self, gps_msg, current_time):
        dt = current_time - self.prev_time
        self.state[0], self.state[1] = gps_msg[0], gps_msg[1]
        if dt > 0.0:
            self.state = self.predictState(self.state, dt)
            self.prev_time = current_time
        
        dt_gps = current_time - self.prev_gps_time
        if dt_gps > 0.0:
            dx, dy = self.state[0] - self.prev_x, self.state[1] - self.prev_y
            self.prev_x, self.prev_y = self.state[0], self.state[1]
            vx, vy = dx / dt_gps, dy / dt_gps
            self.state[3] = self.alpha * self.state[3] + (1-self.alpha) * (vx * math.cos(-self.state[2]) - vy * math.sin(-self.state[2]))
            self.state[4] = self.alpha * self.state[4] + (1-self.alpha) * (vx * math.sin(-self.state[2]) + vy * math.cos(-self.state[2]))
        self.prev_gps_time = current_time
    
    def Quat_to_Euler(self,quaternion):
        yaw = math.atan2(2*(quaternion[3]*quaternion[0]+quaternion[1]*quaternion[2]),(1-2*(quaternion[0]**2+quaternion[1]**2)))
        pitch = -math.pi/2 + 2 * math.atan2(math.sqrt(1+2*(quaternion[3]*quaternion[1]-quaternion[0]*quaternion[2])),math.sqrt(1-2*(quaternion[3]*quaternion[1]-quaternion[0]*quaternion[2])))
        roll = math.atan2(2*(quaternion[3]*quaternion[2]+quaternion[0]*quaternion[1]),(1-2*(quaternion[1]**2+quaternion[2]**2)))
        return roll * 180/math.pi, pitch*180/math.pi, yaw
    
    def predictState(self, state, dt):
        x, y, yaw, vx, vy, yawrate, ax, ay = state
        yaw_middle = yaw + (yawrate * dt / 2)
        new_x = x + vx * math.cos(yaw_middle) * dt + 0.5 * ax * math.cos(yaw_middle) * dt**2
        new_y = y + vx * math.sin(yaw_middle) * dt + 0.5 * ax * math.sin(yaw_middle) * dt**2
        new_yaw = yaw + yawrate * dt
        new_vx = vx + ax * dt
        new_vy = vy + ay * dt
        return [new_x, new_y, new_yaw, new_vx, new_vy, yawrate, ax, ay]

class DataLogger:
    def __init__(self, log_directory: str, session_name: str, video_fps: float = 10.0, max_lidar_points: int = 500):
        self.session_path = os.path.join(log_directory, session_name)
        os.makedirs(self.session_path, exist_ok=True)

        self.csv_path = os.path.join(self.session_path, "log.csv")
        self.csv_header = [
            'timestamp', 'frame_id', 'autonomous_mode', 'control_steering', 'control_throttle', 'control_brake',
            'imu_acc_x', 'imu_acc_y', 'imu_acc_z', 'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
            'gps_latitude', 'gps_longitude', 'yaw', 'vehicle_vx', "vehicle_vy", 'vehicle_yawrate', 'vehicle_ax', 'vehicle_ay',
            'lidar_point_count'
        ]
        self.metadata_csv_file = open(self.csv_path, 'w', newline='')
        self.metadata_csv_writer = csv.DictWriter(self.metadata_csv_file, fieldnames=self.csv_header)
        self.metadata_csv_writer.writeheader()

        self.video_paths = {'cam1': os.path.join(self.session_path, "camera1.avi"), 'cam2': os.path.join(self.session_path, "camera2.avi")}
        self.video_writers = {'cam1': None, 'cam2': None}
        self.video_fps = video_fps
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')

        self.max_lidar_points = max_lidar_points
        self.lidar_csv_path = os.path.join(self.session_path, "lidar.csv")
        self.lidar_csv_file = open(self.lidar_csv_path, 'w', newline='')
        self.lidar_csv_writer = csv.writer(self.lidar_csv_file)
        lidar_header = ['frame_id'] + [f'p{i}_{axis}' for i in range(self.max_lidar_points) for axis in ['x', 'y']]
        self.lidar_csv_writer.writerow(lidar_header)

        self.frame_count = 0
        rospy.loginfo(f"DataLogger initialized. Saving logs to: {self.session_path}")

    def log_entry(self, autonomous_mode: str, control_command: ControlCommand, imu_acc: list, imu_gyro: list, state: list, camera1_image: np.ndarray, camera2_image: np.ndarray, lidar_points: np.ndarray):
        timestamp = rospy.Time.now().to_sec()
        point_count = len(lidar_points) if lidar_points is not None else 0
        
        log_row = {
            'timestamp': timestamp, 'frame_id': self.frame_count, 'autonomous_mode': autonomous_mode,
            'control_steering': control_command.steering, 'control_throttle': control_command.throttle, 'control_brake': control_command.brake,
            'imu_acc_x': imu_acc[0], 'imu_acc_y': imu_acc[1], 'imu_acc_z': imu_acc[2],
            'imu_gyro_x': imu_gyro[0], 'imu_gyro_y': imu_gyro[1], 'imu_gyro_z': imu_gyro[2],
            'gps_latitude': state[0], 'gps_longitude': state[1], 'yaw' : state[2], 'vehicle_vx' : state[3], 'vehicle_vy' : state[4], 'vehicle_yawrate' : state[5], 'vehicle_ax' : state[6], 'vehicle_ay' : state[7],
            'lidar_point_count': point_count
        }
        self.metadata_csv_writer.writerow(log_row)

        for cam_id, img in {'cam1': camera1_image, 'cam2': camera2_image}.items():
            if img is None: continue
            if self.video_writers[cam_id] is None:
                h, w, _ = img.shape
                self.video_writers[cam_id] = cv2.VideoWriter(self.video_paths[cam_id], self.fourcc, self.video_fps, (w, h))
            self.video_writers[cam_id].write(img)

        lidar_row = [self.frame_count]
        if point_count > 0:
            points_flat = lidar_points[:self.max_lidar_points, :2].flatten().tolist()
            lidar_row.extend(points_flat)
        padding_len = (1 + self.max_lidar_points * 2) - len(lidar_row)
        if padding_len > 0:
            lidar_row.extend([''] * padding_len)
        self.lidar_csv_writer.writerow(lidar_row)

        self.frame_count += 1

    def close(self):
        self.metadata_csv_file.close()
        rospy.loginfo(f"Successfully saved metadata to {self.csv_path}")
        for cam_id, writer in self.video_writers.items():
            if writer: writer.release()
        self.lidar_csv_file.close()
        rospy.loginfo(f"Successfully saved logs to {self.session_path}")

class StateMachine:
    def __init__(self):
        self.current_state = AutonomousMode.AS_OFF
        self.state_entry_time = time.monotonic()
        self.current_mission, self.mission_track = "", ""
        self.mission_active = False
        self.valid_transitions = {
            (AutonomousMode.AS_OFF, AutonomousMode.AS_READY): True,
            (AutonomousMode.AS_READY, AutonomousMode.AS_DRIVING): True,
            (AutonomousMode.AS_READY, AutonomousMode.AS_OFF): True,
            (AutonomousMode.AS_DRIVING, AutonomousMode.AS_OFF): True,
        }
        rospy.loginfo("StateMachine: Initialized in AS_OFF state")

    def process_event(self, event: AutonomousEvent):
        if event == AutonomousEvent.SYSTEM_INIT and self.current_state == AutonomousMode.AS_OFF:
            self._perform_state_transition(AutonomousMode.AS_READY, "System Initialized")
        elif event == AutonomousEvent.GO_SIGNAL and self.current_state == AutonomousMode.AS_READY:
            self._perform_state_transition(AutonomousMode.AS_DRIVING, "Go Signal Received")

    def _perform_state_transition(self, new_state: AutonomousMode, reason: str):
        if self.valid_transitions.get((self.current_state, new_state)):
            rospy.loginfo(f"StateMachine: {self.current_state.name} -> {new_state.name} (Reason: {reason})")
            self.current_state = new_state
            self.state_entry_time = time.monotonic()
            if new_state == AutonomousMode.AS_DRIVING: self.mission_active = True
            elif new_state == AutonomousMode.AS_OFF: self.mission_active = False
        else:
            rospy.logwarn(f"StateMachine: Invalid transition from {self.current_state.name} to {new_state.name}")

    def inject_system_init(self): self.process_event(AutonomousEvent.SYSTEM_INIT)
    def inject_go_signal(self, mission: str, track: str):
        self.current_mission, self.mission_track = mission, track
        self.process_event(AutonomousEvent.GO_SIGNAL)
    def get_current_state_string(self) -> str: return self.current_state.name

class Control:
    def __init__(self): pass
    def compute_control(self, current_state, target_state): pass
