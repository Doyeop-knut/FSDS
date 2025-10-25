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
from geometry_msgs.msg import TransformStamped, Point, PoseStamped

# ROS
from std_msgs.msg import String, ColorRGBA
from fs_msgs.msg import ControlCommand
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, Image, Imu, NavSatFix, PointField
import sensor_msgs.point_cloud2 as pc2
import rospkg # rospkg import 추가
from cv_bridge import CvBridge

# 3D LiDAR
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.interpolate import splprep, splev
from scipy.spatial import Delaunay
from scipy.optimize import minimize

# Camera
import torch
import torchvision
torch.backends.cudnn.benchmark = True

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
        
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.bridge = CvBridge()

        self.enable_visualization = rospy.get_param("/system/visualization/enable_visualization", True)
        self.enable_logging = rospy.get_param("/system/logging/enable_logging", True)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('formula_autonomous_system')
        
        if self.enable_logging:
            # ==================== 데이터 로거 추가 ====================
            self.data_logger = DataLogger(
            log_directory=os.path.join(package_path,'log'),
            session_name=datetime.datetime.now().strftime("%Y%m%d_%H%M%S"),
            max_lidar_points=50,  # 필요시 이 값을 조절
            )
        # =========================================================
        # Publishers for visualization
        self.path_publisher = rospy.Publisher("/centerline_path", Path, queue_size=1)
        self.map_cone_publisher = rospy.Publisher("/map_cones", MarkerArray, queue_size=1)
        self.triangulation_publisher = rospy.Publisher("/delaunay_triangulation", Marker, queue_size=1)
        self.path_index_publisher = rospy.Publisher("/path_indices", MarkerArray, queue_size=1)
        self.midpoints_publisher = rospy.Publisher("/cone_midpoints", MarkerArray, queue_size=1)
        self.predicted_path_publisher = rospy.Publisher("/predicted_path", Path, queue_size=1)
        self.last_path_index_count = 0
        self.last_midpoints_count = 0
        self.visualization_frame_counter = 0
        self.visualization_publish_interval = 5 # Publish visualization every 5 frames
        # System Components
        self.gps_util = GPSIMUProcessor()
        self.state_machine = StateMachine()
        self.lidar_util = LiDARProcessor(self.enable_visualization)
        self.camera_util = CameraProcessor()
        self.track_map = TrackMap()
        self.midpoint_map = MidpointMap()
        self.path_planner = PathPlanner()
        self.controller = Control(self.path_planner, self.track_map) # Control 클래스에 path_planner와 track_map 인스턴스 전달

        # rospkg를 사용하여 모델 경로 동적으로 찾기
        self.model_path = os.path.join(package_path,'python', 'retina-cone.pt')
        rospy.loginfo(f"Loading model from: {self.model_path}")

        self.model = torch.load(self.model_path, map_location=torch.device('cpu'))
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        self.model.to(memory_format=torch.channels_last)
        self.model.to(self.device)
        if self.device.type == 'cuda':
            self.model.half()
            print("Model converted to FP16 (half-precision).")
        print(f"Model target device: {self.device}")
        self.model.eval()
        self.get_parameters()

        cam_mat = self.camera_util.cam_matrix()
        self.cam1_transform = self.camera_util.transform_matrix(-self.left_tx, -self.left_ty, -self.left_tz, self.left_rr, self.left_rp, self.left_ry)
        self.cam2_transform = self.camera_util.transform_matrix(-self.right_tx, -self.right_ty, -self.right_tz, self.right_rr, self.right_rp, self.right_ry)
        
        
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
        if not self.is_initialized:
            rospy.logwarn_throttle(1.0, "FormulaAutonomousSystem: Not initialized")
            return False

        autonomous_mode = String()
        autonomous_mode.data = "AS_OFF"
        self.state_machine.inject_system_init()

        acc, gyro, orientation = self.get_imu_data(imu_msg)
        imu_data = [acc[0], acc[1], gyro[2]]
        roll,pitch,yaw = self.gps_util.Quat_to_Euler(orientation)
        lat, lon, alt = self.get_gps_data(gps_msg)
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
        ## LiDAR Processed
        points=self.get_lidar_point_cloud(lidar_msg)
        filtered = self.lidar_util.filtering_points(points, (self.x_min, self.x_max), (self.y_min, self.y_max), (self.z_min, self.z_max))
        removal =  self.lidar_util.ransac_plane_removal(filtered, threshold=self.ransac_distance, max_trials=self.ransac_iter)
        cluster = self.lidar_util.cluster_points(removal, eps=self.dbscan_eps, min_samples=self.dbscan_points)
       
        image1 = self.get_camera_image(camera1_msg)
        image2 = self.get_camera_image(camera2_msg)
        image1 = self.camera_util.preprocessImage(image1)
        image2 = self.camera_util.preprocessImage(image2)
        
        # Process image1
        img1_bgr = np.ascontiguousarray(image1)  # ★ 보장
        img1_rgb = cv2.cvtColor(img1_bgr, cv2.COLOR_BGR2RGB)
        img1_tensor = torch.from_numpy(img1_rgb).permute(2,0,1).float().div_(255.0).pin_memory()

        # Process image2
        img2_bgr = np.ascontiguousarray(image2)
        img2_rgb = cv2.cvtColor(img2_bgr, cv2.COLOR_BGR2RGB)
        img2_tensor = torch.from_numpy(img2_rgb).permute(2,0,1).float().div_(255.0).pin_memory()

        # Combine into a batch for inference
        batched_input = [img1_tensor.to(self.device, non_blocking=True), img2_tensor.to(self.device, non_blocking=True)]
        
        # If the model is in FP16, convert input tensors to FP16 as well
        if self.device.type == 'cuda' and next(self.model.parameters()).is_cuda and next(self.model.parameters()).dtype == torch.float16:
            batched_input = [img.half() for img in batched_input]

        with torch.inference_mode():
                with torch.cuda.amp.autocast(enabled=(self.device.type == 'cuda')):
                    raw_predictions = self.model(batched_input) 
        # with torch.no_grad():
        #     raw_predictions = self.model(batched_input)  # Returns List[List[Dict]] for batched input

        raw_predictions1 = [raw_predictions[0]] # Extract predictions for image1
        raw_predictions2 = [raw_predictions[1]] # Extract predictions for image2

        rendered_img1, left_bbox, left_conf = self.camera_util._process_and_draw_detections(image1, raw_predictions1)
        rendered_img2, right_bbox, right_conf = self.camera_util._process_and_draw_detections(image2, raw_predictions2)
        
        if cluster.size > 0:
            try:
                dt_cam_lidar = camera1_msg.header.stamp.to_sec() - lidar_msg.header.stamp.to_sec()
                vx = vehicle_state[3] # Longitudinal velocity
                vy = vehicle_state[4] # Lateral velocity
                yaw_rate = vehicle_state[5] # Yaw rate

                # Calculate translational compensation
                compensation_x = vx * dt_cam_lidar
                compensation_y = vy * dt_cam_lidar

                compensated_cluster = cluster.copy()

                # Apply translational compensation
                compensated_cluster[:, 0] += compensation_x
                compensated_cluster[:, 1] += compensation_y

                # Apply rotational compensation (rotate points around vehicle's current position)
                if abs(yaw_rate) > 1e-6: # Only rotate if there's significant yaw rate
                    angle_compensation = yaw_rate * dt_cam_lidar
                    cos_angle = math.cos(angle_compensation)
                    sin_angle = math.sin(angle_compensation)

                    # Rotate points around the origin (vehicle's current position is assumed to be origin for this rotation)
                    rotated_x = compensated_cluster[:, 0] * cos_angle - compensated_cluster[:, 1] * sin_angle
                    rotated_y = compensated_cluster[:, 0] * sin_angle + compensated_cluster[:, 1] * cos_angle
                    compensated_cluster[:, 0] = rotated_x
                    compensated_cluster[:, 1] = rotated_y

            except Exception as e:
                rospy.logwarn_throttle(1.0, f"Could not perform time compensation: {e}")
                compensated_cluster = cluster
        else:
            compensated_cluster = cluster

        # print(color)
        cam1_pts = self.camera_util.projectToCam(compensated_cluster, self.cam1_transform)
        cam2_pts = self.camera_util.projectToCam(compensated_cluster, self.cam2_transform)

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

            # Create dictionaries to map LiDAR cluster indices to bounding box indices
            lidar_to_bbox_map1 = {i: [] for i in range(len(compensated_cluster))}
            lidar_to_bbox_map2 = {i: [] for i in range(len(compensated_cluster))}

            # Convert projected points to numpy arrays for vectorized operations
            cam1_pts_np = np.array(cam1_pts) if cam1_pts else np.empty((0, 2))
            cam2_pts_np = np.array(cam2_pts) if cam2_pts else np.empty((0, 2))

            if left_bbox is not None and len(left_bbox) > 0 and len(cam1_pts_np) > 0:
                # Expand dimensions for broadcasting: cam1_pts_np (N, 2), left_bbox (M, 4)
                # Check if point_u >= x1, point_u <= x2, point_v >= y1, point_v <= y2
                # Resulting masks will be (N, M)
                u_in_x_range = (cam1_pts_np[:, 0][:, None] >= left_bbox[:, 0]) & \
                               (cam1_pts_np[:, 0][:, None] <= left_bbox[:, 2])
                v_in_y_range = (cam1_pts_np[:, 1][:, None] >= left_bbox[:, 1]) & \
                               (cam1_pts_np[:, 1][:, None] <= left_bbox[:, 3])
                
                # Combined mask (N, M) where True means point i is in bbox j
                point_in_bbox_mask = u_in_x_range & v_in_y_range

                # Populate lidar_to_bbox_map1
                for i in range(len(compensated_cluster)):
                    # Find which bounding boxes contain the i-th projected LiDAR point
                    matching_bboxes_indices = np.where(point_in_bbox_mask[i])[0]
                    if len(matching_bboxes_indices) > 0:
                        lidar_to_bbox_map1[i].extend(matching_bboxes_indices.tolist())

            if right_bbox is not None and len(right_bbox) > 0 and len(cam2_pts_np) > 0:
                u_in_x_range = (cam2_pts_np[:, 0][:, None] >= right_bbox[:, 0]) & \
                               (cam2_pts_np[:, 0][:, None] <= right_bbox[:, 2])
                v_in_y_range = (cam2_pts_np[:, 1][:, None] >= right_bbox[:, 1]) & \
                               (cam2_pts_np[:, 1][:, None] <= right_bbox[:, 3])
                
                point_in_bbox_mask = u_in_x_range & v_in_y_range

                for i in range(len(compensated_cluster)):
                    matching_bboxes_indices = np.where(point_in_bbox_mask[i])[0]
                    if len(matching_bboxes_indices) > 0:
                        lidar_to_bbox_map2[i].extend(matching_bboxes_indices.tolist())

            # Iterate through each 3D cluster point and determine its color
            for i, cone_3d_veh_frame in enumerate(compensated_cluster):
                detected_color = "unknown"
                # Algorithm 1: Bbox-based detection
                color_from_bbox = "unknown"
                if i in lidar_to_bbox_map1 and lidar_to_bbox_map1[i]:
                    bbox_index = lidar_to_bbox_map1[i][0]
                    bbox = left_bbox[bbox_index]
                    color_from_bbox = self.camera_util.detect_color_from_bbox(image1, bbox, debug_image=rendered_img1)

                if color_from_bbox == "unknown" and i in lidar_to_bbox_map2 and lidar_to_bbox_map2[i]:
                    bbox_index = lidar_to_bbox_map2[i][0]
                    bbox = right_bbox[bbox_index]
                    color_from_bbox = self.camera_util.detect_color_from_bbox(image2, bbox, debug_image=rendered_img2)

                # Algorithm 2: Point-based detection
                color_from_point = "unknown"
                if i < len(cam1_pts):
                    color_from_point = self.camera_util.detectConeColor(cam1_pts[i], image1, debug_image=rendered_img1)
                
                if color_from_point == "unknown" and i < len(cam2_pts):
                    color_from_point = self.camera_util.detectConeColor(cam2_pts[i], image2, debug_image=rendered_img2)

                # Combine results: If either algorithm finds a color, use it.
                # Priority is given to bbox-based detection in case of conflict.
                if color_from_bbox != "unknown":
                    detected_color = color_from_bbox
                elif color_from_point != "unknown":
                    detected_color = color_from_point
                else:
                    detected_color = "unknown"
                
                # --- Color ID assignment ---
                color_id = 0  # Default to unknown
                if detected_color == "blue":
                    color_id = 1
                elif detected_color == "yellow":
                    color_id = 2
                elif detected_color == "orange":
                    color_id = 3
                
                # Transform 3D cone from vehicle frame to map frame
                cone_3d_map_frame_xy = np.dot(cone_3d_veh_frame[:2], rot_matrix.T) + np.array([veh_x, veh_y])
                cone_3d_map_frame_z = cone_3d_veh_frame[2]
                
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
        self.track_map.update(global_clusters, vehicle_state)
        # print(f"closed loop  = {self.track_map.is_loop_closed}")

        # Plan path using the map
        path, tri, tri_points, tri_colors, midpoints, is_fallback = self.path_planner.plan_path(self.track_map.get_cones(), vehicle_state)
        
        # Visualize Map and Path
        self.publish_map_cones()
        self.publish_triangulation(tri, tri_points, tri_colors)
        self.publish_midpoints(midpoints)
        if path is not None:
            self.publish_path(path)

        # =====================================================
        # Draw LiDAR points on the images that already have bounding boxes
        img1 = self.camera_util.visualization(cam1_pts, rendered_img1)
        img2 = self.camera_util.visualization(cam2_pts, rendered_img2)

        if self.enable_visualization:
            cv2.imshow("image1", img1)
            cv2.imshow("image2", img2)
            cv2.waitKey(1)
        self.lidar_util.publish_point_cloud(cluster)        

        ## GO_SIGNAL
        if go_signal_msg.mission != "None" and go_signal_msg.mission != "":
            self.state_machine.inject_go_signal(go_signal_msg.mission, go_signal_msg.track)
        autonomous_mode.data = self.state_machine.get_current_state_string()

        # filtered_points = LiDARProcessor().filtering_points(np.array([[x,y,z]]), (1.0, 20.0), (-10.0, 10.0), (-0.5, 0.5))
        # print("Filtered Points:", filtered_points)
        ## GPS velocity
        # Control
        control_command_msg = ControlCommand()
        if self.state_machine.current_state == AutonomousMode.AS_DRIVING and path is not None and len(path) > 0:
            # Call the selected controller to compute commands
            throttle, steer, brake, predicted_path = self.controller.compute_control(vehicle_state, path, is_fallback)
            control_command_msg.throttle = throttle
            control_command_msg.steering = steer
            control_command_msg.brake = brake
            self.publish_predicted_path(predicted_path)
        else:
            # If not driving or no path, apply brakes and zero throttle/steering
            control_command_msg.throttle = 0.0
            control_command_msg.steering = 0.0
            control_command_msg.brake = 0.5        # print(go_signal_ms

        if self.enable_logging:
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
        # # =========================================================
        
        # plt.axis([-50,50,-20,200])
        # if len(cluster) == 0:
        #     pass
        
        # plt.scatter(x=cluster[:,0] + offset[0] ,y=cluster[:,1]+ offset[1])
        # plt.pause(0.001)
        
        # plt.clf()
        # # print(go_signal_msg.mission, go_signal_msg.track)
        self.visualization_frame_counter += 1

        return True, control_command_msg, autonomous_mode

    def publish_map_cones(self):
        if not self.enable_visualization:
            return
        if self.visualization_frame_counter % self.visualization_publish_interval != 0:
            return

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
        if not self.enable_visualization:
            return
        if self.visualization_frame_counter % self.visualization_publish_interval != 0:
            return

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for point in path:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.1 # slightly above ground
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

        # Publish markers for indices
        marker_array = MarkerArray()
        header = path_msg.header
        
        # Add text markers for each path point
        for i, point in enumerate(path):
            marker = Marker()
            marker.header = header
            marker.ns = "path_indices"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.5  # Offset text above the path
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.5  # Text size
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.text = str(i)
            marker_array.markers.append(marker)

        # Add delete markers for old markers that are no longer present
        for i in range(len(path), self.last_path_index_count):
            marker = Marker()
            marker.header = header
            marker.ns = "path_indices"
            marker.id = i
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self.last_path_index_count = len(path)
        if len(marker_array.markers) > 0:
            self.path_index_publisher.publish(marker_array)

    def publish_predicted_path(self, predicted_path):
        if not self.enable_visualization:
            return
        # Predicted path is usually shorter than the full path, so no need for interval check

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for point in predicted_path:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.2 # Slightly above the main path
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.predicted_path_publisher.publish(path_msg)

    def publish_midpoints(self, midpoints):
        if not self.enable_visualization:
            return
        if self.visualization_frame_counter % self.visualization_publish_interval != 0:
            return

        marker_array = MarkerArray()
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        if midpoints is not None:
            for i, point in enumerate(midpoints):
                marker = Marker()
                marker.header = header
                marker.ns = "cone_midpoints"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.1 # slightly above ground
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker_array.markers.append(marker)
        
        # Add delete markers for old markers
        for i in range(len(midpoints) if midpoints is not None else 0, self.last_midpoints_count):
            marker = Marker()
            marker.header = header
            marker.ns = "cone_midpoints"
            marker.id = i
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
        
        self.last_midpoints_count = len(midpoints) if midpoints is not None else 0
        self.midpoints_publisher.publish(marker_array)

    def publish_triangulation(self, tri, points, colors):
        if not self.enable_visualization:
            # Always clear markers if visualization is disabled
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"
            marker.ns = "delaunay_mesh"
            marker.id = 0
            marker.action = Marker.DELETEALL
            self.triangulation_publisher.publish(marker)
            return

        if self.visualization_frame_counter % self.visualization_publish_interval != 0 or (tri is None or points is None or colors is None):
            # Clear previous markers if triangulation is not available or not publishing this frame
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
        marker.scale.x = 0.1  # Line width
        
        # Define colors
        blue = ColorRGBA(0.0, 0.0, 1.0, 0.8)
        yellow = ColorRGBA(1.0, 1.0, 0.0, 0.8)
        gray = ColorRGBA(0.6, 0.6, 0.6, 0.4)

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

                    color1 = colors[p1_idx]
                    color2 = colors[p2_idx]

                    line_color = gray
                    if color1 == 1 and color2 == 1: # Blue
                        line_color = blue
                    elif color1 == 2 and color2 == 2: # Yellow
                        line_color = yellow
                    
                    marker.colors.append(line_color)
                    marker.colors.append(line_color)
                
        self.triangulation_publisher.publish(marker)

    def get_lidar_point_cloud(self, msg):
        """Convert ROS PointCloud2 message to point cloud"""
        pointcloud = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[:3]
            pointcloud.append([x, y, z])
            
        return np.array(pointcloud, dtype=np.float32)

    def get_camera_image(self, msg):
        """Convert ROS Image message to OpenCV Mat"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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

        self.min_bbox_area = rospy.get_param("/perception/camera_cone_detection/min_bbox_area", 100)
        self.visualize_lidar_on_camera = rospy.get_param("/perception/camera_cone_detection/visualize_lidar_on_camera", False)

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
        # Filters are bypassed for performance. If needed, re-enable and tune parameters.
        return rgb_image
    
    def projectToCam(self, points, transform):
        if not points.size: # Handle empty points array
            return []

        T = np.array(transform)
        rotation = T[:3, :3]
        translation = T[:3, 3]

        # Apply the transformation to all points at once
        # points is (N, 3), rotation is (3, 3), translation is (3,)
        cone_points_in_cam = np.dot(points, rotation.T) + translation

        # Filter points in front of the camera (ROS X-axis points forward)
        valid_mask = cone_points_in_cam[:, 0] > 0
        cone_points_in_cam_filtered = cone_points_in_cam[valid_mask]

        if not cone_points_in_cam_filtered.size:
            return []

        # Convert from ROS camera coordinates to standard CV/image coordinates
        z_cv = cone_points_in_cam_filtered[:, 0]  # ROS X -> CV Z
        x_cv = -cone_points_in_cam_filtered[:, 1] # ROS Y -> CV X
        y_cv = -cone_points_in_cam_filtered[:, 2] # ROS Z -> CV Y

        # Perform pinhole projection
        u = self.fx * (x_cv / z_cv) + self.px
        v = self.fy * (y_cv / z_cv) + self.py
        
        # Combine u and v into a list of tuples
        projected_points = np.vstack((u, v)).T.tolist()
        return projected_points


        # print(f"base = {cone_point_in_base}, cam = {cone_point_in_cam}")
        # return (u,v)
    def visualization(self, points, rgb_image):
        viz = rgb_image.copy()
        if self.visualize_lidar_on_camera:
            image_size = rgb_image.shape
            for point in points:
                if 0 <= point[0] < image_size[1] and 0 <= point[1] < image_size[0]:
                    projected = (int(point[0]), int(point[1]))
                    cv2.circle(viz, projected, 5, (0, 255, 255), -1) # Yellow circles, filled
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

        # --- 신뢰도 기반 색상 탐지 로직으로 개선 ---
        color_scores = {"yellow": 0.0, "blue": 0.0, "orange": 0.0}
        
        # 각 색상 마스크 생성
        mask_yellow = cv2.inRange(hsv_roi, self.hsv_yellow_min, self.hsv_yellow_max)
        mask_blue = cv2.inRange(hsv_roi, self.hsv_blue_min, self.hsv_blue_max)
        mask_orange = cv2.inRange(hsv_roi, self.hsv_orange_min, self.hsv_orange_max)

        masks = {"yellow": mask_yellow, "blue": mask_blue, "orange": mask_orange}

        for color, mask in masks.items():
            pixel_count = cv2.countNonZero(mask)
            if pixel_count > 0:
                # 마스크에 해당하는 픽셀들의 평균 채도(S)와 명도(V)를 계산
                # 채도와 명도가 높을수록 색상이 뚜렷하므로 가중치를 줌
                mean_sv = cv2.mean(hsv_roi, mask=mask)
                avg_saturation = mean_sv[1] / 255.0 # 0~1 정규화
                avg_value = mean_sv[2] / 255.0      # 0~1 정규화
                
                # 신뢰도 점수 = 픽셀 수 * (평균 채도 + 평균 명도)
                # 이렇게 하면 흐릿한 색상의 넓은 영역보다 뚜렷한 색상의 작은 영역이 더 높은 점수를 받을 수 있음
                color_scores[color] = pixel_count * (avg_saturation + avg_value)
                # print(f"from LiDAR - All score = {color_scores}")

        # 가장 높은 점수를 받은 색상을 선택
        max_score = 0
        dominant_color = "unknown"
        for color, score in color_scores.items():
            if score > max_score:
                max_score = score
                dominant_color = color
        
        # 최소 픽셀 수 임계값 (노이즈 제거)
        if dominant_color != "unknown":
            # 점수 계산에 사용된 픽셀 수가 전체 ROI의 5% 미만이면 노이즈로 간주
            # print(f"LiDAR - count from mask = {cv2.countNonZero(masks[dominant_color])}, limit = {roi.size * 0.05}")
            if cv2.countNonZero(masks[dominant_color]) < (roi.size * 0.04):
                 dominant_color = "unknown"

        if debug_image is not None and dominant_color != "unknown":
             # Put text for the detected color
             cv2.putText(debug_image, dominant_color, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            #  print(f"from LiDAR - {color_scores}, color = {dominant_color} \n")

        return dominant_color
    def _process_and_draw_detections(self, image, raw_predictions, conf_threshold=0.5, iou_threshold=0.45):
        """
        torchvision RetinaNet 출력 처리 및 바운딩 박스 그리기
        raw_predictions: List[Dict[Tensor]] 형식
        각 Dict는 'boxes', 'labels', 'scores' 키를 포함
        """
        img_copy = image.copy()
    
        # torchvision detection 모델은 List[Dict]를 반환
        # 첫 번째 이미지의 예측 결과 가져오기
        if isinstance(raw_predictions, list) and len(raw_predictions) > 0:
            prediction = raw_predictions[0]  # 첫 번째 이미지의 결과
        else:
            rospy.logwarn("Invalid prediction format")
            return img_copy, np.array([]), np.array([])
    
        # 딕셔너리에서 boxes, scores, labels 추출
        boxes = prediction['boxes'].cpu().numpy()  # [N, 4] - [x1, y1, x2, y2]
        scores = prediction['scores'].cpu().numpy()  # [N]
        labels = prediction['labels'].cpu().numpy()  # [N]
    
        # Confidence threshold 적용
        confidence_mask = scores > conf_threshold
        boxes = boxes[confidence_mask]
        scores = scores[confidence_mask]
        labels = labels[confidence_mask]
    
        # 검출 결과가 없는 경우
        if len(boxes) == 0:
            return img_copy, np.array([]), np.array([])
    
        # NMS는 이미 모델 내부에서 적용되었으므로 생략 가능
        # 필요하다면 추가 NMS 적용:
        # indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), conf_threshold, iou_threshold)
    
        box_output = []
        confidence_output = []
    
        for i in range(len(boxes)):
            x1, y1, x2, y2 = map(int, boxes[i])
            width = x2 - x1
            height = y2 - y1
            area = width * height

            if area < self.min_bbox_area:
                continue

            box_output.append([x1, y1, x2, y2])
            confidence_output.append(scores[i])

            # 바운딩 박스 그리기
            color = (0, 255, 0) # Green color for bounding box
            cv2.rectangle(img_copy, (x1, y1), (x2, y2), color, 2)

            # 레이블 그리기
            label = f"Cone: {scores[i]:.2f}"
            cv2.putText(img_copy, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
        return img_copy, np.array(box_output), np.array(confidence_output)
    def is_point_in_bbox(self, point, bbox):
        """Checks if a 2D point is inside a bounding box."""
        if point is None or bbox is None:
            return False
        u, v = point
        x1, y1, x2, y2 = bbox
        return x1 <= u <= x2 and y1 <= v <= y2

    def detect_color_from_bbox(self, image, bbox, debug_image=None):
        """Detects the dominant color within a given bounding box."""
        x1, y1, x2, y2 = map(int, bbox)

        # Clamp coordinates to be within image dimensions
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(image.shape[1], x2)
        y2 = min(image.shape[0], y2)

        if x2 <= x1 or y2 <= y1:
            return "unknown"

        roi = image[y1:y2, x1:x2]

        if roi.size == 0:
            return "unknown"

        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # --- 신뢰도 기반 색상 탐지 로직으로 개선 ---
        color_scores = {"yellow": 0.0, "blue": 0.0, "orange": 0.0}
        
        # 각 색상 마스크 생성
        mask_yellow = cv2.inRange(hsv_roi, self.hsv_yellow_min, self.hsv_yellow_max)
        mask_blue = cv2.inRange(hsv_roi, self.hsv_blue_min, self.hsv_blue_max)
        mask_orange = cv2.inRange(hsv_roi, self.hsv_orange_min, self.hsv_orange_max)

        masks = {"yellow": mask_yellow, "blue": mask_blue, "orange": mask_orange}
        
        for color, mask in masks.items():
            pixel_count = cv2.countNonZero(mask)
            if pixel_count > 0:
                mean_sv = cv2.mean(hsv_roi, mask=mask)
                avg_saturation = mean_sv[1] / 255.0
                avg_value = mean_sv[2] / 255.0
                color_scores[color] = pixel_count * (avg_saturation + avg_value)
                # print(f"from bbox - All score = {color_scores}")

        # 가장 높은 점수를 받은 색상을 선택
        max_score = 0
        dominant_color = "unknown"
        for color, score in color_scores.items():
            if score > max_score:
                max_score = score
                dominant_color = color

        # 최소 픽셀 수 임계값 (노이즈 제거)
        if dominant_color != "unknown":
            # print(f"bbox - count from mask = {cv2.countNonZero(masks[dominant_color])}, limit = {roi.size * 0.05}")
            if cv2.countNonZero(masks[dominant_color]) < (roi.size * 0.05):
                 dominant_color = "unknown"

        if debug_image is not None and dominant_color != "unknown":
            cv2.putText(debug_image, dominant_color, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            # print(f"from bbox - {color_scores}, color = {dominant_color} \n")

    
        return dominant_color
        

class LiDARProcessor:
    def __init__(self, enable_visualization=True):
        self.enable_visualization = enable_visualization
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

    def filtering_points(self, points, x_range, y_range, z_range):
        if points.size == 0: return points
        n = points.shape[0]
        if n > 80000: points = points[::8]
        elif n > 40000: points = points[::6]
        elif n > 20000: points = points[::4]  # 기존 유지
        mask = (
            (points[:,0] >= x_range[0]) & (points[:,0] <= x_range[1]) &
            (points[:,1] >= y_range[0]) & (points[:,1] <= y_range[1]) &
            (points[:,2] >= z_range[0]) & (points[:,2] <= z_range[1])
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
        base_eps = self.dbscan_eps
        base_min = self.dbscan_points
        n = points.shape[0]
        scale = 0.8 if n < 2000 else (1.0 if n < 8000 else 1.2)
        eps = (eps or base_eps) * scale
        min_samples = int((min_samples or base_min) * scale)
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
        if not self.enable_visualization:
            return

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
        self.cones = []
        self.next_cone_id = 0
        self.association_threshold = rospy.get_param("/mapping/association_threshold", 1.5)

        # --- Loop Closure Parameters ---
        self.is_loop_closed = False
        self.min_cones_for_lc = rospy.get_param("/mapping/lc/min_cones", 10000) # Temporarily set to a very high value to disable loop closure
        self.lc_trigger_distance = rospy.get_param("/mapping/lc/trigger_distance", 8.0)
        self.lc_search_radius = rospy.get_param("/mapping/lc/search_radius", 15.0)
        self.lc_min_match_pairs = rospy.get_param("/mapping/lc/min_pairs", 4)
        self.lc_max_transform_error = rospy.get_param("/mapping/lc/max_error", 0.75)
        self.start_line_center = None
        self.update_count = 0
        self.lc_cooldown_period = rospy.get_param("/mapping/lc/cooldown_updates", 100) # Cooldown in number of updates

    def update(self, new_cones_observations, vehicle_state):
        """
        Updates the map with new cone observations, and attempts loop closure.
        """
        self.update_count += 1

        # 1. Find start line if not already found
        if self.start_line_center is None and len(new_cones_observations) > 1:
            self._find_start_line(new_cones_observations)

        # 2. Attempt loop closure and get corrected observations
        # This will return transformed observations if LC is successful
        processed_observations = self._detect_and_correct_loop_closure(new_cones_observations, vehicle_state)

        # 3. Continue with original update logic
        if not self.cones:
            for cone_obs in processed_observations:
                self._add_new_cone(cone_obs)
            return

        if processed_observations.size == 0:
            return

        map_cone_positions = np.array([[c['x'], c['y']] for c in self.cones])
        obs_cone_positions = processed_observations[:, :2]
        distance_matrix = cdist(map_cone_positions, obs_cone_positions)

        matched_obs_indices = set()
        matched_map_indices = set()

        # 1단계: 위치와 색상이 모두 일치하는 경우 먼저 매칭 (가장 신뢰도 높은 매칭)
        for map_idx, map_cone in enumerate(self.cones):
            if map_idx in matched_map_indices:
                continue

            possible_matches_mask = (processed_observations[:, 3] == map_cone['color_id'])
            if np.any(possible_matches_mask):
                distances_to_map_cone = distance_matrix[map_idx, possible_matches_mask]
                obs_indices_for_color = np.where(possible_matches_mask)[0]
                
                if distances_to_map_cone.size > 0:
                    best_match_local_idx = np.argmin(distances_to_map_cone)
                    min_dist = distances_to_map_cone[best_match_local_idx]

                    if min_dist < self.association_threshold:
                        obs_idx = obs_indices_for_color[best_match_local_idx]
                        if obs_idx not in matched_obs_indices:
                            self._update_cone(map_idx, processed_observations[obs_idx])
                            matched_obs_indices.add(obs_idx)
                            matched_map_indices.add(map_idx)

        # 2단계: 색상이 다르더라도 위치가 매우 가까운 경우, 색상 정보를 업데이트 (오탐지 교정)
        for map_idx, map_cone in enumerate(self.cones):
            if map_idx in matched_map_indices: continue

            # 아직 매칭되지 않은 관측값들 중에서 가장 가까운 것을 찾음
            unmatched_obs_indices = [i for i in range(len(processed_observations)) if i not in matched_obs_indices]
            if not unmatched_obs_indices: break

            distances_to_unmatched = distance_matrix[map_idx, unmatched_obs_indices]
            if distances_to_unmatched.size == 0:
                continue
            best_match_local_idx = np.argmin(distances_to_unmatched)
            min_dist = distances_to_unmatched[best_match_local_idx]

            if min_dist < self.association_threshold:
                obs_idx = unmatched_obs_indices[best_match_local_idx]
                if obs_idx not in matched_obs_indices:
                    new_color_id = int(processed_observations[obs_idx][3])
                    # 알려진 색상(0이 아님)을 알 수 없는 색상(0)으로 덮어쓰지 않도록 합니다.
                    # 기존 색상이 '알 수 없음'이거나 새로운 색상이 '알 수 없음'이 아닌 경우에만 색상을 업데이트합니다.
                    if new_color_id != 0 or map_cone['color_id'] == 0:
                        rospy.logdebug(f"Correcting color of cone {map_cone['id']} from {map_cone['color_id']} to {new_color_id}")
                        self._update_cone(map_idx, processed_observations[obs_idx], update_color=True)
                    else:
                        # 그렇지 않으면 기존 색상을 유지하고 위치만 업데이트합니다.
                        self._update_cone(map_idx, processed_observations[obs_idx], update_color=False)
                    
                    matched_obs_indices.add(obs_idx)
                    matched_map_indices.add(map_idx)

        for obs_idx, cone_obs in enumerate(processed_observations):
            if obs_idx not in matched_obs_indices:
                self._add_new_cone(cone_obs)

    def _find_start_line(self, observations):
        blue_cones = observations[observations[:, 3] == 1]
        yellow_cones = observations[observations[:, 3] == 2]
        if blue_cones.shape[0] > 0 and yellow_cones.shape[0] > 0:
            avg_blue = np.mean(blue_cones[:, :2], axis=0)
            avg_yellow = np.mean(yellow_cones[:, :2], axis=0)
            self.start_line_center = (avg_blue + avg_yellow) / 2.0
            rospy.loginfo(f"TrackMap: Start line center established at {self.start_line_center}")

    def _detect_and_correct_loop_closure(self, new_observations, vehicle_state):
        # --- 1. Check Trigger Conditions ---
        if self.is_loop_closed or len(self.cones) < self.min_cones_for_lc or self.start_line_center is None or self.update_count < self.lc_cooldown_period:
            return new_observations

        car_pos = vehicle_state[:2]
        dist_to_start = np.linalg.norm(car_pos - self.start_line_center)

        if dist_to_start > self.lc_trigger_distance:
            return new_observations

        rospy.loginfo_throttle(1.0, f"TrackMap: Loop closure check triggered (dist to start: {dist_to_start:.2f}m)")

        # --- 2. Find Candidate Cones for Matching ---
        map_cones_np = np.array([[c['x'], c['y'], c['color_id']] for c in self.cones])
        
        # Reference cones: old cones from the map near the start line
        dist_from_start = np.linalg.norm(map_cones_np[:, :2] - self.start_line_center, axis=1)
        reference_mask = dist_from_start < self.lc_search_radius
        reference_cones = map_cones_np[reference_mask]

        # Current cones: new observations near the car
        dist_from_car = np.linalg.norm(new_observations[:, :2] - car_pos, axis=1)
        current_mask = dist_from_car < self.lc_search_radius
        current_cones = new_observations[current_mask]

        if len(reference_cones) < self.lc_min_match_pairs or len(current_cones) < self.lc_min_match_pairs:
            rospy.logwarn_throttle(1.0, "TrackMap: Not enough cones for loop closure matching.")
            return new_observations

        # --- 3. Find Matching Pairs ---
        src_pts, dst_pts = [], []
        # Use distance matrix between current and reference cones
        dist_matrix = cdist(current_cones[:, :2], reference_cones[:, :2])
        
        for i, c_cone in enumerate(current_cones):
            # Find potential matches of the same color
            color_mask = reference_cones[:, 2] == c_cone[3]
            if not np.any(color_mask):
                continue
            
            row = dist_matrix[i, color_mask]
            ref_indices = np.where(color_mask)[0]

            if row.size == 0:
                continue

            best_ref_local_idx = np.argmin(row)
            if row[best_ref_local_idx] < self.association_threshold:
                src_pts.append(c_cone[:2])
                dst_pts.append(reference_cones[ref_indices[best_ref_local_idx]][:2])

        if len(src_pts) < self.lc_min_match_pairs:
            rospy.logwarn_throttle(1.0, f"TrackMap: Found only {len(src_pts)} pairs for LC, need {self.lc_min_match_pairs}.")
            return new_observations

        # --- 4. Estimate and Verify Transform ---
        src_pts_np = np.array(src_pts, dtype=np.float32)
        dst_pts_np = np.array(dst_pts, dtype=np.float32)
        
        # Using estimateAffine2D as it's more robust than the deprecated estimateRigidTransform
        transform_matrix, _ = cv2.estimateAffine2D(src_pts_np, dst_pts_np, ransacReprojThreshold=0.5)

        if transform_matrix is None:
            rospy.logwarn("TrackMap: Loop closure transform estimation failed.")
            return new_observations

        # Verify the transformation by checking the error
        src_transformed = cv2.transform(src_pts_np.reshape(-1, 1, 2), transform_matrix).reshape(-1, 2)
        avg_error = np.mean(np.linalg.norm(src_transformed - dst_pts_np, axis=1))

        if avg_error > self.lc_max_transform_error:
            rospy.logwarn(f"TrackMap: Loop closure failed. High transform error: {avg_error:.2f}m")
            return new_observations

        # --- 5. Apply Correction ---
        rospy.loginfo(f"*** Loop Closure Successful! *** Error: {avg_error:.2f}m. Correcting observations.")
        self.is_loop_closed = True
        
        # Apply the transform to ALL new observations for this timestep
        new_obs_pts = new_observations[:, :2].astype(np.float32)
        corrected_obs_pts = cv2.transform(new_obs_pts.reshape(-1, 1, 2), transform_matrix).reshape(-1, 2)
        
        corrected_observations = new_observations.copy()
        corrected_observations[:, :2] = corrected_obs_pts
        
        return corrected_observations

    def _add_new_cone(self, cone_obs):
        new_cone = {
            'id': self.next_cone_id,
            'x': cone_obs[0],
            'y': cone_obs[1],
            'z': cone_obs[2],
            'color_id': int(cone_obs[3]),
            'observations': 1, # 관측 횟수 추가
            'covariance': np.eye(2) * 0.5
        }
        self.cones.append(new_cone)
        self.next_cone_id += 1

    def _update_cone(self, map_idx, cone_obs, update_color=False):
        # --- 관측 횟수에 기반한 가중 평균으로 업데이트 로직 개선 ---
        # 관측 횟수가 많을수록 기존 맵의 신뢰도를 높게, 새로운 관측의 영향은 적게
        self.cones[map_idx]['observations'] += 1
        n = self.cones[map_idx]['observations']
        alpha = 1.0 / n  # 새로운 관측에 대한 가중치
        self.cones[map_idx]['x'] = (1 - alpha) * self.cones[map_idx]['x'] + alpha * cone_obs[0]
        self.cones[map_idx]['y'] = (1 - alpha) * self.cones[map_idx]['y'] + alpha * cone_obs[1]
        if update_color:
            self.cones[map_idx]['color_id'] = int(cone_obs[3])

    def get_cones(self):
        return self.cones

class MidpointMap:
    def __init__(self):
        self.midpoints = []
        self.next_midpoint_id = 0
        self.association_threshold = rospy.get_param("/planning/midpoint_map/association_threshold", 1.5)
        self.smoothing_alpha = rospy.get_param("/planning/midpoint_map/smoothing_alpha", 0.5)

    def update(self, new_midpoints_obs):
        if new_midpoints_obs is None or len(new_midpoints_obs) == 0:
            return

        if not self.midpoints:
            for p in new_midpoints_obs:
                self._add_new_midpoint(p)
            return

        map_points = np.array([[m['x'], m['y']] for m in self.midpoints])
        obs_points = np.array(new_midpoints_obs)
        distance_matrix = cdist(map_points, obs_points)

        matched_obs_indices = set()
        for map_idx, map_point in enumerate(self.midpoints):
            if distance_matrix.shape[1] == 0:
                break

            best_match_obs_idx = np.argmin(distance_matrix[map_idx])
            min_dist = distance_matrix[map_idx, best_match_obs_idx]

            if min_dist < self.association_threshold:
                if best_match_obs_idx not in matched_obs_indices:
                    self._update_midpoint(map_idx, obs_points[best_match_obs_idx])
                    matched_obs_indices.add(best_match_obs_idx)
        
        for obs_idx, obs_point in enumerate(obs_points):
            if obs_idx not in matched_obs_indices:
                self._add_new_midpoint(obs_point)

    def _add_new_midpoint(self, point):
        new_midpoint = {
            'id': self.next_midpoint_id,
            'x': point[0],
            'y': point[1],
        }
        self.midpoints.append(new_midpoint)
        self.next_midpoint_id += 1

    def _update_midpoint(self, map_idx, obs_point):
        self.midpoints[map_idx]['x'] = (1 - self.smoothing_alpha) * self.midpoints[map_idx]['x'] + self.smoothing_alpha * obs_point[0]
        self.midpoints[map_idx]['y'] = (1 - self.smoothing_alpha) * self.midpoints[map_idx]['y'] + self.smoothing_alpha * obs_point[1]

    def get_all_midpoints(self):
        # Sort by id to maintain path order
        sorted_midpoints = sorted(self.midpoints, key=lambda p: p['id'])
        return [[p['x'], p['y']] for p in sorted_midpoints]

class PathPlanner:
    def __init__(self):
        self.max_edge_length = rospy.get_param("/planning/path_planner/max_edge_length", 7.0)
        self.spline_smoothing_factor = rospy.get_param("/planning/path_planner/spline_smoothing_factor", 0.5)
        self.w_dist = rospy.get_param("/planning/path_planner/weight_dist", 0.3)
        self.w_angle = rospy.get_param("/planning/path_planner/weight_angle", 0.7)
        self.max_path_distance = rospy.get_param("/planning/path_planner/max_path_distance", 200.0)
        self.spline_smoothing_factor = rospy.get_param("/planning/path_planner/spline_smoothing_factor", 0.5)
        self.path_obstacle_threshold = rospy.get_param("/planning/path_planner/path_obstacle_threshold", 1.5)
        self.lane_offset = rospy.get_param("/local_planning/trajectory/lane_offset", 2.0)
        
        # --- 경로 계획기 관심 영역(ROI) 필터링 파라미터 ---
        self.planner_roi_distance = rospy.get_param("/local_planning/trajectory/planner_roi_distance", 30.0)
        self.planner_roi_angle_rad = math.radians(rospy.get_param("/local_planning/trajectory/planner_roi_angle", 90.0))
        self.planner_clustering_eps = rospy.get_param("/local_planning/trajectory/planner_clustering_eps", 8.0)
        
        # --- 경로 안정화를 위한 이동 평균 필터 추가 ---
        self.path_direction_history = deque(maxlen=3) # 최근 3개의 경로 방향 벡터를 저장
        self.path_direction_smoothing_factor = 0.6 # 새로운 방향 벡터에 대한 가중치


    def _normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _angle_between_vectors(self, v1, v2):
        """Calculates the angle in radians between two vectors."""
        v1_u = v1 / (np.linalg.norm(v1) + 1e-6)
        v2_u = v2 / (np.linalg.norm(v2) + 1e-6)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def _generate_fallback_path(self, blue_cones, yellow_cones, car_pos, car_yaw):
        """Generates a simple straight path if Delaunay is not possible, handling single cone scenarios."""
        
        effective_blue_cones = blue_cones
        effective_yellow_cones = yellow_cones

        if not blue_cones and effective_yellow_cones:
            # Only yellow cones, create virtual blue cones
            effective_blue_cones = []
            for y_cone in effective_yellow_cones:
                # Offset perpendicular to the car's current direction
                # Assuming y_cone is [x, y] in vehicle frame or map frame
                # If in map frame, offset needs to be relative to car_yaw
                # For simplicity, let's assume a fixed offset perpendicular to the track
                effective_blue_cones.append({'x': y_cone['x'] - self.lane_offset * math.sin(car_yaw),
                                             'y': y_cone['y'] + self.lane_offset * math.cos(car_yaw)})
            rospy.logwarn_throttle(1.0, "PathPlanner: Only yellow cones, generating virtual blue cones.")
        elif effective_blue_cones and not effective_yellow_cones:
            # Only blue cones, create virtual yellow cones
            effective_yellow_cones = []
            for b_cone in effective_blue_cones:
                effective_yellow_cones.append({'x': b_cone['x'] + self.lane_offset * math.sin(car_yaw),
                                               'y': b_cone['y'] - self.lane_offset * math.cos(car_yaw)})
            rospy.logwarn_throttle(1.0, "PathPlanner: Only blue cones, generating virtual yellow cones.")
        elif not effective_blue_cones and not effective_yellow_cones:
            rospy.logwarn_throttle(1.0, "PathPlanner: No cones for fallback path.")
            return None, None, None, None, None, True

        rospy.logwarn_throttle(1.0, "PathPlanner: Not enough cones for triangulation, generating fallback path.")
        
        avg_blue = np.mean(np.array([[c['x'], c['y']] for c in effective_blue_cones]), axis=0)
        avg_yellow = np.mean(np.array([[c['x'], c['y']] for c in effective_yellow_cones]), axis=0)
        
        midpoint = (avg_blue + avg_yellow) / 2.0
        
        # Generate a direction vector from car_pos towards the midpoint, aligned with car_yaw
        # This ensures the fallback path is somewhat aligned with the car's current heading
        direction_to_midpoint = midpoint - car_pos
        
        # Project direction_to_midpoint onto the car's current heading to get a forward-biased direction
        car_heading_vec = np.array([math.cos(car_yaw), math.sin(car_yaw)])
        
        # Use a weighted average or just the car's heading if the midpoint is too far off
        # For a simple straight path, let's just extend from car_pos in car_yaw direction
        
        # Let's make the fallback path point towards the midpoint, but ensure it's not going backward
        
        # Calculate a target direction that blends the car's current heading and the direction to the midpoint
        # This helps in smoothly transitioning to the fallback path
        
        # Option 1: Simple straight path along car's current yaw
        # direction_vec_normalized = car_heading_vec
        
        # Option 2: Path towards the midpoint
        if np.linalg.norm(direction_to_midpoint) < 1:
            direction_vec_normalized = car_heading_vec # If midpoint is too close, just go straight
        else:
            direction_vec_normalized = direction_to_midpoint / np.linalg.norm(direction_to_midpoint)
            # Ensure it's generally forward
            if np.dot(direction_vec_normalized, car_heading_vec) < 0: # If pointing backward
                direction_vec_normalized = car_heading_vec # Revert to car's heading
        
        # Generate path points
        path = [car_pos + direction_vec_normalized * i * 1.0 for i in range(1, 6)] # 5 points, 1m spacing
        
        # Add the midpoint as the first point if it's not too far
        if np.linalg.norm(midpoint - car_pos) < 10.0: # Only if midpoint is reasonably close
            path.insert(0, midpoint)
            
        return np.array(path), None, None, None, None, True # Add a flag to indicate it's a fallback path

    def _sort_midpoints(self, midpoints, car_pos, car_yaw):
        """Sorts midpoints into a logical path, starting near the car and following the track's flow."""
        if len(midpoints) < 2:
            return midpoints

        midpoints_list = midpoints.tolist()
        
        # Find the best starting point: close and in front of the car
        start_idx = -1
        min_cost = float('inf')
        for i, p in enumerate(midpoints_list):
            dist = np.hypot(p[0] - car_pos[0], p[1] - car_pos[1])
            angle_to_point = math.atan2(p[1] - car_pos[1], p[0] - car_pos[0])
            angle_diff = self._normalize_angle(angle_to_point - car_yaw)
            
            if abs(angle_diff) < (math.pi / 1.5): # Wider 120-degree arc
                cost = dist * (1 + abs(angle_diff)) # Penalize points off to the side
                if cost < min_cost:
                    min_cost = cost
                    start_idx = i
        
        if start_idx == -1: # If no points are in the front arc, fall back to closest
            start_idx = np.argmin([np.hypot(p[0] - car_pos[0], p[1] - car_pos[1]) for p in midpoints_list])

        ordered_path = [midpoints_list.pop(start_idx)]

        # Establish initial direction with the second point
        if midpoints_list:
            last_point = ordered_path[-1]
            
            best_next_idx = -1
            min_cost = float('inf')
            for i, p in enumerate(midpoints_list):
                dist = np.hypot(p[0] - last_point[0], p[1] - last_point[1])
                angle_to_point = math.atan2(p[1] - last_point[1], p[0] - last_point[0])
                angle_diff = self._normalize_angle(angle_to_point - car_yaw) # Compare with car's yaw

                cost = dist * (1 + abs(angle_diff))
                if cost < min_cost:
                    min_cost = cost
                    best_next_idx = i
            
            if best_next_idx != -1:
                ordered_path.append(midpoints_list.pop(best_next_idx))

        # Sort the rest based on a cost function of distance and angle
        while midpoints_list and len(ordered_path) >= 2:
            last_point = np.array(ordered_path[-1])
            
            # --- 경로 방향성 계산 로직 개선 ---
            # 직전 두 점이 아닌, 최근 경로의 전반적인 방향을 사용
            current_path_vec = last_point - np.array(ordered_path[-2])
            self.path_direction_history.append(current_path_vec / (np.linalg.norm(current_path_vec) + 1e-6))
            
            # 이동 평균을 사용하여 부드러운 경로 방향 벡터 계산
            smooth_path_vec = np.mean(self.path_direction_history, axis=0)
            smooth_path_vec /= (np.linalg.norm(smooth_path_vec) + 1e-6)
            
            best_candidate_idx = -1
            min_cost = float('inf')

            for i, candidate_point in enumerate(midpoints_list):
                candidate_point = np.array(candidate_point)
                dist = np.linalg.norm(candidate_point - last_point)
                
                if dist > self.max_edge_length: # Don't jump too far
                    continue

                candidate_vec = candidate_point - last_point
                
                # Angle relative to the current path segment
                angle_path_segment = self._angle_between_vectors(smooth_path_vec, candidate_vec)

                # Combine these angles into the cost function
                norm_dist = dist / self.max_edge_length
                norm_angle_path = angle_path_segment / math.pi

                cost = self.w_dist * norm_dist + self.w_angle * norm_angle_path
                
                if cost < min_cost:
                    min_cost = cost
                    best_candidate_idx = i
            
            if best_candidate_idx != -1:
                ordered_path.append(midpoints_list.pop(best_candidate_idx))
            else:
                break # No suitable point found
        
        return np.array(ordered_path)

    def _correct_path_detours(self, path, car_yaw):
        if len(path) < 2:
            return path

        corrected_path = [path[0]]
        for i in range(len(path) - 1):
            p1 = corrected_path[-1]
            p2 = path[i+1]

            segment_vec = p2 - p1
            
            # Check if the segment is going backward relative to the car's yaw
            angle_to_car_yaw = self._normalize_angle(math.atan2(segment_vec[1], segment_vec[0]) - car_yaw)
            
            if abs(angle_to_car_yaw) > (math.pi / 2.0): # If segment is pointing more than 90 degrees away from car_yaw
                # This segment is going backward or sharply sideways. Ignore this point.
                continue
            else:
                corrected_path.append(p2)
        
        return np.array(corrected_path)

    def _is_valid_cone_pair(self, p1_idx, p2_idx, all_points, colors):
        """
        두 콘(p1, p2)이 유효한 중간점 생성 쌍인지 확인합니다.
        규칙: p1과 p2를 잇는 직선 위에 다른 콘이 너무 가까이 있으면 안 됩니다.
        """
        p1 = all_points[p1_idx]
        p2 = all_points[p2_idx]
        # color1 = colors[p1_idx] # No longer needed

        # p1에서 p2로 향하는 벡터
        line_vec = p2 - p1
        line_len_sq = np.dot(line_vec, line_vec)

        if line_len_sq == 0:
            return False

        for i in range(len(all_points)):
            if i == p1_idx or i == p2_idx: # Check all other cones, regardless of color
                continue
            
            p3 = all_points[i]
            # p3가 p1-p2 선분 위에 있는지 확인 (투영(projection) 사용)
            dot_product = np.dot(p3 - p1, line_vec)
            if 0 < dot_product < line_len_sq: # p3가 p1과 p2 사이에 투영되는 경우
                # 선분과의 거리 계산
                dist_to_line = np.linalg.norm(np.cross(line_vec, p1 - p3)) / np.linalg.norm(line_vec)
                if dist_to_line < self.path_obstacle_threshold:  # 임계값보다 가까우면 방해물로 간주
                    return False # 유효하지 않은 쌍
        return True

    def _filter_cones_for_planning(self, cones, vehicle_state):
        """
        경로 계획에 사용할 콘을 차량 주변의 관심 영역(ROI)으로 필터링합니다.
        """
        car_pos = vehicle_state[:2]
        car_yaw = vehicle_state[2]
        
        filtered_cones = []
        for cone in cones:
            cone_pos = np.array([cone['x'], cone['y']])
            
            # 1. 거리 필터
            dist = np.linalg.norm(cone_pos - car_pos)
            if dist > self.planner_roi_distance:
                continue
                
            # 2. 각도 필터
            angle_to_cone = math.atan2(cone_pos[1] - car_pos[1], cone_pos[0] - car_pos[0])
            angle_diff = self._normalize_angle(angle_to_cone - car_yaw)
            if abs(angle_diff) > self.planner_roi_angle_rad:
                continue
            
            filtered_cones.append(cone)
        return filtered_cones

    def plan_path(self, cones, vehicle_state):
        """
        Generates a driving path based on the detected cones.
        Uses Delaunay triangulation and a robust sorting algorithm.
        Falls back to a simple path if not enough cones are available.
        """
        current_car_pos = vehicle_state[:2]
        vehicle_yaw = vehicle_state[2]

        # --- 경로 계획에 사용할 콘 필터링 ---
        local_cones = self._filter_cones_for_planning(cones, vehicle_state)

        # --- 클러스터링 단계 추가 ---
        if len(local_cones) > 5: # 최소 5개 이상의 콘이 있을 때만 클러스터링 수행
            cone_points = np.array([[c['x'], c['y']] for c in local_cones])
            
            # DBSCAN을 사용하여 콘 그룹 찾기
            db = DBSCAN(eps=self.planner_clustering_eps, min_samples=3).fit(cone_points)
            labels = db.labels_
            
            unique_labels = set(labels)
            if -1 in unique_labels: # 노이즈 레이블 제거
                unique_labels.remove(-1)

            # 두 개 이상의 클러스터가 감지된 경우
            if len(unique_labels) > 1:
                closest_cluster_idx = -1
                min_dist_to_cluster = float('inf')
                
                # 차량에서 가장 가까운 클러스터 찾기
                for label in unique_labels:
                    cluster_mask = (labels == label)
                    cluster_points = cone_points[cluster_mask]
                    cluster_center = np.mean(cluster_points, axis=0)
                    dist_to_car = np.linalg.norm(cluster_center - current_car_pos)
                    
                    if dist_to_car < min_dist_to_cluster:
                        min_dist_to_cluster = dist_to_car
                        closest_cluster_idx = label
                
                # 가장 가까운 클러스터에 속한 콘만 사용하도록 필터링
                if closest_cluster_idx != -1:
                    closest_cluster_mask = (labels == closest_cluster_idx)
                    local_cones = [cone for i, cone in enumerate(local_cones) if closest_cluster_mask[i]]
                    rospy.loginfo_throttle(1.0, f"PathPlanner: Multiple cone clusters found. Using closest cluster with {len(local_cones)} cones.")
        
        blue_cones = [c for c in local_cones if c['color_id'] == 1]
        yellow_cones = [c for c in local_cones if c['color_id'] == 2]

        # --- Condition for Delaunay Path ---
        if len(blue_cones) < 2 or len(yellow_cones) < 2:
            path, tri, all_points, colors, unique_midpoints, is_fallback = self._generate_fallback_path(blue_cones, yellow_cones, current_car_pos, vehicle_yaw)
            return path, tri, all_points, colors, unique_midpoints, is_fallback

        # 1. Prepare points for triangulation
        all_points = np.array([[c['x'], c['y']] for c in blue_cones] + [[c['x'], c['y']] for c in yellow_cones])
        if len(all_points) < 3:
            path, tri, all_points, colors, unique_midpoints, is_fallback = self._generate_fallback_path(blue_cones, yellow_cones, current_car_pos, vehicle_yaw)
            return path, tri, all_points, colors, unique_midpoints, is_fallback

        num_blue = len(blue_cones)
        colors = np.array([1] * num_blue + [2] * len(yellow_cones))

        # 2. Perform Delaunay Triangulation
        try:
            tri = Delaunay(all_points)
        except Exception as e:
            rospy.logwarn(f"Delaunay triangulation failed: {e}")
            return None, None, None, None, None, True # Indicate fallback

        # 3. Find centerline midpoints
        midpoints = []
        for simplex in tri.simplices:
            indices = sorted(simplex) # 순서를 고정하여 중복 방지
            for i in range(3):
                p1_idx = indices[i]
                p2_idx = indices[(i + 1) % 3]

                # 색이 다른 두 콘을 연결하되, 유효한 쌍인지 검사
                if colors[p1_idx] != colors[p2_idx]:
                    if self._is_valid_cone_pair(p1_idx, p2_idx, all_points, colors) and \
                       self._is_valid_cone_pair(p2_idx, p1_idx, all_points, colors):
                        if np.linalg.norm(all_points[p1_idx] - all_points[p2_idx]) < self.max_edge_length:
                            midpoints.append((all_points[p1_idx] + all_points[p2_idx]) / 2.0)
        
        if not midpoints:
            return None, tri, all_points, colors, None, True # Indicate fallback

        # 4. Sort midpoints to form a continuous path
        unique_midpoints = np.unique(np.array(midpoints), axis=0)
        if len(unique_midpoints) < 2:
            return None, tri, all_points, colors, unique_midpoints, True # Indicate fallback

        ordered_midpoints = self._sort_midpoints(unique_midpoints, current_car_pos, vehicle_yaw)
        if ordered_midpoints is None or len(ordered_midpoints) < 2:
            return None, tri, all_points, colors, unique_midpoints, True # Indicate fallback

        self.path_direction_history.clear() # 새로운 계획 시작 시 방향 기록 초기화
        # Correct any detours in the path
        corrected_path = self._correct_path_detours(ordered_midpoints, vehicle_yaw)

        # Filter path to include only points within max_path_distance from the car
        filtered_path = []
        for p in corrected_path:
            if np.linalg.norm(p - current_car_pos) < self.max_path_distance:
                filtered_path.append(p)
        
        # Handle cases with very few points
        if len(filtered_path) == 0:
            return None, tri, all_points, colors, unique_midpoints, True # Indicate fallback
        elif len(filtered_path) == 1:
            # If only one point, create a path from current car position to that point
            path = np.array([current_car_pos, filtered_path[0]])
            rospy.logwarn_throttle(1.0, "PathPlanner: Only 1 point for path, generating straight line to it.")
            return path, tri, all_points, colors, unique_midpoints, True # Indicate fallback

        # 5. Smooth the path with a spline
        if len(filtered_path) < 3:
            if len(filtered_path) == 2: # If only two points, draw a straight line between them
                path = np.array(filtered_path)
                rospy.logwarn_throttle(1.0, "PathPlanner: Only 2 points for spline, generating straight path.")
            else: # This case should ideally not be reached if previous checks are correct
                fallback_path, _, _, _, _, is_fallback = self._generate_fallback_path(blue_cones, yellow_cones, current_car_pos, vehicle_yaw)
                return fallback_path, tri, all_points, colors, unique_midpoints, is_fallback

        try:
            if len(filtered_path) >= 3: # Only attempt spline if 3 or more points
                k = min(2, len(filtered_path) - 1)
                tck, u = splprep([np.array(filtered_path)[:, 0], np.array(filtered_path)[:, 1]], s=self.spline_smoothing_factor, k=k)
                u_new = np.linspace(u.min(), u.max(), 50)
                x_new, y_new = splev(u_new, tck)
                path = np.vstack((x_new, y_new)).T
            # If len(filtered_path) was 1 or 2, path is already set to a straight line
        except Exception as e:
            rospy.logwarn(f"Spline generation failed: {e}. Returning raw midpoints or straight path.")
            if len(filtered_path) >= 1:
                # If spline fails, and we have at least one point, return the raw filtered path
                path = np.array(filtered_path) 
            else:
                # This case should ideally not be reached if previous checks are correct
                return None, tri, all_points, colors, unique_midpoints, True # Indicate fallback
            return path, tri, all_points, colors, unique_midpoints, True # Indicate fallback

        return path, tri, all_points, colors, unique_midpoints, False # Not a fallback path

    def _calculate_path_curvature(self, path, lookahead=5):
        """경로의 각 지점에서 곡률을 계산합니다."""
        curvatures = [0.0] * len(path)
        if len(path) < 3:
            return curvatures

        for i in range(len(path)):
            p_prev_idx = max(0, i - lookahead)
            p_next_idx = min(len(path) - 1, i + lookahead)
            
            if p_prev_idx == i or p_next_idx == i: continue

            p_prev, p_curr, p_next = path[p_prev_idx], path[i], path[p_next_idx]
            # Menger Curvature: 세 점으로 곡률 근사
            area = 0.5 * abs((p_prev[0]*(p_curr[1]-p_next[1]) + p_curr[0]*(p_next[1]-p_prev[1]) + p_next[0]*(p_prev[1]-p_curr[1])))
            curvatures[i] = (4 * area) / (np.linalg.norm(p_prev-p_curr) * np.linalg.norm(p_curr-p_next) * np.linalg.norm(p_next-p_prev) + 1e-6)
        return curvatures
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

    def _normalize_angle_difference(self, angle1, angle2):
        """Normalizes the difference between two angles to be within [-pi, pi]."""
        diff = angle1 - angle2
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff

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
            # This is the complementary filter step, handling angle wrapping.
            yaw_diff = self._normalize_angle_difference(yaw_from_imu, predicted_state[2])
            fused_yaw = predicted_state[2] + self.yaw_filter_alpha * yaw_diff
            
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
    def __init__(self, log_directory: str, session_name: str, video_fps: float = 10.0, max_lidar_points: int = 500, enable_logging: bool = True):
        self.enable_logging = enable_logging
        self.session_path = None
        self.metadata_csv_file = None
        self.metadata_csv_writer = None
        self.video_paths = {'cam1': None, 'cam2': None}
        self.video_writers = {'cam1': None, 'cam2': None}
        self.video_fps = video_fps
        self.fourcc = None
        self.max_lidar_points = max_lidar_points
        self.lidar_csv_path = None
        self.lidar_csv_file = None
        self.lidar_csv_writer = None
        self.map_cones_csv_path = None
        self.map_cones_csv_file = None
        self.map_cones_csv_writer = None
        self.frame_count = 0

        if not self.enable_logging:
            rospy.loginfo("DataLogger: Logging is disabled.")
            return

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

        rospy.loginfo(f"DataLogger initialized. Saving logs to: {self.session_path}")

    def log_entry(self, autonomous_mode: str, control_command: ControlCommand,
                  imu_acc: list, imu_gyro: list, state: list,
                  camera1_image: np.ndarray, camera2_image: np.ndarray, lidar_points: np.ndarray,
                  map_cones: list):
        if not self.enable_logging:
            return

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
        if not self.enable_logging:
            return
        
        if self.metadata_csv_file: # Check if file was opened
            self.metadata_csv_file.close()
            rospy.loginfo(f"Successfully saved metadata to {self.csv_path}")

        for cam_id, writer in self.video_writers.items():
            if writer is not None:
                writer.release()
                rospy.loginfo(f"Successfully saved video to {self.video_paths[cam_id]}")

        if self.lidar_csv_file: # Check if file was opened
            self.lidar_csv_file.close()
            rospy.loginfo(f"Successfully saved LiDAR data to {self.lidar_csv_path}")

        if self.map_cones_csv_file: # Check if file was opened
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
    """
    Main control class that manages and selects the active path tracking controller.
    """
    def __init__(self, path_planner, track_map):
        # --- Controller Selection ---
        try:
            self.path_planner = path_planner # PathPlanner 인스턴스를 멤버 변수로 저장
            self.track_map = track_map # TrackMap 인스턴스를 멤버 변수로 저장
            self.controller_type = rospy.get_param("/control/ControllerSelection/lateral_controller_type", "PurePursuit")
        except (rospy.ROSException, KeyError):
            self.controller_type = "PurePursuit"
            rospy.logwarn("Control: '/control/controller_type' parameter not found. Defaulting to 'PurePursuit'.")

        # --- Get all parameters for all controllers ---
        # Vehicle
        self.wheelbase = rospy.get_param("/control/Vehicle/wheelbase", 1.54)
        self.max_steer = math.radians(rospy.get_param("/control/Vehicle/max_steer_angle", 10.0)) # radians
        self.max_accel = rospy.get_param("/control/Vehicle/max_accel", 0.5) # m/s^2
        self.min_accel = rospy.get_param("/control/Vehicle/min_accel", -0.5) # m/s^2 (braking)

        # Common
        self.target_speed = rospy.get_param("/control/SpeedControl/target_speed", 5.0) # m/s
        self.kp_throttle = rospy.get_param("/control/SpeedControl/pid_kp", 0.5)
        
        # --- 곡률 기반 속도 제어 파라미터 ---
        self.max_speed = rospy.get_param("/control/SpeedControl/target_speed", 5.0)
        self.loop_closed_target_speed = rospy.get_param("/control/SpeedControl/loop_closed_target_speed", 10.0)
        self.min_speed = rospy.get_param("/control/SpeedControl/min_speed", 2.0) # 코너 최소 속도
        self.curvature_speed_factor = rospy.get_param("/control/SpeedControl/curvature_factor", 2.5) # 곡률에 따른 감속 강도
        self.curvature_lookahead = rospy.get_param("/control/SpeedControl/curvature_lookahead", 5) # 곡률 계산 시 참고할 포인트 거리
        self.lc_target_speed_duration = rospy.get_param("/control/lc_target_speed_duration", 5.0) # 루프 클로저 시 속도 감소 유지 시간
        self.lc_speed_reduction_start_time = 0.0
        self.is_lc_active = False
        self.pre_lc_target_speed = rospy.get_param("/control/SpeedControl/pre_loop_closure/target_speed", self.target_speed) # 루프 클로저 전 목표 속도 저장
        self.post_lc_target_speed = rospy.get_param("/control/SpeedControl/post_loop_closure/target_speed", self.loop_closed_target_speed) # 루프 클로저 후 목표 속도 저장
        self.pre_lc_min_speed = rospy.get_param("/control/SpeedControl/pre_loop_closure/min_speed", self.min_speed) # 루프 클로저 전 최소 속도
        self.post_lc_min_speed = rospy.get_param("/control/SpeedControl/post_loop_closure/min_speed", self.min_speed) # 루프 클로저 후 최소 속도
        self.pre_lc_curvature_speed_factor = rospy.get_param("/control/SpeedControl/pre_loop_closure/curvature_speed_factor", self.curvature_speed_factor) # 루프 클로저 전 곡률 감속 계수
        self.post_lc_curvature_speed_factor = rospy.get_param("/control/SpeedControl/post_loop_closure/curvature_speed_factor", self.curvature_speed_factor) # 루프 클로저 후 곡률 감속 계수
        self.pre_lc_curvature_lookahead = rospy.get_param("/control/SpeedControl/pre_loop_closure/curvature_lookahead", self.curvature_lookahead) # 루프 클로저 전 곡률 계산 시 참고할 포인트 거리
        self.post_lc_curvature_lookahead = rospy.get_param("/control/SpeedControl/post_loop_closure/curvature_lookahead", self.curvature_lookahead) # 루프 클로저 후 곡률 계산 시 참고할 포인트 거리


        # Pure Pursuit
        self.lookahead_dist = rospy.get_param("/control/pure_pursuit/lookahead_distance", 2.5)
        
        # Stanley
        self.k_crosstrack = rospy.get_param("/control/stanley/k_gain", 0.7)
        # MPC
        self.mpc_weights_pre_lc = rospy.get_param("/control/MPC/pre_loop_closure")
        self.mpc_weights_post_lc = rospy.get_param("/control/MPC/post_loop_closure")
        self.mpc_speed_scaling_factor = rospy.get_param("/control/MPC/mpc_speed_scaling_factor", 1.0) # New parameter
        self.max_speed_for_scaling = rospy.get_param("/control/SpeedControl/max_speed_for_scaling", 10.0) # Use max target speed for normalization

        # --- Assign the compute function based on selected type ---
        if self.controller_type == "PurePursuit":
            self.compute_control = self._compute_pure_pursuit
        elif self.controller_type == "Stanley":
            self.compute_control = self._compute_stanley
        elif self.controller_type == "MPC":
            self.compute_control = self._compute_mpc
        else:
            rospy.logerr(f"Control: Invalid controller type '{self.controller_type}'. Defaulting to 'pure_pursuit'.")
            self.compute_control = self._compute_pure_pursuit
            
        rospy.loginfo(f"Control: Using {self.controller_type} controller.")

    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _compute_pure_pursuit(self, vehicle_state, path, is_fallback=False):
        # Unpack vehicle state
        veh_x, veh_y, veh_yaw = vehicle_state[0], vehicle_state[1], vehicle_state[2]
        current_speed = math.sqrt(vehicle_state[3]**2 + vehicle_state[4]**2)

        # --- 곡률 기반 목표 속도 계산 ---
        if is_fallback:
            target_speed = 5.0 # Set speed to 5.0 for fallback paths
        else:
            path_curvatures = self.path_planner._calculate_path_curvature(path, self.curvature_lookahead)
            # 전방 경로의 평균 곡률 계산 (예: 앞 10개 포인트)
            lookahead_curvatures = path_curvatures
            avg_curvature = np.mean(lookahead_curvatures) if lookahead_curvatures else 0.0
            
            if self.track_map.is_loop_closed:
                base_target_speed = self.loop_closed_target_speed
            else:
                base_target_speed = self.max_speed

            target_speed = base_target_speed / (1.0 + self.curvature_speed_factor * abs(avg_curvature))
            target_speed = np.clip(target_speed, self.min_speed, base_target_speed)

        # 1. Find the closest point on the path to the vehicle
        path_points = np.array(path)
        distances = np.linalg.norm(path_points - np.array([veh_x, veh_y]), axis=1)
        closest_idx = np.argmin(distances)

        # 2. Find the lookahead point
        lookahead_point = None
        for i in range(closest_idx, len(path_points)):
            dist_from_veh = np.linalg.norm(path_points[i] - np.array([veh_x, veh_y]))
            if dist_from_veh >= self.lookahead_dist:
                lookahead_point = path_points[i]
                break
        
        if lookahead_point is None:
            lookahead_point = path_points[-1]

        # 3. Transform the lookahead point to the vehicle's coordinate frame
        rot_inv = np.array([[math.cos(veh_yaw), math.sin(veh_yaw)],
                            [-math.sin(veh_yaw), math.cos(veh_yaw)]])
        translated_point = lookahead_point - np.array([veh_x, veh_y])
        local_point = rot_inv.dot(translated_point)

        # 4. Calculate the steering angle
        alpha = math.atan2(local_point[1], local_point[0])
        actual_lookahead_dist = np.linalg.norm(lookahead_point - np.array([veh_x, veh_y]))
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), actual_lookahead_dist)
        steer = -np.clip(steer, -self.max_steer, self.max_steer)

        # 5. Throttle control
        throttle = self.kp_throttle * (target_speed - current_speed)
        throttle = np.clip(throttle, 0.0, 1.0)
        
        brake = 0.0
        if target_speed < current_speed:
            brake = 0.3

        # Normalize steering angle to [-1, 1]
        normalized_steer = steer / self.max_steer

        return throttle, normalized_steer, brake

    def _compute_stanley(self, vehicle_state, path, is_fallback=False):
        """
        Computes control commands using the Stanley method.
        """
        # Unpack vehicle state
        veh_x, veh_y, veh_yaw = vehicle_state[0], vehicle_state[1], vehicle_state[2]
        current_speed = math.sqrt(vehicle_state[3]**2 + vehicle_state[4]**2)

        # --- 곡률 기반 목표 속도 계산 ---
        if is_fallback:
            target_speed = 5.0 # Set speed to 5.0 for fallback paths
        else:
            if self.track_map.is_loop_closed:
                target_speed_params = {
                    'target_speed': self.post_lc_target_speed,
                    'min_speed': self.post_lc_min_speed,
                    'curvature_speed_factor': self.post_lc_curvature_speed_factor,
                    'curvature_lookahead': self.post_lc_curvature_lookahead
                }
            else:
                target_speed_params = {
                    'target_speed': self.pre_lc_target_speed,
                    'min_speed': self.pre_lc_min_speed,
                    'curvature_speed_factor': self.pre_lc_curvature_speed_factor,
                    'curvature_lookahead': self.pre_lc_curvature_lookahead
                }

            path_curvatures = self.path_planner._calculate_path_curvature(path, target_speed_params['curvature_lookahead'])
            lookahead_curvatures = path_curvatures[:10]
            avg_curvature = np.mean(lookahead_curvatures) if lookahead_curvatures else 0.0
            
            base_target_speed = target_speed_params['target_speed']

            target_speed = base_target_speed / (1.0 + target_speed_params['curvature_speed_factor'] * abs(avg_curvature))
            target_speed = np.clip(target_speed, target_speed_params['min_speed'], base_target_speed)
        # 1. Find the closest path point (target_idx)
        path_points = np.array(path)
        distances = np.linalg.norm(path_points - np.array([veh_x, veh_y]), axis=1)
        target_idx = np.argmin(distances)

        # Ensure target_idx is not the last point of the path to calculate path heading
        if target_idx >= len(path_points) - 1:
            target_idx = len(path_points) - 2

        # 2. Calculate path heading (yaw)
        p1 = path_points[target_idx]
        p2 = path_points[target_idx + 1]
        path_yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])

        # 3. Calculate heading error (theta_e)
        heading_error = self.normalize_angle(path_yaw - veh_yaw)

        # 4. Calculate cross-track error (e_fa)
        # Vector from closest path point to vehicle
        vec_path_to_veh = np.array([veh_x, veh_y]) - p1
        # Path vector
        vec_path = p2 - p1
        vec_path_normalized = vec_path / (np.linalg.norm(vec_path) + 1e-6)
        
        # Cross product to find the error and its sign
        cross_track_error = np.cross(vec_path_normalized, vec_path_to_veh)
        
        # 5. Calculate steering angle (delta)
        # Cross-track steering component
        cte_steer = math.atan2(self.k_crosstrack * cross_track_error, max(current_speed, 0.1)) # Add small epsilon to avoid division by zero

        # Total steering angle
        steer = heading_error + cte_steer
        steer = -np.clip(steer, -self.max_steer, self.max_steer)

        # 6. Throttle control (reusing the same P-controller)
        throttle = self.kp_throttle * (target_speed - current_speed)
        throttle = np.clip(throttle, 0.0, 1.0)
        
        brake = 0.0
        if target_speed < current_speed:
            brake = 0.1

        # Normalize steering angle to [-1, 1]
        normalized_steer = steer / self.max_steer
        return throttle, normalized_steer, brake

    def _cost_function(self, u, *args):
        initial_state, ref_path, target_speed, weights, horizon, dt = args
        
        # Unpack control inputs
        accels = u[0::2]
        steers = u[1::2]

        # Predict states over the horizon
        predicted_states = np.zeros((horizon + 1, 4))
        predicted_states[0] = initial_state
        for i in range(horizon):
            # Kinematic Bicycle Model
            x, y, yaw, v = predicted_states[i]
            a = accels[i]
            delta = steers[i]
            # Update state
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += v / self.wheelbase * math.tan(delta) * dt
            v += a * dt
            predicted_states[i+1] = [x, y, yaw, v]
        # Calculate cost
        cost = 0.0
        closest_idx = 0
        search_window = 5  
        # Find closest reference path points for each predicted state
        for i in range(1, horizon + 1):
            pred_x, pred_y, pred_yaw, pred_v = predicted_states[i]
            # Find closest point on reference path in a search window
            start_idx = max(0, closest_idx - search_window)
            end_idx = min(len(ref_path), closest_idx + search_window)
            search_space = ref_path[start_idx:end_idx]
            distances = np.linalg.norm(search_space - np.array([pred_x, pred_y]), axis=1)
            closest_local_idx = np.argmin(distances)
            closest_idx = start_idx + closest_local_idx
            # Find closest point on reference path
            # distances = np.linalg.norm(ref_path - np.array([pred_x, pred_y]), axis=1)
            # closest_idx = np.argmin(distances)
            
            # Cross-track error
            ref_p1 = ref_path[closest_idx]
            ref_p2 = ref_path[closest_idx + 1] if closest_idx < len(ref_path) - 1 else ref_p1

            vec_path = ref_p2 - ref_p1
            vec_path_normalized = vec_path / (np.linalg.norm(vec_path) + 1e-6)
            vec_to_pred = np.array([pred_x, pred_y]) - ref_p1
            cte = np.cross(vec_path_normalized, vec_to_pred)

            # Heading error
            ref_yaw = math.atan2(ref_p2[1] - ref_p1[1], ref_p2[0] - ref_p1[0])
            etheta = self.normalize_angle(pred_yaw - ref_yaw)

            cost += weights['w_cte'] * cte**2
            cost += weights['w_etheta'] * etheta**2
            cost += weights['w_vel'] * (target_speed - pred_v)**2

        # Control input cost
        cost += weights['w_accel'] * np.sum(accels**2)
        cost += weights['w_steer'] * np.sum(steers**2)

        # Control rate cost (smoothness)
        cost += weights['w_accel_rate'] * np.sum((accels[1:] - accels[:-1])**2)
        cost += weights['w_steer_rate'] * np.sum((steers[1:] - steers[:-1])**2)
        
        self._predicted_states = predicted_states # Store for visualization
        return cost

    def _compute_mpc(self, vehicle_state, path, is_fallback=False):
        # Unpack vehicle state
        veh_x, veh_y, veh_yaw = vehicle_state[0], vehicle_state[1], vehicle_state[2]
        current_speed = math.sqrt(vehicle_state[3]**2 + vehicle_state[4]**2)
        initial_state = [veh_x, veh_y, veh_yaw, current_speed]

        # --- 곡률 기반 목표 속도 계산 ---
        if is_fallback:
            mpc_target_speed = (self.pre_lc_target_speed + self.pre_lc_min_speed)/2 # Set speed to 5.0 for fallback paths
            weights = self.mpc_weights_pre_lc.copy() # Use pre-LC weights for fallback
        else:
            if self.track_map.is_loop_closed:
                target_speed_params = {
                    'target_speed': self.post_lc_target_speed,
                    'min_speed': self.post_lc_min_speed,
                    'curvature_speed_factor': self.post_lc_curvature_speed_factor,
                    'curvature_lookahead': self.post_lc_curvature_lookahead
                }
                weights = self.mpc_weights_post_lc.copy() # Use a copy to avoid modifying original params
            else:
                target_speed_params = {
                    'target_speed': self.pre_lc_target_speed,
                    'min_speed': self.pre_lc_min_speed,
                    'curvature_speed_factor': self.pre_lc_curvature_speed_factor,
                    'curvature_lookahead': self.pre_lc_curvature_lookahead
                }
                weights = self.mpc_weights_pre_lc.copy() # Use a copy to avoid modifying original params

            path_curvatures = self.path_planner._calculate_path_curvature(path, target_speed_params['curvature_lookahead'])
            lookahead_curvatures = path_curvatures[:5]
            avg_curvature = np.mean(lookahead_curvatures) if lookahead_curvatures else 0.0
            
            mpc_target_speed = target_speed_params['target_speed'] / (1.0 + target_speed_params['curvature_speed_factor'] * abs(avg_curvature))
            mpc_target_speed = np.clip(mpc_target_speed, target_speed_params['min_speed'], target_speed_params['target_speed'])

        horizon = weights.get('horizon', 10) # default to 10 if not found
        dt = weights.get('dt', 0.1) # default to 0.1 if not found

        # Apply speed-proportional scaling to MPC weights
        # Normalize current speed by a maximum expected speed for scaling factor
        normalized_speed = current_speed / self.max_speed_for_scaling
        # Use a power function to make scaling more aggressive or less aggressive
        speed_scaling_factor = (normalized_speed ** self.mpc_speed_scaling_factor) if self.mpc_speed_scaling_factor != 0 else 1.0
        speed_scaling_factor = np.clip(speed_scaling_factor, 0.1, 2.0) # Clip to reasonable range

        # Example: Increase path following weights with speed, decrease control input weights
        weights['w_cte'] *= speed_scaling_factor
        weights['w_etheta'] *= speed_scaling_factor
        weights['w_vel'] *= speed_scaling_factor # Penalize velocity error more at higher speeds

        # At higher speeds, generally want tighter path following and smoother control inputs.
        # So, increase penalties for path deviation and control input changes.
        weights['w_accel'] *= speed_scaling_factor
        weights['w_steer'] /= speed_scaling_factor
        weights['w_accel_rate'] *= speed_scaling_factor
        weights['w_steer_rate'] *= speed_scaling_factor

        # Get reference path for the horizon
        path_points = np.array(path, dtype=np.float32)
        distances = np.linalg.norm(path_points - np.array([veh_x, veh_y]), axis=1)
        start_idx = np.argmin(distances)
        ref_path = path_points[start_idx:start_idx + horizon + 2] # Need one extra point for heading calculation
        if len(ref_path) < horizon + 2:
            # Pad the reference path if it's too short
            last_point = ref_path[-1]
            padding = np.array([last_point] * (horizon + 2 - len(ref_path)))
            ref_path = np.vstack([ref_path, padding])

        # Initial guess for control inputs (warm start)
        if not hasattr(self, "_u_prev") or self._u_prev.shape[0] != 2 * horizon:
            self._u_prev = np.zeros(2 * horizon)
        
        u0 = np.roll(self._u_prev, -2)
        u0[-2:] = u0[-4:-2]

        # Bounds for control inputs
        bounds = []
        for _ in range(horizon):
            bounds.append((self.min_accel, self.max_accel))
            bounds.append((-self.max_steer, self.max_steer))

        # --- Solve the optimization problem ---
        solution = minimize(
            self._cost_function,
            u0,
            args=(initial_state, ref_path, mpc_target_speed, weights, horizon, dt),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 5, 'ftol': 1e-5, 'disp': False}
        )
        self._u_prev = solution.x.copy()

        # Get the predicted states for visualization
        predicted_path = self._predicted_states[:, :2] # Extract x, y coordinates

        # Get the first optimal control input
        optimal_accel = solution.x[0]
        optimal_steer = solution.x[1]
        
        # print("MPC Optimization Success:", solution.success, "Cost:", solution.fun)
        # print("Optimal Accel:", optimal_accel, "Optimal Steer:", optimal_steer)
        # print(self.max_accel, self.min_accel)
        # --- Map acceleration to throttle/brake ---
        throttle = 0.0
        brake = 0.0
        if optimal_accel > 0:
            # Simple mapping: scale accel to [0,1] throttle
            throttle = np.clip(optimal_accel / self.max_accel, 0.0, 1.0)
        else:
            # Simple mapping: scale decel to [0,1] brake
            brake = np.clip(-optimal_accel / abs(self.min_accel), 0.0, 1.0)

        # Normalize steering angle to [-1, 1]
        normalized_steer = np.clip(-optimal_steer / self.max_steer, -1.0, 1.0)

        return throttle, normalized_steer, brake, predicted_path
