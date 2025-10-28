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
import open3d as o3d

# Camera
import torch
import torchvision

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

        self.enable_visualization = rospy.get_param("/system/visualization/enable_visualization", True)
        self.enable_logging = rospy.get_param("/system/logging/enable_logging", True)
        if self.enable_logging:
            # ==================== 데이터 로거 추가 ====================
            self.data_logger = DataLogger(
            log_directory="/home/user/FSDS/log",
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
        self.last_path_index_count = 0
        self.last_midpoints_count = 0
        self.visualization_frame_counter = 0
        self.visualization_publish_interval = 0.5 # Publish visualization every 5 frames

        self.current_path = None
        self.current_path_is_fallback = False
        self.path_end_threshold = 3 # meters. Distance to end of path to trigger replanning.

        # rospkg를 사용하여 모델 경로 동적으로 찾기
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('formula_autonomous_system')
        self.model_path = os.path.join(package_path,'python', 'retinanet_QAT.pt')
        rospy.loginfo(f"Loading model from: {self.model_path}")

        self.model = torch.load(self.model_path, map_location=torch.device('cpu'))
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)
        if self.device.type == 'cuda':
            self.model.half()
            print("Model converted to FP16 (half-precision).")
        print(f"Model target device: {self.device}")
        self.model.eval()

        # System Components
        self.gps_util = GPSIMUProcessor()
        self.state_machine = StateMachine()
        self.lidar_util = LiDARProcessor(self.enable_visualization, self.device)
        self.camera_util = CameraProcessor()
        self.track_map = TrackMap(self.device)
        self.midpoint_map = MidpointMap(self.device)
        self.path_planner = PathPlanner(self.device)
        self.controller = Control(self.path_planner, self.track_map, self.device) # Control 클래스에 path_planner와 track_map 인스턴스 전달

        self.pointcloud_sub = rospy.Subscriber("/lidar_pointcloud", PointCloud2, self.pointcloud_callback)
        self.cones_pub = rospy.Publisher("/detected_cones", PointCloud2, queue_size=10) # Example publisher for detected cones

        # RANSAC parameters for cone detection
        self.ransac_threshold = 0.05  # Distance threshold for inliers
        self.ransac_max_iterations = 100 # Number of RANSAC iterations
        self.cone_min_radius = 0.1    # Minimum expected cone base radius (e.g., 10 cm)
        self.cone_max_radius = 0.3    # Maximum expected cone base radius (e.g., 30 cm)
        self.ground_plane_z_threshold = 0.1 # Max Z distance from ground to consider for cone base

        # DBSCAN parameters for clustering detected cone points
        self.dbscan_eps = 0.3         # Maximum distance between two samples for one to be considered as in the neighborhood of the other
        self.dbscan_min_samples = 5   # The number of samples (or total weight) in a neighborhood for a point to be considered as a core point. 

        rospy.loginfo("Cone Detection GPU Node Initialized.")

        self.get_parameters()

    def pointcloud_callback(self, msg):
        rospy.loginfo("Received point cloud message.")
        # Convert ROS PointCloud2 to a PyTorch tensor on GPU
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points_list:
            rospy.logwarn("Received empty point cloud after NaN removal.")
            self.publish_cones(torch.tensor([], device=self.device), msg.header) # Publish empty if no cones
            return

        points_np = np.array(points_list, dtype=np.float32)
        points_tensor = torch.from_numpy(points_np).to(self.device)

        # Filter points close to the ground for cone base detection
        # Assuming the ground is around z=0. Adjust if your lidar frame has a different ground level.
        ground_points_mask = torch.abs(points_tensor[:, 2]) < self.ground_plane_z_threshold
        ground_points = points_tensor[ground_points_mask]

        if ground_points.shape[0] < 3:
            rospy.logwarn("Not enough ground points for RANSAC.")
            self.publish_cones(torch.tensor([], device=self.device), msg.header) # Publish empty if no cones
            return

        # Project 3D ground points to 2D (x, y) for circle RANSAC
        points_2d = ground_points[:, :2]

        # Perform RANSAC for circle detection on GPU
        best_circle_params, best_inliers_2d = self.ransac_gpu(
            points_2d, self.ransac_threshold, self.ransac_max_iterations,
            self.cone_min_radius, self.cone_max_radius
        )

        detected_cone_centroids = torch.tensor([], device=self.device)
        if best_circle_params is not None and best_inliers_2d.shape[0] > 0:
            cx, cy, r = best_circle_params
            rospy.loginfo(f"RANSAC found cone candidate: Center=({cx:.2f}, {cy:.2f}), Radius={r:.2f} with {best_inliers_2d.shape[0]} inliers.")
            
            # Re-evaluate inliers in 3D based on the detected 2D circle
            distances_to_center_3d = torch.sqrt((ground_points[:, 0] - cx)**2 + (ground_points[:, 1] - cy)**2)
            distances_to_circle_3d = torch.abs(distances_to_center_3d - r)
            inliers_mask_3d = distances_to_circle_3d < self.ransac_threshold
            ransac_inliers_3d = ground_points[inliers_mask_3d]

            if ransac_inliers_3d.shape[0] > 0:
                # Apply DBSCAN to the RANSAC inliers to cluster individual cones
                labels = self.lidar_util.cluster_points(ransac_inliers_3d, self.dbscan_eps, self.dbscan_min_samples)
                
                unique_labels = labels.unique()
                for label in unique_labels:
                    if label == -1: # Noise points
                        continue
                    
                    cluster_points = ransac_inliers_3d[labels == label]
                    if cluster_points.shape[0] >= self.dbscan_min_samples: # Only consider clusters with enough points
                        centroid = torch.mean(cluster_points, dim=0)
                        # We can also refine the cone position/radius here if needed
                        detected_cone_centroids = torch.cat((detected_cone_centroids, centroid.unsqueeze(0)), dim=0)

        self.publish_cones(detected_cone_centroids, msg.header)
        
    def init(self):
        """Initialize the system"""
        self.is_initialized = True
        return True

    def get_parameters(self):
        """Get parameters from ROS parameter server"""
        self.x_min, self.x_max = rospy.get_param("/perception/lidar_roi_extraction/x_min", -10.0) , rospy.get_param("/perception/lidar_roi_extraction/x_max", 10.0)
        self.y_min, self.y_max = rospy.get_param("/perception/lidar_roi_extraction/y_min", -10.0) , rospy.get_param("/perception/lidar_roi_extraction/y_max", 10.0)
        self.z_min, self.z_max = rospy.get_param("/perception/lidar_roi_extraction/z_min", -2.0) , rospy.get_param("/perception/lidar_roi_extraction/z_max", 2.0)
        self.ransac_iter = rospy.get_param("/perception/lidar_ground_removal/ransac_iterations", 100)
        self.ransac_distance = rospy.get_param("/perception/lidar_ground_removal/ransac_distance_threshold", 0.1)
        self.dbscan_eps = rospy.get_param("/perception/lidar_clustering/dbscan_eps", 0.5)
        self.dbscan_points = rospy.get_param("/perception/lidar_clustering/dbscan_min_points", 5)
        self.left_tx, self.left_ty, self.left_tz = rospy.get_param("/perception/camera_extrinsics/translation_x", 0.0), rospy.get_param("/perception/camera_extrinsics/translation_y", 0.0),rospy.get_param("/perception/camera_extrinsics/translation_z", 0.0)
        self.left_rr, self.left_rp, self.left_ry = rospy.get_param("/perception/camera_extrinsics/rotation_roll", 0.0), rospy.get_param("/perception/camera_extrinsics/rotation_pitch", 0.0), rospy.get_param("/perception/camera_extrinsics/rotation_yaw", 0.0)
        self.right_tx, self.right_ty, self.right_tz = rospy.get_param("/perception/camera_right_extrinsics/translation_x", 0.0), rospy.get_param("/perception/camera_right_extrinsics/translation_y", 0.0),rospy.get_param("/perception/camera_right_extrinsics/translation_z", 0.0)
        self.right_rr, self.right_rp, self.right_ry = rospy.get_param("/perception/camera_right_extrinsics/rotation_roll", 0.0), rospy.get_param("/perception/camera_right_extrinsics/rotation_pitch", 0.0), rospy.get_param("/perception/camera_right_extrinsics/rotation_yaw", 0.0)
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

        # rospy.loginfo_throttle(0.001,f"v = {round(math.sqrt(vehicle_state[3]**2 + vehicle_state[4]**2),4)} m/s")
        ## LiDAR Processed
        points=self.get_lidar_point_cloud(lidar_msg)
        filtered = self.lidar_util.filtering_points(points, (self.x_min, self.x_max), (self.y_min, self.y_max), (self.z_min, self.z_max))
        removal =  self.lidar_util.ransac_plane_removal(filtered, threshold=self.ransac_distance, max_trials=self.ransac_iter)
        cluster = self.lidar_util.cluster_points(removal, eps=self.dbscan_eps, min_samples=self.dbscan_points)
       
        image1 = self.get_camera_image(camera1_msg)
        image2 = self.get_camera_image(camera2_msg)
        cam_mat = self.camera_util.cam_matrix()
        cam1_transform = self.camera_util.transform_matrix(-self.left_tx, -self.left_ty, -self.left_tz, self.left_rr, self.left_rp, self.left_ry)
        cam2_transform = self.camera_util.transform_matrix(-self.right_tx, -self.right_ty, -self.right_tz, self.right_rr, self.right_rp, self.right_ry)
        image1 = self.camera_util.preprocessImage(image1)
        image2 = self.camera_util.preprocessImage(image2)
        
        # Process image1
        img1_rgb = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)
        img1_tensor = torch.from_numpy(img1_rgb).permute(2, 0, 1).float() / 255.0

        # Process image2
        img2_rgb = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)
        img2_tensor = torch.from_numpy(img2_rgb).permute(2, 0, 1).float() / 255.0

        # Combine into a batch for inference
        batched_input = [img1_tensor.to(self.device), img2_tensor.to(self.device)]
        
        # If the model is in FP16, convert input tensors to FP16 as well
        if self.device.type == 'cuda' and next(self.model.parameters()).is_cuda and next(self.model.parameters()).dtype == torch.float16:
            batched_input = [img.half() for img in batched_input]

        # Get predictions from the model
        with torch.no_grad():
            raw_predictions = self.model(batched_input)  # Returns List[List[Dict]] for batched input

        raw_predictions1 = [raw_predictions[0]] # Extract predictions for image1
        raw_predictions2 = [raw_predictions[1]] # Extract predictions for image2

        rendered_img1, left_bbox, left_conf = self.camera_util._process_and_draw_detections(image1, raw_predictions1)
        rendered_img2, right_bbox, right_conf = self.camera_util._process_and_draw_detections(image2, raw_predictions2)
        
        if cluster.numel() > 0:
            try:
                dt_cam_lidar = camera1_msg.header.stamp.to_sec() - lidar_msg.header.stamp.to_sec()
                vx = vehicle_state[3] # Longitudinal velocity
                vy = vehicle_state[4] # Lateral velocity
                yaw_rate = vehicle_state[5] # Yaw rate
                ax = vehicle_state[6] # Longitudinal acceleration
                ay = vehicle_state[7] # Lateral acceleration

                # Calculate translational compensation using 2nd order model
                compensation_x = vx * dt_cam_lidar + 0.5 * ax * dt_cam_lidar**2
                compensation_y = vy * dt_cam_lidar + 0.5 * ay * dt_cam_lidar**2

                compensated_cluster = cluster.clone() # Use clone to avoid modifying original tensor

                # Apply translational compensation
                compensated_cluster[:, 0] += compensation_x
                compensated_cluster[:, 1] += compensation_y

                # Apply rotational compensation (rotate points around vehicle's current position)
                if abs(yaw_rate) > 1e-6: # Only rotate if there's significant yaw rate
                    angle_compensation = yaw_rate * dt_cam_lidar
                    cos_angle = torch.cos(torch.tensor(angle_compensation, device=self.device))
                    sin_angle = torch.sin(torch.tensor(angle_compensation, device=self.device))

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
        cam1_pts = self.camera_util.projectToCam(compensated_cluster, cam1_transform)
        cam2_pts = self.camera_util.projectToCam(compensated_cluster, cam2_transform)

        if cluster.numel() > 0:
            # Convert compensated_cluster to numpy for OpenCV-based color detection
            compensated_cluster_np = compensated_cluster.cpu().numpy()

            # Transform cluster points to map frame
            veh_x = vehicle_state[0]
            veh_y = vehicle_state[1]
            veh_yaw = vehicle_state[2]  # Assumes radians

            cos_yaw = math.cos(veh_yaw)
            sin_yaw = math.sin(veh_yaw)
            rot_matrix_torch = torch.tensor([[cos_yaw, -sin_yaw],
                                             [sin_yaw,  cos_yaw]], dtype=torch.float32, device=self.device)
            # Prepare a list to store cones with color information
            cones_with_color = []
            left, right = [], []

            # Iterate through each 3D cluster point and determine its color
            for i, cone_3d_veh_frame_np in enumerate(compensated_cluster_np):
                detected_color = "unknown"
                
                # --- IoU-based Association for Camera 1 ---
                best_iou1 = -1
                best_bbox_index1 = -1
                if i < len(cam1_pts) and left_bbox is not None and len(left_bbox) > 0:
                    u, v = cam1_pts[i]
                    half_size = self.camera_util.hsv_window_base_size // 2
                    lidar_window1 = [u - half_size, v - half_size, u + half_size, v + half_size]
                    
                    for j, cam_bbox in enumerate(left_bbox):
                        iou = self.camera_util.calculate_iou(lidar_window1, cam_bbox)
                        if iou > best_iou1:
                            best_iou1 = iou
                            best_bbox_index1 = j
                
                # --- IoU-based Association for Camera 2 ---
                best_iou2 = -1
                best_bbox_index2 = -1
                if i < len(cam2_pts) and right_bbox is not None and len(right_bbox) > 0:
                    u, v = cam2_pts[i]
                    half_size = self.camera_util.hsv_window_base_size // 2
                    lidar_window2 = [u - half_size, v - half_size, u + half_size, v + half_size]

                    for j, cam_bbox in enumerate(right_bbox):
                        iou = self.camera_util.calculate_iou(lidar_window2, cam_bbox)
                        if iou > best_iou2:
                            best_iou2 = iou
                            best_bbox_index2 = j

                # --- Determine Color based on best IoU match ---
                # Prioritize camera 1 if both have a good match
                if best_iou1 > self.camera_util.lidar_cam_iou_threshold:
                    matched_bbox = left_bbox[best_bbox_index1]
                    detected_color = self.camera_util.detect_color_from_bbox(image1, matched_bbox, debug_image=rendered_img1)
                elif best_iou2 > self.camera_util.lidar_cam_iou_threshold:
                    matched_bbox = right_bbox[best_bbox_index2]
                    detected_color = self.camera_util.detect_color_from_bbox(image2, matched_bbox, debug_image=rendered_img2)

                # --- Fallback to Point-based Color Detection if IoU fails ---
                if detected_color == "unknown":
                    distance = np.linalg.norm(cone_3d_veh_frame_np[:2]) # Calculate distance using numpy for now
                    # Try with camera 1
                    if i < len(cam1_pts):
                        point_color = self.camera_util.detectConeColor(cam1_pts[i], distance, image1, debug_image=rendered_img1)
                        if point_color != "unknown":
                            detected_color = point_color
                    # If still unknown, try with camera 2
                    if detected_color == "unknown" and i < len(cam2_pts):
                        point_color = self.camera_util.detectConeColor(cam2_pts[i], distance, image2, debug_image=rendered_img2)
                        if point_color != "unknown":
                            detected_color = point_color

                # --- Color ID assignment ---
                color_id = 0  # Default to unknown
                if detected_color == "blue":
                    color_id = 1
                elif detected_color == "yellow":
                    color_id = 2
                elif detected_color == "orange":
                    color_id = 3
                
                # Transform 3D cone from vehicle frame to map frame using torch
                cone_3d_veh_frame_tensor = torch.tensor(cone_3d_veh_frame_np, dtype=torch.float32, device=self.device)
                cone_3d_map_frame_xy_tensor = torch.matmul(cone_3d_veh_frame_tensor[:2], rot_matrix_torch.T) + torch.tensor([veh_x, veh_y], device=self.device)
                cone_3d_map_frame_z = cone_3d_veh_frame_np[2]
                
                cones_with_color.append([cone_3d_map_frame_xy_tensor[0].item(), cone_3d_map_frame_xy_tensor[1].item(), cone_3d_map_frame_z, color_id])
                if color_id == 1:
                    left.append([cone_3d_map_frame_xy_tensor[0].item(), cone_3d_map_frame_xy_tensor[1].item()])
                elif color_id == 2:
                    right.append([cone_3d_map_frame_xy_tensor[0].item(), cone_3d_map_frame_xy_tensor[1].item()])
            
            global_clusters = torch.tensor(cones_with_color, dtype=torch.float32, device=self.device)
            rospy.loginfo_throttle(1.0, f"FormulaAutonomousSystem: Cones from perception: {len(global_clusters)}")
        else:
            global_clusters = torch.empty((0, 4), dtype=torch.float32, device=self.device)

        # ==================== Map & Path ===================
        # Update map with new cone observations
        self.track_map.update(global_clusters, vehicle_state)
        rospy.loginfo_throttle(1.0, f"FormulaAutonomousSystem: Cones in TrackMap: {len(self.track_map.get_cones())}")

        # Check if we should replan
        should_replan = False
        if self.current_path is None or len(self.current_path) < 2 or self.current_path_is_fallback:
            should_replan = True
        else:
            # Check distance to the end of the path
            car_pos = vehicle_state[:2]
            path_end_pos = self.current_path[-1]
            dist_to_end = np.linalg.norm(car_pos - path_end_pos)
            if dist_to_end < self.path_end_threshold:
                should_replan = True

        if should_replan:
            rospy.loginfo_throttle(1.0, "Replanning path.")
            # Plan path using the map
            path, tri, tri_points, tri_colors, midpoints_tensor, is_fallback, num_blue_cones, num_yellow_cones, num_midpoints = self.path_planner.plan_path(self.track_map.get_cones(), vehicle_state)
            
            # If a valid path is generated, store it.
            if path is not None and not is_fallback:
                self.current_path = path
                self.current_path_is_fallback = is_fallback
            # If a fallback path is generated, use it for this cycle but don't store it, so we replan on the next cycle.
            elif path is not None and is_fallback:
                self.current_path = None
                self.current_path_is_fallback = True

        else:
            rospy.loginfo_throttle(1.0, "Following existing path.")
            # Use the existing path
            path = self.current_path
            is_fallback = self.current_path_is_fallback
            # These values are not used when not replanning, so we can set them to default values
            tri, tri_points, tri_colors, midpoints_tensor, num_blue_cones, num_yellow_cones, num_midpoints = None, None, None, None, 0, 0, 0
        
        # Update midpoint map with new midpoints
        self.midpoint_map.update(midpoints_tensor)

        # Visualize Map and Path
        self.publish_map_cones()
        self.publish_triangulation(tri, tri_points, tri_colors)
        self.publish_midpoints(midpoints_tensor.cpu().numpy() if midpoints_tensor is not None else None)
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
            throttle, steer, brake, debug_data = self.controller.compute_control(vehicle_state, path, is_fallback)
            control_command_msg.throttle = throttle
            control_command_msg.steering = steer
            control_command_msg.brake = brake
        else:
            # If not driving or no path, apply brakes and zero throttle/steering
            throttle, steer, brake, debug_data = 0.0, 0.0, 0.5, {}
            control_command_msg.throttle = throttle
            control_command_msg.steering = steer
            control_command_msg.brake = brake
        # print(go_signal_msg)


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
                lidar_points=global_clusters.cpu().numpy() if global_clusters is not None else np.empty((0,4)),
                map_cones=self.track_map.get_cones(),
                path=path,
                control_debug_data=debug_data,
                is_fallback=is_fallback,
                num_blue_cones=num_blue_cones,
                num_yellow_cones=num_yellow_cones,
                num_midpoints=num_midpoints
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

    def publish_path(self, path_np):
        if not self.enable_visualization:
            return
        if self.visualization_frame_counter % self.visualization_publish_interval != 0:
            return

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for point in path_np:
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
        for i, point in enumerate(path_np):
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
        for i in range(len(path_np), self.last_path_index_count):
            marker = Marker()
            marker.header = header
            marker.ns = "path_indices"
            marker.id = i
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self.last_path_index_count = len(path_np)
        if len(marker_array.markers) > 0:
            self.path_index_publisher.publish(marker_array)

        # Publish markers for indices
        marker_array = MarkerArray()
        header = path_msg.header
        
        # Add text markers for each path point
        for i, point in enumerate(path_np):
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
        for i in range(len(path_np), self.last_path_index_count):
            marker = Marker()
            marker.header = header
            marker.ns = "path_indices"
            marker.id = i
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self.last_path_index_count = len(path_np)
        if len(marker_array.markers) > 0:
            self.path_index_publisher.publish(marker_array)

    def publish_midpoints(self, midpoints_np):
        if not self.enable_visualization:
            return
        if self.visualization_frame_counter % self.visualization_publish_interval != 0:
            return

        marker_array = MarkerArray()
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        if midpoints_np is not None:
            for i, point in enumerate(midpoints_np):
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
        for i in range(len(midpoints_np) if midpoints_np is not None else 0, self.last_midpoints_count):
            marker = Marker()
            marker.header = header
            marker.ns = "cone_midpoints"
            marker.id = i
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
        
        self.last_midpoints_count = len(midpoints_np) if midpoints_np is not None else 0
        self.midpoints_publisher.publish(marker_array)

    def publish_triangulation(self, tri, points_np, colors_np):
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

        if self.visualization_frame_counter % self.visualization_publish_interval != 0:
            # Clear previous markers if triangulation is not available or not publishing this frame
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"
            marker.ns = "delaunay_mesh"
            marker.id = 0
            marker.action = Marker.DELETEALL
            self.triangulation_publisher.publish(marker)
            return

        if tri is None or points_np is None or colors_np is None:
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
                
                p1_coords = points_np[p1_idx]
                p2_coords = points_np[p2_idx]

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

                    color1 = colors_np[p1_idx]
                    color2 = colors_np[p2_idx]

                    line_color = gray
                    if color1 == 1 and color2 == 1: # Blue
                        line_color = blue
                    elif color1 == 2 and color2 == 2: # Yellow
                        line_color = yellow
                    
                    marker.colors.append(line_color)
                    marker.colors.append(line_color)
                
        self.triangulation_publisher.publish(marker)

    def get_lidar_point_cloud(self, msg):
        """Convert ROS PointCloud2 message to point cloud (torch.Tensor)"""
        pointcloud_list = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pointcloud_list.append([point[0], point[1], point[2]])
        
        if not pointcloud_list:
            return torch.empty((0, 3), dtype=torch.float32, device=self.device)
            
        return torch.tensor(pointcloud_list, dtype=torch.float32, device=self.device)

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
        
        self.hsv_window_base_size = rospy.get_param("/perception/camera_hsv_window_size/base_size", 20)
        self.hsv_window_distance_factor = rospy.get_param("/perception/camera_hsv_window_size/distance_factor", 0.6)

        self.hsv_yellow_min = np.array([rospy.get_param("/perception/camera_hsv_yellow/hue_min"), rospy.get_param("/perception/camera_hsv_yellow/saturation_min"), rospy.get_param("/perception/camera_hsv_yellow/value_min")])
        self.hsv_yellow_max = np.array([rospy.get_param("/perception/camera_hsv_yellow/hue_max"), 255, 255])

        self.hsv_blue_min = np.array([rospy.get_param("/perception/camera_hsv_blue/hue_min"), rospy.get_param("/perception/camera_hsv_blue/saturation_min"), rospy.get_param("/perception/camera_hsv_blue/value_min")])
        self.hsv_blue_max = np.array([rospy.get_param("/perception/camera_hsv_blue/hue_max"), 255, 255])

        self.hsv_orange_min = np.array([rospy.get_param("/perception/camera_hsv_orange/hue_min"), rospy.get_param("/perception/camera_hsv_orange/saturation_min"), rospy.get_param("/perception/camera_hsv_orange/value_min")])
        self.hsv_orange_max = np.array([rospy.get_param("/perception/camera_hsv_orange/hue_max"), 255, 255])

        self.min_bbox_area = rospy.get_param("/perception/camera_cone_detection/min_bbox_area", 100)
        self.visualize_lidar_on_camera = rospy.get_param("/perception/camera_cone_detection/visualize_lidar_on_camera", False)
        self.lidar_cam_iou_threshold = rospy.get_param("/perception/camera_cone_detection/lidar_cam_iou_threshold", 0.5)
        self.color_score_separation_threshold = rospy.get_param("/perception/camera_cone_detection/color_score_separation_threshold", 0.1)

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
    
    def projectToCam(self, points: torch.Tensor, transform: list) -> list:
        if points.numel() == 0:
            return []

        T = torch.tensor(transform, dtype=torch.float32, device=points.device)
        rotation = T[:3, :3]
        translation = T[:3, 3]

        # Apply the transformation to all points at once
        # points is (N, 3), rotation is (3, 3), translation is (3,)
        cone_points_in_cam = torch.matmul(points, rotation.T) + translation

        # Filter points in front of the camera (ROS X-axis points forward)
        valid_mask = cone_points_in_cam[:, 0] > 0
        cone_points_in_cam_filtered = cone_points_in_cam[valid_mask]

        if cone_points_in_cam_filtered.numel() == 0:
            return []

        # Convert from ROS camera coordinates to standard CV/image coordinates
        z_cv = cone_points_in_cam_filtered[:, 0]  # ROS X -> CV Z
        x_cv = -cone_points_in_cam_filtered[:, 1] # ROS Y -> CV X
        y_cv = -cone_points_in_cam_filtered[:, 2] # ROS Z -> CV Y

        # Perform pinhole projection
        u = self.fx * (x_cv / z_cv) + self.px
        v = self.fy * (y_cv / z_cv) + self.py
        
        # Combine u and v into a list of tuples (converting back to CPU numpy for cv2 operations later)
        projected_points = torch.stack((u, v), dim=1).cpu().numpy().tolist()
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
    
    def detectConeColor(self, cone_point_img, distance, rgb_image, debug_image=None):
        # cone_point_img is expected to be a single (u, v) tuple or list
        if cone_point_img is None or not isinstance(cone_point_img, (tuple, list)) or len(cone_point_img) != 2:
            return "unknown"
        # cv2.imshow("debug", cv2.cvtColor(rgb_image,cv2.COLOR_BGR2HSV))
        u, v = int(cone_point_img[0]), int(cone_point_img[1])
        
        # Define ROI around the cone with dynamic size (inversely proportional to distance)
        half_window = max(3, (self.hsv_window_base_size // 2) - int(distance * self.hsv_window_distance_factor)) # Ensure minimum half_window of 3
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

        cv2.imshow("lidar roi", np.concatenate((hsv_roi,roi),axis=1))
        # cv2.imshow("masks", np.concatenate((mask_yellow,mask_blue,mask_orange),axis=1))

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
        if not color_scores:
            dominant_color = "unknown"
        else:
            # Sort colors by score in descending order
            sorted_colors = sorted(color_scores.items(), key=lambda item: item[1], reverse=True)
            
            best_color, best_score = sorted_colors[0]

            if best_score > 0:
                dominant_color = best_color
                # Check for score separation if there is a second color
                if len(sorted_colors) > 1 and sorted_colors[1][1] > 0:
                    second_best_score = sorted_colors[1][1]
                    if best_score < second_best_score * self.color_score_separation_threshold:
                        dominant_color = "unknown" # Scores are too close, classification is ambiguous
            else:
                dominant_color = "unknown"

        # 최소 픽셀 수 임계값 (노이즈 제거)
        if dominant_color != "unknown":
            # 점수 계산에 사용된 픽셀 수가 전체 ROI의 5% 미만이면 노이즈로 간주
            # print(f"LiDAR - count from mask = {cv2.countNonZero(masks[dominant_color])}, limit = {roi.size * 0.05}")
            if cv2.countNonZero(masks[dominant_color]) < (roi.size * 0.02):
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

            if area < (self.hsv_window_base_size//2)**2 or width > height:
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

        # print(f"bbox = {bbox}")

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
        cv2.imshow("hsv", np.concatenate((hsv_roi,roi),axis=1))
        # cv2.imshow("masks", np.concatenate((mask_yellow,mask_blue,mask_orange),axis=1))
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
        if not color_scores:
            dominant_color = "unknown"
        else:
            # Sort colors by score in descending order
            sorted_colors = sorted(color_scores.items(), key=lambda item: item[1], reverse=True)
            
            best_color, best_score = sorted_colors[0]

            if best_score > 0:
                dominant_color = best_color
                # Check for score separation if there is a second color
                if len(sorted_colors) > 1 and sorted_colors[1][1] > 0:
                    second_best_score = sorted_colors[1][1]
                    if best_score < second_best_score * self.color_score_separation_threshold:
                        dominant_color = "unknown" # Scores are too close, classification is ambiguous
            else:
                dominant_color = "unknown"

        # 최소 픽셀 수 임계값 (노이즈 제거)
        if dominant_color != "unknown":
            # print(f"bbox - count from mask = {cv2.countNonZero(masks[dominant_color])}, limit = {roi.size * 0.05}")
            if cv2.countNonZero(masks[dominant_color]) < (roi.size * 0.02):
                 dominant_color = "unknown"

        if debug_image is not None and dominant_color != "unknown":
            cv2.putText(debug_image, dominant_color, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            # print(f"from bbox - {color_scores}, color = {dominant_color} \n")

    
        return dominant_color
        

    def calculate_iou(self, boxA, boxB):
        # box format: [x1, y1, x2, y2]
        # Determine the (x, y)-coordinates of the intersection rectangle
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])

        # Compute the area of intersection rectangle
        interArea = max(0, xB - xA) * max(0, yB - yA)

        # Compute the area of both the prediction and ground-truth rectangles
        boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
        boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

        # Compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        iou = interArea / float(boxAArea + boxBArea - interArea + 1e-6) # Add epsilon to avoid division by zero

        # Return the intersection over union value
        return iou

class LiDARProcessor:
    def __init__(self, enable_visualization=True, device=None):
        self.device = device if device is not None else torch.device('cpu')
        self.enable_visualization = enable_visualization
        self.lidar_publisher = rospy.Publisher("/processed_lidar", PointCloud2, queue_size=1)
        self.marker_publisher = rospy.Publisher("/cluster_indices", MarkerArray, queue_size=1)
        self.last_marker_count = 0
        self.x_min, self.x_max = rospy.get_param("/perception/lidar_roi_extraction/x_min", -10.0) , rospy.get_param("/perception/lidar_roi_extraction/x_max", 10.0)
        self.y_min, self.y_max = rospy.get_param("/perception/lidar_roi_extraction/y_min", -10.0) , rospy.get_param("/perception/lidar_roi_extraction/y_max", 10.0)
        self.z_min, self.z_max = rospy.get_param("/perception/lidar_roi_extraction/z_min", -2.0) , rospy.get_param("/perception/lidar_roi_extraction/z_max", 2.0)
        self.ransac_iter = rospy.get_param("/perception/lidar_ground_removal/ransac_iterations", 100)
        self.ransac_distance = rospy.get_param("/perception/lidar_ground_removal/ransac_distance_threshold", 0.1)
        self.dbscan_eps = rospy.get_param("/perception/lidar_clustering/dbscan_eps", 0.5)
        self.dbscan_points = rospy.get_param("/perception/lidar_clustering/dbscan_min_points", 5)

        self.trans_x,self.trans_y,self.trans_z = rospy.get_param("/perception/lidar_extrinsics/translation_x", 0.0), rospy.get_param("/perception/lidar_extrinsics/translation_y", 0.0), rospy.get_param("/perception/lidar_extrinsics/translation_z", 0.0)
        self.rot_r, self.rot_p, self.rot_yaw = rospy.get_param("/perception/lidar_extrinsics/rotation_roll", 0.0), rospy.get_param("/perception/lidar_extrinsics/rotation_pitch", 0.0), rospy.get_param("/perception/lidar_extrinsics/rotation_yaw", 0.0)
        # self.gps_util = GPSIMUProcessor()
    def vehicle_to_lidar_Transform(self, x, y, z, r, p, yaw):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(yaw), math.sin(yaw)

        T = torch.tensor([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, x],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, y],
            [-sp,   cp*sr,            cp*cr,            z],
            [0,     0,                0,                1]
        ], dtype=torch.float32, device=self.device)
        return T

    def filtering_points(self, points: torch.Tensor, x_range: Tuple[float, float], y_range: Tuple[float, float], z_range: Tuple[float, float]) -> torch.Tensor:
        """Filter points within specified ranges using torch.Tensor"""
        if points.numel() == 0:
            return points

        mask = (
            (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
            (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
            (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
        )
        return points[mask]
    
    def ransac_plane_removal(self, points: torch.Tensor, threshold: float = 0.05, max_trials: int = 100) -> torch.Tensor:
        """Remove ground plane using Open3D's RANSAC"""
        if points.numel() == 0 or points.shape[0] < 3:
            return torch.empty((0, 3), dtype=torch.float32, device=self.device)

        # Convert torch.Tensor to Open3D PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.cpu().numpy())

        # Perform RANSAC plane segmentation
        plane_model, inliers = pcd.segment_plane(distance_threshold=threshold, ransac_n=3, num_iterations=max_trials)

        # Select the outlier points (non-ground)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

        # Convert back to torch.Tensor and move to device
        object_points_np = np.asarray(outlier_cloud.points)
        return torch.tensor(object_points_np, dtype=torch.float32, device=self.device)

    def cluster_points(self, points: torch.Tensor, eps: float = 0.5, min_samples: int = 5) -> torch.Tensor:
        """Cluster points using Open3D's DBSCAN and return centroids."""
        if points.numel() == 0 or points.shape[0] < min_samples:
            return torch.empty((0, 3), dtype=torch.float32, device=self.device)

        # Convert torch.Tensor to Open3D PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.cpu().numpy())

        # Perform DBSCAN clustering
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_samples, print_progress=False))

        # Get unique labels, excluding noise (-1)
        unique_labels = np.unique(labels)
        cluster_centroids = []

        for label in unique_labels:
            if label == -1:
                continue

            # Get points belonging to the current cluster
            cluster_indices = np.where(labels == label)[0]
            cluster_points = points[cluster_indices]

            # Compute the centroid of the cluster
            centroid = torch.mean(cluster_points, dim=0)
            cluster_centroids.append(centroid)

        if not cluster_centroids:
            return torch.empty((0, 3), dtype=torch.float32, device=self.device)

        # Stack centroids into a single tensor
        return torch.stack(cluster_centroids)
    
    def ransac_gpu(self, points_2d, threshold, max_iterations, min_radius, max_radius):
        """
        GPU-accelerated RANSAC for detecting circular cone bases in 2D projected point clouds.
        This implementation uses a more numerically stable method for circle fitting.
        points_2d: Nx2 tensor of (x, y) coordinates.
        threshold: Maximum distance for a point to be considered an inlier.
        max_iterations: Number of RANSAC iterations.
        min_radius, max_radius: Expected radius range for cone bases.
        Returns: best_circle_params (cx, cy, r), best_inliers (Nx2 tensor).
        """
        num_points = points_2d.shape[0]
        if num_points < 3:
            return None, torch.tensor([], device=self.device)

        best_inlier_count = 0
        best_circle_params = None
        best_inliers = torch.tensor([], device=self.device)

        for _ in range(max_iterations):
            # Randomly sample 3 points
            sample_indices = torch.randint(0, num_points, (3,), device=self.device)
            p1, p2, p3 = points_2d[sample_indices]

            # Using a more stable method to find the circumcenter of a triangle
            # based on solving a system of linear equations.
            # The general equation of a circle is x^2 + y^2 + Ax + By + C = 0
            # We can solve for A, B, C using the three points.
            try:
                A = torch.tensor([[p1[0], p1[1], 1],
                                  [p2[0], p2[1], 1],
                                  [p3[0], p3[1], 1]], device=self.device)
                
                b = -torch.tensor([p1[0]**2 + p1[1]**2,
                                    p2[0]**2 + p2[1]**2,
                                    p3[0]**2 + p3[1]**2], device=self.device)

                # Solve the linear system Ax = b
                # Use torch.linalg.solve for better numerical stability
                x = torch.linalg.solve(A, b)
                
                # Circle parameters
                cx = -0.5 * x[0]
                cy = -0.5 * x[1]
                r_sq = cx**2 + cy**2 - x[2]

                if r_sq <= 0:
                    continue
                
                r = torch.sqrt(r_sq)

            except torch.linalg.LinAlgError:
                # This can happen if the points are collinear, leading to a singular matrix
                continue

            # Filter by radius constraints
            if not (min_radius <= r <= max_radius):
                continue

            # Calculate distances from all points to the circle
            distances_to_center = torch.sqrt((points_2d[:, 0] - cx)**2 + (points_2d[:, 1] - cy)**2)
            distances_to_circle = torch.abs(distances_to_center - r)

            # Find inliers
            inliers_mask = distances_to_circle < threshold
            current_inliers = points_2d[inliers_mask]
            current_inlier_count = current_inliers.shape[0]

            if current_inlier_count > best_inlier_count:
                best_inlier_count = current_inlier_count
                best_circle_params = (cx, cy, r)
                best_inliers = current_inliers

        return best_circle_params, best_inliers
    
        

    def left_right_split(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Split points into left and right based on y-coordinate"""
        left_points = points[points[:, 1] > 0]
        right_points = points[points[:, 1] <= 0]
        return left_points, right_points
    
    ### For Debugging: Publish Processed Point Cloud ###
    def publish_point_cloud(self, points: torch.Tensor):
        """Publish processed point cloud and their indices as markers"""
        if not self.enable_visualization:
            return

        clusters_np = points.cpu().numpy() # Convert to numpy for ROS message

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "odom"
        
        # Publish point cloud
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        point_cloud_msg = pc2.create_cloud(header, fields, clusters_np)
        self.lidar_publisher.publish(point_cloud_msg)

        # Publish markers for indices
        marker_array = MarkerArray()
        
        # Add text markers for each cluster
        for i, point in enumerate(clusters_np):
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
        for i in range(len(clusters_np), self.last_marker_count):
            marker = Marker()
            marker.header = header
            marker.ns = "cluster_indices"
            marker.id = i
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self.last_marker_count = len(clusters_np)
        if len(marker_array.markers) > 0:
            self.marker_publisher.publish(marker_array)

# ==================== Map & Path Planner ====================

class TrackMap:
    def __init__(self, device):
        self.device = device
        self.cones = []
        self.next_cone_id = 0
        self.association_threshold = rospy.get_param("/mapping/association_threshold", 1.5)
        self.smoothing_alpha = rospy.get_param("/mapping/smoothing_alpha", 0.2) # 이전 값과 새 값의 가중치

        # --- Loop Closure Parameters ---
        self.is_loop_closed = False
        self.min_cones_for_lc = rospy.get_param("/mapping/lc/min_cones", 20) # Temporarily set to a very high value to disable loop closure
        self.lc_trigger_distance = rospy.get_param("/mapping/lc/trigger_distance", 8.0)
        self.lc_search_radius = rospy.get_param("/mapping/lc/search_radius", 15.0)
        self.lc_min_match_pairs = rospy.get_param("/mapping/lc/min_pairs", 4)
        self.lc_max_transform_error = rospy.get_param("/mapping/lc/max_error", 0.75)
        self.start_line_center = None
        self.update_count = 0
        self.lc_cooldown_period = rospy.get_param("/mapping/lc/cooldown_updates", 100) # Cooldown in number of updates

    def update(self, new_cones_observations: torch.Tensor, vehicle_state):
        """
        Updates the map with new cone observations, and attempts loop closure.
        """
        self.update_count += 1

        # 1. Find start line if not already found
        if self.start_line_center is None and new_cones_observations.numel() > 1:
            self._find_start_line(new_cones_observations)

        # 2. Attempt loop closure and get corrected observations
        # This will return transformed observations if LC is successful
        processed_observations = self._detect_and_correct_loop_closure(new_cones_observations, vehicle_state)

        # 3. Continue with original update logic
        if not self.cones:
            for cone_obs in processed_observations:
                self._add_new_cone(cone_obs)
            return

        if processed_observations.numel() == 0:
            return

        map_cone_positions = torch.tensor([[c['x'], c['y']] for c in self.cones], dtype=torch.float32, device=self.device)
        obs_cone_positions = processed_observations[:, :2]
        distance_matrix = torch.cdist(map_cone_positions, obs_cone_positions)

        matched_obs_indices = set()
        matched_map_indices = set()

        # 1단계: 위치와 색상이 모두 일치하는 경우 먼저 매칭 (가장 신뢰도 높은 매칭)
        for map_idx, map_cone in enumerate(self.cones):
            if map_idx in matched_map_indices:
                continue

            possible_matches_mask = (processed_observations[:, 3] == map_cone['color_id'])
            if torch.any(possible_matches_mask):
                distances_to_map_cone = distance_matrix[map_idx, possible_matches_mask]
                obs_indices_for_color = torch.where(possible_matches_mask)[0]
                
                if distances_to_map_cone.numel() > 0:
                    best_match_local_idx = torch.argmin(distances_to_map_cone)
                    min_dist = distances_to_map_cone[best_match_local_idx]

                    if min_dist < self.association_threshold:
                        obs_idx = obs_indices_for_color[best_match_local_idx]
                        if obs_idx.item() not in matched_obs_indices:
                            self._update_cone(map_idx, processed_observations[obs_idx.item()])
                            matched_obs_indices.add(obs_idx.item())
                            matched_map_indices.add(map_idx)

        # 2단계: 색상이 다르더라도 위치가 매우 가까운 경우, 색상 정보를 업데이트 (오탐지 교정)
        for map_idx, map_cone in enumerate(self.cones):
            if map_idx in matched_map_indices: continue

            # 아직 매칭되지 않은 관측값들 중에서 가장 가까운 것을 찾음
            unmatched_obs_indices = [i for i in range(len(processed_observations)) if i not in matched_obs_indices]
            if not unmatched_obs_indices: break

            distances_to_unmatched = distance_matrix[map_idx, unmatched_obs_indices]
            if distances_to_unmatched.numel() == 0:
                continue
            best_match_local_idx = torch.argmin(distances_to_unmatched)
            min_dist = distances_to_unmatched[best_match_local_idx]

            if min_dist < self.association_threshold:
                obs_idx = unmatched_obs_indices[best_match_local_idx]
                if obs_idx not in matched_obs_indices:
                    new_color_id = int(processed_observations[obs_idx][3].item())
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

    def _find_start_line(self, observations: torch.Tensor):
        blue_cones = observations[observations[:, 3] == 1]
        yellow_cones = observations[observations[:, 3] == 2]
        if blue_cones.numel() > 0 and yellow_cones.numel() > 0:
            avg_blue = torch.mean(blue_cones[:, :2], dim=0)
            avg_yellow = torch.mean(yellow_cones[:, :2], dim=0)
            self.start_line_center = ((avg_blue + avg_yellow) / 2.0).cpu().numpy() # Store as numpy array
            rospy.loginfo(f"TrackMap: Start line center established at {self.start_line_center}")

    def _detect_and_correct_loop_closure(self, new_observations: torch.Tensor, vehicle_state):
        # --- 1. Check Trigger Conditions ---
        if self.is_loop_closed or len(self.cones) < self.min_cones_for_lc or self.start_line_center is None or self.update_count < self.lc_cooldown_period:
            return new_observations

        car_pos = torch.tensor(vehicle_state[:2], dtype=torch.float32, device=self.device)
        start_line_center_tensor = torch.tensor(self.start_line_center, dtype=torch.float32, device=self.device)
        dist_to_start = torch.linalg.norm(car_pos - start_line_center_tensor)

        if dist_to_start > self.lc_trigger_distance:
            return new_observations

        rospy.loginfo_throttle(1.0, f"TrackMap: Loop closure check triggered (dist to start: {dist_to_start:.2f}m)")

        # --- 2. Find Candidate Cones for Matching ---
        map_cones_tensor = torch.tensor([[c['x'], c['y'], c['color_id']] for c in self.cones], dtype=torch.float32, device=self.device)
        
        # Reference cones: old cones from the map near the start line
        dist_from_start = torch.linalg.norm(map_cones_tensor[:, :2] - start_line_center_tensor, dim=1)
        reference_mask = dist_from_start < self.lc_search_radius
        reference_cones = map_cones_tensor[reference_mask]

        # Current cones: new observations near the car
        dist_from_car = torch.linalg.norm(new_observations[:, :2] - car_pos, dim=1)
        current_mask = dist_from_car < self.lc_search_radius
        current_cones = new_observations[current_mask]

        if len(reference_cones) < self.lc_min_match_pairs or len(current_cones) < self.lc_min_match_pairs:
            rospy.logwarn_throttle(1.0, "TrackMap: Not enough cones for loop closure matching.")
            return new_observations

        # --- 3. Find Matching Pairs ---
        src_pts, dst_pts = [], []
        # Use distance matrix between current and reference cones
        dist_matrix = torch.cdist(current_cones[:, :2], reference_cones[:, :2])
        
        for i, c_cone in enumerate(current_cones):
            # Find potential matches of the same color
            color_mask = reference_cones[:, 2] == c_cone[3]
            if not torch.any(color_mask):
                continue
            
            row = dist_matrix[i, color_mask]
            ref_indices = torch.where(color_mask)[0]

            if row.numel() == 0:
                continue

            best_ref_local_idx = torch.argmin(row)
            if row[best_ref_local_idx] < self.association_threshold:
                src_pts.append(c_cone[:2])
                dst_pts.append(reference_cones[ref_indices[best_ref_local_idx]][:2])

        if len(src_pts) < self.lc_min_match_pairs:
            rospy.logwarn_throttle(1.0, f"TrackMap: Found only {len(src_pts)} pairs for LC, need {self.lc_min_match_pairs}.")
            return new_observations

        # --- 4. Estimate and Verify Transform ---
        src_pts_np = torch.stack(src_pts).cpu().numpy().astype(np.float32)
        dst_pts_np = torch.stack(dst_pts).cpu().numpy().astype(np.float32)
        
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
        new_obs_pts_np = new_observations[:, :2].cpu().numpy().astype(np.float32)
        corrected_obs_pts_np = cv2.transform(new_obs_pts_np.reshape(-1, 1, 2), transform_matrix).reshape(-1, 2)
        
        corrected_observations = new_observations.clone()
        corrected_observations[:, :2] = torch.tensor(corrected_obs_pts_np, dtype=torch.float32, device=self.device)
        
        return corrected_observations

    def _add_new_cone(self, cone_obs: torch.Tensor):
        new_cone = {
            'id': self.next_cone_id,
            'x': cone_obs[0].item(),
            'y': cone_obs[1].item(),
            'z': cone_obs[2].item(),
            'color_id': int(cone_obs[3].item()),
            'covariance': torch.eye(2, device=self.device) * 0.5
        }
        self.cones.append(new_cone)
        self.next_cone_id += 1

    def _update_cone(self, map_idx, cone_obs: torch.Tensor, update_color=False):
        # Use exponential smoothing to update cone position
        alpha = self.smoothing_alpha
        self.cones[map_idx]['x'] = (1 - alpha) * self.cones[map_idx]['x'] + alpha * cone_obs[0].item()
        self.cones[map_idx]['y'] = (1 - alpha) * self.cones[map_idx]['y'] + alpha * cone_obs[1].item()
        if update_color:
            self.cones[map_idx]['color_id'] = int(cone_obs[3].item())

    def get_cones(self):
        return self.cones

class MidpointMap:
    def __init__(self, device):
        self.device = device
        self.midpoints = []
        self.next_midpoint_id = 0
        self.association_threshold = rospy.get_param("/planning/midpoint_map/association_threshold", 1.5)
        self.smoothing_alpha = rospy.get_param("/planning/midpoint_map/smoothing_alpha", 0.5)

    def update(self, new_midpoints_obs: torch.Tensor):
        if new_midpoints_obs is None or new_midpoints_obs.numel() == 0:
            return

        if not self.midpoints:
            for p in new_midpoints_obs:
                self._add_new_midpoint(p)
            return

        map_points = torch.tensor([[m['x'], m['y']] for m in self.midpoints], dtype=torch.float32, device=self.device)
        obs_points = new_midpoints_obs
        distance_matrix = torch.cdist(map_points, obs_points)

        matched_obs_indices = set()
        for map_idx, map_point in enumerate(self.midpoints):
            if distance_matrix.shape[1] == 0:
                break

            best_match_obs_idx = torch.argmin(distance_matrix[map_idx])
            min_dist = distance_matrix[map_idx, best_match_obs_idx]

            if min_dist < self.association_threshold:
                if best_match_obs_idx.item() not in matched_obs_indices:
                    self._update_midpoint(map_idx, obs_points[best_match_obs_idx.item()])
                    matched_obs_indices.add(best_match_obs_idx.item())
        
        for obs_idx, obs_point in enumerate(obs_points):
            if obs_idx not in matched_obs_indices:
                self._add_new_midpoint(obs_point)

    def _add_new_midpoint(self, point: torch.Tensor):
        new_midpoint = {
            'id': self.next_midpoint_id,
            'x': point[0].item(),
            'y': point[1].item(),
        }
        self.midpoints.append(new_midpoint)
        self.next_midpoint_id += 1

    def _update_midpoint(self, map_idx, obs_point: torch.Tensor):
        self.midpoints[map_idx]['x'] = (1 - self.smoothing_alpha) * self.midpoints[map_idx]['x'] + self.smoothing_alpha * obs_point[0].item()
        self.midpoints[map_idx]['y'] = (1 - self.smoothing_alpha) * self.midpoints[map_idx]['y'] + self.smoothing_alpha * obs_point[1].item()

    def get_all_midpoints(self):
        # Sort by id to maintain path order
        sorted_midpoints = sorted(self.midpoints, key=lambda p: p['id'])
        return [[p['x'], p['y']] for p in sorted_midpoints]

class PathPlanner:
<<<<<<< HEAD
    def __init__(self):
        self.path_planning_algorithm = rospy.get_param("/planning/path_planner/path_planning_algorithm", "Delaunay")
        self.max_edge_length = rospy.get_param("/planning/path_planner/max_edge_length", 50.0)
        self.spline_smoothing_factor = rospy.get_param("/planning/path_planner/spline_smoothing_factor", 0.5)
=======
    def __init__(self, device):
        self.device = device
        self.max_edge_length = rospy.get_param("/planning/path_planner/max_edge_length", 7.0)
        self.w_dist = rospy.get_param("/planning/path_planner/weight_dist", 0.3)
        self.w_angle = rospy.get_param("/planning/path_planner/weight_angle", 0.7)
        self.max_path_distance = rospy.get_param("/planning/path_planner/max_path_distance", 200.0)
        self.path_obstacle_threshold = rospy.get_param("/planning/path_planner/path_obstacle_threshold", 1.5)
        self.lane_offset = rospy.get_param("/local_planning/trajectory/lane_offset", 2.0)
        
        # --- 경로 계획기 관심 영역(ROI) 필터링 파라미터 ---
        self.planner_roi_distance = rospy.get_param("/local_planning/trajectory/planner_roi_distance", 30.0)
        self.planner_roi_angle_rad = math.radians(rospy.get_param("/local_planning/trajectory/planner_roi_angle", 90.0))
        self.planner_clustering_eps = rospy.get_param("/local_planning/trajectory/planner_clustering_eps", 8.0)
        
        # --- 경로 안정화를 위한 이동 평균 필터 추가 ---
        self.path_direction_history = deque(maxlen=3) # 최근 3개의 경로 방향 벡터를 저장
        self.path_direction_smoothing_factor = 0.6 # 새로운 방향 벡터에 대한 가중치

        self.fallback_path_length = rospy.get_param("/planning/path_planner/fallback_path_length", 15.0)
        self.fallback_path_points = rospy.get_param("/planning/path_planner/fallback_path_points", 10)

        # --- GPU Path Smoothing Parameters ---
        self.path_smoothing_kernel_size = rospy.get_param("/planning/path_planner/path_smoothing_kernel_size", 5)
        self.path_smoothing_sigma = rospy.get_param("/planning/path_planner/path_smoothing_sigma", 1.0)
>>>>>>> 5969398c7d0e79a39200146d11b230ba4be93326


    def _normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        if isinstance(angle, torch.Tensor):
            return torch.fmod(angle + torch.pi, 2.0 * torch.pi) - torch.pi
        else:
            while angle > math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi
            return angle

    def _angle_between_vectors(self, v1, v2):
        """Calculates the angle in radians between two vectors."""
        if isinstance(v1, np.ndarray):
            v1 = torch.tensor(v1, dtype=torch.float32, device=self.device)
            v2 = torch.tensor(v2, dtype=torch.float32, device=self.device)

        v1_u = v1 / (torch.linalg.norm(v1) + 1e-6)
        v2_u = v2 / (torch.linalg.norm(v2) + 1e-6)
        return torch.arccos(torch.clamp(torch.dot(v1_u, v2_u), -1.0, 1.0))

    def _generate_fallback_path(self, blue_cones, yellow_cones, car_pos, car_yaw):
        """
        Generates a simple, straight fallback path.
        """
        car_pos_tensor = torch.tensor(car_pos, dtype=torch.float32, device=self.device)
        car_heading_vec = torch.tensor([math.cos(car_yaw), math.sin(car_yaw)], dtype=torch.float32, device=self.device)

        rospy.logwarn_throttle(1.0, "PathPlanner: Generating straight fallback path.")
        point_distance = self.fallback_path_length / self.fallback_path_points
        path = [car_pos_tensor + car_heading_vec * i * point_distance for i in range(1, self.fallback_path_points + 1)]
        return torch.stack(path).cpu().numpy(), None, None, None, None, True
    def _sort_midpoints(self, midpoints, car_pos, car_yaw):
        """Sorts midpoints into a logical path, starting near the car and following the track's flow."""
        if len(midpoints) < 2:
            return midpoints

        midpoints_tensor = torch.tensor(midpoints, dtype=torch.float32, device=self.device)
        car_pos_tensor = torch.tensor(car_pos, dtype=torch.float32, device=self.device)
        
        # Find the best starting point: close and in front of the car
        start_idx = -1
        min_cost = float('inf')
        for i, p_tensor in enumerate(midpoints_tensor):
            dist = torch.hypot(p_tensor[0] - car_pos_tensor[0], p_tensor[1] - car_pos_tensor[1])
            angle_to_point = torch.atan2(p_tensor[1] - car_pos_tensor[1], p_tensor[0] - car_pos_tensor[0])
            angle_diff = self._normalize_angle(angle_to_point - car_yaw)
            
            if torch.abs(angle_diff) < (math.pi / 1.5): # Wider 120-degree arc
                cost = dist * (1 + torch.abs(angle_diff)) # Penalize points off to the side
                if cost < min_cost:
                    min_cost = cost
                    start_idx = i
        
        if start_idx == -1: # If no points are in the front arc, fall back to closest
            distances = torch.hypot(midpoints_tensor[:, 0] - car_pos_tensor[0], midpoints_tensor[:, 1] - car_pos_tensor[1])
            start_idx = torch.argmin(distances).item()

        ordered_path = [midpoints_tensor[start_idx]]
        remaining_midpoints = [midpoints_tensor[i] for i in range(len(midpoints_tensor)) if i != start_idx]

        # Establish initial direction with the second point
        if remaining_midpoints:
            last_point = ordered_path[-1]
            
            best_next_idx = -1
            min_cost = float('inf')
            for i, p_tensor in enumerate(remaining_midpoints):
                dist = torch.hypot(p_tensor[0] - last_point[0], p_tensor[1] - last_point[1])
                angle_to_point = torch.atan2(p_tensor[1] - last_point[1], p_tensor[0] - last_point[0])
                angle_diff = self._normalize_angle(angle_to_point - car_yaw) # Compare with car's yaw

                cost = dist * (1 + torch.abs(angle_diff))
                if cost < min_cost:
                    min_cost = cost
                    best_next_idx = i
            
            if best_next_idx != -1:
                ordered_path.append(remaining_midpoints.pop(best_next_idx))

        # Sort the rest based on a cost function of distance and angle
        while remaining_midpoints and len(ordered_path) >= 2:
            last_point = ordered_path[-1]
            
            # --- 경로 방향성 계산 로직 개선 ---
            # 직전 두 점이 아닌, 최근 경로의 전반적인 방향을 사용
            current_path_vec = last_point - ordered_path[-2]
            self.path_direction_history.append(current_path_vec / (torch.linalg.norm(current_path_vec) + 1e-6))
            
            # 이동 평균을 사용하여 부드러운 경로 방향 벡터 계산
            smooth_path_vec = torch.mean(torch.stack(list(self.path_direction_history)), dim=0)
            smooth_path_vec /= (torch.linalg.norm(smooth_path_vec) + 1e-6)
            
            best_candidate_idx = -1
            min_cost = float('inf')

            for i, candidate_point in enumerate(remaining_midpoints):
                dist = torch.linalg.norm(candidate_point - last_point)
                
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
                ordered_path.append(remaining_midpoints.pop(best_candidate_idx))
            else:
                break # No suitable point found
        
        return torch.stack(ordered_path).cpu().numpy()
    def _correct_path_detours(self, path: torch.Tensor, car_yaw):
        if path.numel() < 2:
            return path

        corrected_path = [path[0]]
        for i in range(len(path) - 1):
            p1 = corrected_path[-1]
            p2 = path[i+1]

            segment_vec = p2 - p1
            
            # Check if the segment is going backward relative to the car's yaw
            angle_to_car_yaw = self._normalize_angle(torch.atan2(segment_vec[1], segment_vec[0]) - car_yaw)
            
            if torch.abs(angle_to_car_yaw) > (math.pi / 2.0): # If segment is pointing more than 90 degrees away from car's yaw
                # This segment is going backward or sharply sideways. Ignore this point.
                continue
            else:
                corrected_path.append(p2)
        
        return torch.stack(corrected_path)
    def _is_valid_cone_pair(self, p1_idx, p2_idx, all_points: torch.Tensor, colors):
        """
        두 콘(p1, p2)이 유효한 중간점 생성 쌍인지 확인합니다.
        규칙: p1과 p2를 잇는 직선 위에 다른 콘이 너무 가까이 있으면 안 됩니다.
        """
        p1 = all_points[p1_idx]
        p2 = all_points[p2_idx]

        # p1에서 p2로 향하는 벡터
        line_vec = p2 - p1
        line_len_sq = torch.dot(line_vec, line_vec)

        if line_len_sq == 0:
            return False

        for i in range(len(all_points)):
            if i == p1_idx or i == p2_idx: # Check all other cones, regardless of color
                continue
            
            p3 = all_points[i]
            # p3가 p1-p2 선분 위에 있는지 확인 (투영(projection) 사용)
            dot_product = torch.dot(p3 - p1, line_vec)
            if 0 < dot_product < line_len_sq: # p3가 p1과 p2 사이에 투영되는 경우
                # 선분과의 거리 계산
                dist_to_line = torch.linalg.norm(torch.cross(line_vec, p1 - p3)) / torch.linalg.norm(line_vec)
                if dist_to_line < self.path_obstacle_threshold:  # 임계값보다 가까우면 방해물로 간주
                    return False # 유효하지 않은 쌍
        return True
    def _filter_cones_for_planning(self, cones, vehicle_state):
        """
        경로 계획에 사용할 콘을 차량 주변의 관심 영역(ROI)으로 필터링합니다.
        """
        car_pos = torch.tensor(vehicle_state[:2], dtype=torch.float32, device=self.device)
        car_yaw = vehicle_state[2]
        
        filtered_cones = []
        for cone in cones:
            cone_pos = torch.tensor([cone['x'], cone['y']], dtype=torch.float32, device=self.device)
            
            # 1. 거리 필터
            dist = torch.linalg.norm(cone_pos - car_pos)
            if dist > self.planner_roi_distance:
                continue
                
            # 2. 각도 필터
            angle_to_cone = torch.atan2(cone_pos[1] - car_pos[1], cone_pos[0] - car_pos[0])
            angle_diff = self._normalize_angle(angle_to_cone - car_yaw)
            # print(f"angle_diff {angle_diff}")
            if torch.abs(angle_diff) > self.planner_roi_angle_rad:#math.radians(self.planner_roi_angle_rad):
                continue
            
            filtered_cones.append(cone)
        return filtered_cones
    
    def _smooth_path_gpu(self, path):
        if path.shape[0] < self.path_smoothing_kernel_size:
            return path

        # Ensure kernel size is odd
        kernel_size = self.path_smoothing_kernel_size
        if kernel_size % 2 == 0:
            kernel_size += 1

        # Create a 1D Gaussian kernel
        kernel_range = torch.arange(kernel_size, dtype=torch.float32, device=self.device) - (kernel_size - 1) / 2
        gaussian_kernel = torch.exp(-0.5 * (kernel_range / self.path_smoothing_sigma)**2)
        gaussian_kernel = gaussian_kernel / gaussian_kernel.sum()
        gaussian_kernel = gaussian_kernel.view(1, 1, -1)

        # Path needs to be in (N, C, L) format for conv1d, here (1, 2, num_points)
        path_transposed = path.T.unsqueeze(0)

        # Apply padding
        padding = (kernel_size - 1) // 2
        
        # Apply convolution
        smoothed_path_transposed = torch.nn.functional.conv1d(path_transposed, gaussian_kernel.repeat(2, 1, 1), padding=padding, groups=2)

        return smoothed_path_transposed.squeeze(0).T

    def plan_path(self, cones, vehicle_state):
        """
        Generates a driving path based on the detected cones.
        Uses GPU-accelerated midpoint generation and path smoothing.
        Falls back to a simple path if not enough cones are available.
        """
        current_car_pos = torch.tensor(vehicle_state[:2], dtype=torch.float32, device=self.device)
        vehicle_yaw = vehicle_state[2]

        # --- 경로 계획에 사용할 콘 필터링 ---
        local_cones = self._filter_cones_for_planning(cones, vehicle_state)
        rospy.loginfo_throttle(1.0, f"PathPlanner: Cones after ROI filter: {len(local_cones)}")

        # --- 클러스터링 단계 추가 ---
        if len(local_cones) > 5: # 최소 5개 이상의 콘이 있을 때만 클러스터링 수행
            cone_points_np = np.array([[c['x'], c['y']] for c in local_cones])
            # Convert 2D points to 3D for Open3D by adding a dummy Z-coordinate
            cone_points_3d = np.hstack((cone_points_np, np.zeros((cone_points_np.shape[0], 1))))
            
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(cone_points_3d)
            
            # Perform DBSCAN clustering using Open3D
            labels = np.array(pcd.cluster_dbscan(eps=self.planner_clustering_eps, min_points=3, print_progress=False))
            
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
                    cluster_points = cone_points_np[cluster_mask]
                    cluster_center = np.mean(cluster_points, axis=0)
                    dist_to_car = np.linalg.norm(cluster_center - current_car_pos.cpu().numpy())
                    
                    if dist_to_car < min_dist_to_cluster:
                        min_dist_to_cluster = dist_to_car
                        closest_cluster_idx = label
                
                # 가장 가까운 클러스터에 속한 콘만 사용하도록 필터링
                if closest_cluster_idx != -1:
                    closest_cluster_mask = (labels == closest_cluster_idx)
                    local_cones = [cone for i, cone in enumerate(local_cones) if closest_cluster_mask[i]]
                    rospy.loginfo_throttle(1.0, f"PathPlanner: Multiple cone clusters found. Using closest cluster with {len(local_cones)} cones.")
        
        for cone in local_cones:
            rospy.loginfo_throttle(1.0, f"PathPlanner: Cone in local_cones: ID={cone['id']}, Color={cone['color_id']}, Pos=({cone['x']:.2f}, {cone['y']:.2f})")
        
        blue_cones = [c for c in local_cones if c['color_id'] == 1]
        yellow_cones = [c for c in local_cones if c['color_id'] == 2]

        if len(blue_cones) == 0 or len(yellow_cones) == 0:
            rospy.logwarn_throttle(1.0, f"PathPlanner: Fallback reason: No blue ({len(blue_cones)}) or no yellow ({len(yellow_cones)}) cones.")
            path, tri, all_points, colors, unique_midpoints, is_fallback = self._generate_fallback_path(blue_cones, yellow_cones, current_car_pos.cpu().numpy(), vehicle_yaw)
            return path, None, None, None, None, True, len(blue_cones), len(yellow_cones), 0 # 0 midpoints in fallback

        # --- GPU-based Midpoint Generation ---
        blue_points = torch.tensor([[c['x'], c['y']] for c in blue_cones], dtype=torch.float32, device=self.device)
        yellow_points = torch.tensor([[c['x'], c['y']] for c in yellow_cones], dtype=torch.float32, device=self.device)

        dist_matrix = torch.cdist(blue_points, yellow_points)
        
        # For each blue cone, find the closest yellow cone
        min_dists, min_indices = torch.min(dist_matrix, dim=1)
        
        # Create midpoints
        midpoints = (blue_points + yellow_points[min_indices]) / 2.0
        
        # Filter midpoints that are too far apart
        valid_mask = min_dists < self.max_edge_length
        midpoints = midpoints[valid_mask]

        if midpoints.shape[0] < 2:
            rospy.logwarn_throttle(1.0, f"PathPlanner: Fallback reason: Not enough midpoints ({midpoints.shape[0]}).")
            return None, None, None, None, midpoints, True, len(blue_cones), len(yellow_cones), 0

        # Sort midpoints to form a continuous path
        ordered_midpoints_np = self._sort_midpoints(midpoints.cpu().numpy(), current_car_pos.cpu().numpy(), vehicle_yaw)
        if ordered_midpoints_np is None or len(ordered_midpoints_np) < 2:
            rospy.logwarn_throttle(1.0, f"PathPlanner: Fallback reason: Ordered midpoints are insufficient.")
            return None, None, None, None, midpoints, True, len(blue_cones), len(yellow_cones), 0

        ordered_midpoints = torch.from_numpy(ordered_midpoints_np).to(self.device)

        self.path_direction_history.clear() # 새로운 계획 시작 시 방향 기록 초기화
        # Correct any detours in the path
        corrected_path = self._correct_path_detours(ordered_midpoints, vehicle_yaw)

        # Filter path to include only points within max_path_distance from the car
        filtered_path_list = []
        for p in corrected_path:
            if torch.linalg.norm(p - current_car_pos) < self.max_path_distance:
                filtered_path_list.append(p)
        
        if len(filtered_path_list) < 2:
            rospy.logwarn_throttle(1.0, "PathPlanner: Fallback reason: Filtered path has less than 2 points.")
            return None, None, None, None, midpoints, True, len(blue_cones), len(yellow_cones), 0

        filtered_path = torch.stack(filtered_path_list)

        # Smooth the path with GPU-based method
        path_tensor = self._smooth_path_gpu(filtered_path)
        path = path_tensor.cpu().numpy()

        return path, None, None, None, midpoints, False, len(blue_cones), len(yellow_cones), len(midpoints)

    def _calculate_path_curvature(self, path, lookahead=5):
        """경로의 각 지점에서 곡률을 계산합니다."""
        curvatures = torch.zeros(len(path), dtype=torch.float32, device=self.device)
        if len(path) < 3:
            return curvatures

        path_tensor = torch.tensor(path, dtype=torch.float32, device=self.device)

        for i in range(len(path_tensor)):
            p_prev_idx = max(0, i - lookahead)
            p_next_idx = min(len(path_tensor) - 1, i + lookahead)
            
            if p_prev_idx == i or p_next_idx == i: continue
            p_prev, p_curr, p_next = path_tensor[p_prev_idx], path_tensor[i], path_tensor[p_next_idx]
            # Menger Curvature: 세 점으로 곡률 근사
            area = 0.5 * torch.abs((p_prev[0]*(p_curr[1]-p_next[1]) + p_curr[0]*(p_next[1]-p_prev[1]) + p_next[0]*(p_prev[1]-p_curr[1])))
            curvatures[i] = (4 * area) / (torch.linalg.norm(p_prev-p_curr) * torch.linalg.norm(p_curr-p_next) * torch.linalg.norm(p_next-p_prev) + 1e-6)
        return curvatures.cpu().numpy()
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
    def __init__(self, log_directory: str, session_name: str, video_fps: float = 10.0, max_lidar_points: int = 500, max_path_points: int = 100, enable_logging: bool = True):
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
        self.max_path_points = max_path_points
        self.path_csv_file = None


        if not self.enable_logging:
            rospy.loginfo("DataLogger: Logging is disabled.")
            return

        expanded_log_dir = os.path.expanduser(log_directory)
        self.session_path = os.path.join(expanded_log_dir, session_name)
        os.makedirs(self.session_path, exist_ok=True)

        # 1. 메타데이터 CSV 설정
        self.csv_path = os.path.join(self.session_path, "log.csv")
        # 헤더에 제어 튜닝용 데이터 추가
        self.csv_header = [
            'timestamp', 'frame_id', 'autonomous_mode',
            'control_steering', 'control_throttle', 'control_brake',
            'imu_acc_x', 'imu_acc_y', 'imu_acc_z',
            'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
            'gps_latitude', 'gps_longitude',
            'yaw', 'vehicle_vx', "vehicle_vy", 'vehicle_yawrate', 'vehicle_ax', 'vehicle_ay',
            'lidar_point_count',
            # Control Debug Data
            'control_target_speed', 'control_avg_curvature', 'control_side_slip_angle', 'mpc_cost',
            'control_cte', 'control_etheta',
            'mpc_w_cte', 'mpc_w_etheta', 'mpc_w_vel', 'mpc_w_steer', 'mpc_w_accel', 'mpc_w_steer_rate', 'mpc_w_accel_rate',
            'is_fallback_path',
            'num_blue_cones', 'num_yellow_cones', 'num_midpoints'
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

        # 5. Path CSV 설정
        self.path_csv_path = os.path.join(self.session_path, "path.csv")
        self.path_csv_file = open(self.path_csv_path, 'w', newline='')
        self.path_csv_writer = csv.writer(self.path_csv_file)
        path_header = ['frame_id']
        for i in range(self.max_path_points):
            path_header.extend([f'p{i}_x', f'p{i}_y'])
        self.path_csv_writer.writerow(path_header)

        rospy.loginfo(f"DataLogger initialized. Saving logs to: {self.session_path}")

    def log_entry(self, autonomous_mode, control_command, imu_acc, imu_gyro, state, camera1_image, camera2_image, lidar_points, map_cones, path, control_debug_data, is_fallback, num_blue_cones, num_yellow_cones, num_midpoints):
        if not self.enable_logging:
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
        
        # Add control debug data, using .get() to avoid errors if a key is missing
        log_row['control_target_speed'] = control_debug_data.get('target_speed')
        log_row['control_avg_curvature'] = control_debug_data.get('avg_curvature')
        log_row['control_side_slip_angle'] = control_debug_data.get('side_slip_angle')
        log_row['mpc_cost'] = control_debug_data.get('cost')
        log_row['control_cte'] = control_debug_data.get('cte')
        log_row['control_etheta'] = control_debug_data.get('etheta')

        weights = control_debug_data.get('weights', {})
        log_row['mpc_w_cte'] = weights.get('w_cte')
        log_row['mpc_w_etheta'] = weights.get('w_etheta')
        log_row['mpc_w_vel'] = weights.get('w_vel')
        log_row['mpc_w_steer'] = weights.get('w_steer')
        log_row['mpc_w_accel'] = weights.get('w_accel')
        log_row['mpc_w_steer_rate'] = weights.get('w_steer_rate')
        log_row['mpc_w_accel_rate'] = weights.get('w_accel_rate')

        log_row['is_fallback_path'] = is_fallback
        log_row['num_blue_cones'] = num_blue_cones
        log_row['num_yellow_cones'] = num_yellow_cones
        log_row['num_midpoints'] = num_midpoints

        self.metadata_csv_writer.writerow(log_row)

        # --- Log LiDAR Data ---
        if lidar_points is not None and len(lidar_points) > 0:
            lidar_row = [self.frame_count]
            # Flatten points up to the max limit
            flat_lidar = lidar_points[:self.max_lidar_points, :2].flatten().tolist()
            lidar_row.extend(flat_lidar)
            # Pad with empty strings if fewer points than max
            padding = [''] * (self.max_lidar_points * 2 - len(flat_lidar))
            lidar_row.extend(padding)
            self.lidar_csv_writer.writerow(lidar_row)

        # --- Log Path Data ---
        if path is not None and len(path) > 0:
            path_row = [self.frame_count]
            flat_path = path[:self.max_path_points].flatten().tolist()
            path_row.extend(flat_path)
            # Pad if necessary
            padding = [''] * (self.max_path_points * 2 - len(flat_path))
            path_row.extend(padding)
            self.path_csv_writer.writerow(path_row)

        # --- Log Map Cones Data ---
        if map_cones:
            for cone in map_cones:
                self.map_cones_csv_writer.writerow([
                    self.frame_count, cone['id'], cone['color_id'],
                    cone['x'], cone['y'], cone['z']
                ])

        # --- Log Camera Images ---
        self.frame_count += 1
        if camera1_image is not None:
            if self.video_writers['cam1'] is None:
                h, w, _ = camera1_image.shape
                self.video_writers['cam1'] = cv2.VideoWriter(self.video_paths['cam1'], self.fourcc, self.video_fps, (w, h))
            self.video_writers['cam1'].write(camera1_image)

        if camera2_image is not None:
            if self.video_writers['cam2'] is None:
                h, w, _ = camera2_image.shape
                self.video_writers['cam2'] = cv2.VideoWriter(self.video_paths['cam2'], self.fourcc, self.video_fps, (w, h))
            self.video_writers['cam2'].write(camera2_image)


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

        if self.path_csv_file:
            self.path_csv_file.close()
            rospy.loginfo(f"Successfully saved path data to {self.path_csv_path}")

StateTransitionResult = namedtuple(
    'StateTransitionResult', 
    [
        'success', 'from_state', 'to_state', 'reason'
    ]
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
    def __init__(self, path_planner, track_map, device):
        self.device = device # Store the device (cpu or cuda)
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
        self.wheelbase = rospy.get_param("/control/Vehicle/wheel_base", 1.55)
        self.max_steer = rospy.get_param("~max_steer_angle", 1.0) # radians
        self.max_accel = rospy.get_param("/control/Vehicle/max_accel", 0.5) # m/s^2
        self.min_accel = rospy.get_param("/control/Vehicle/min_accel", -0.5) # m/s^2 (braking)

        # Common
        self.kp_throttle = rospy.get_param("/control/SpeedControl/pid_kp", 0.5)
        
        # --- Curvature-based speed control parameters ---
        self.pre_lc_target_speed = rospy.get_param("/control/SpeedControl/pre_loop_closure/target_speed", 8.0)
        self.post_lc_target_speed = rospy.get_param("/control/SpeedControl/post_loop_closure/target_speed", 15.0)
        self.pre_lc_min_speed = rospy.get_param("/control/SpeedControl/pre_loop_closure/min_speed", 6.0)
        self.post_lc_min_speed = rospy.get_param("/control/SpeedControl/post_loop_closure/min_speed", 6.0)
        self.pre_lc_curvature_speed_factor = rospy.get_param("/control/SpeedControl/pre_loop_closure/curvature_speed_factor", 25.0)
        self.post_lc_curvature_speed_factor = rospy.get_param("/control/SpeedControl/post_loop_closure/curvature_speed_factor", 20.0)
        self.pre_lc_curvature_lookahead = rospy.get_param("/control/SpeedControl/pre_loop_closure/curvature_lookahead", 2)
        self.post_lc_curvature_lookahead = rospy.get_param("/control/SpeedControl/post_loop_closure/curvature_lookahead", 2)

        # Pure Pursuit
        self.lookahead_dist = rospy.get_param("/control/PurePursuit/lookahead_distance", 2.5)
        
        # Stanley
        self.k_crosstrack = rospy.get_param("/control/Stanley/k_gain", 0.7)

        # MPC
        self.mpc_horizon = rospy.get_param("/control/MPC/horizon", 5)
        self.mpc_dt = rospy.get_param("/control/MPC/dt", 0.01)
        self.mpc_first_lap_weights = rospy.get_param("/control/MPC/first_lap_weights", {})
        self.mpc_second_lap_weights = rospy.get_param("/control/MPC/second_lap_weights", {})
        self.mpc_fallback_weights = rospy.get_param("/control/MPC/fallback_weights", {})


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
        if isinstance(angle, torch.Tensor):
            return torch.fmod(angle + torch.pi, 2.0 * torch.pi) - torch.pi
        else:
            while angle > math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi
            return angle

    def _compute_pure_pursuit(self, vehicle_state, path, is_fallback=False):
        # Unpack vehicle state
        veh_x, veh_y, veh_yaw = vehicle_state[0], vehicle_state[1], vehicle_state[2]
        current_speed = round(math.sqrt(vehicle_state[3]**2 + vehicle_state[4]**2),4)

        path_tensor = torch.tensor(path, dtype=torch.float32, device=self.device)
        veh_pos_tensor = torch.tensor([veh_x, veh_y], dtype=torch.float32, device=self.device)

        # --- 곡률 기반 목표 속도 계산 ---
        if is_fallback:
            target_speed = 6.0 # Set speed to 5.0 for fallback paths
        else:
            path_curvatures = self.path_planner._calculate_path_curvature(path_tensor, self.pre_lc_curvature_lookahead)
            # 전방 경로의 평균 곡률 계산 (예: 앞 10개 포인트)
            lookahead_curvatures = path_curvatures[:self.pre_lc_curvature_lookahead]
            avg_curvature = torch.mean(torch.tensor(lookahead_curvatures, device=self.device)) if len(lookahead_curvatures) > 0 else torch.tensor(0.0, device=self.device)
            
            if self.track_map.is_loop_closed:
                base_target_speed = self.post_lc_target_speed
            else:
                base_target_speed = self.pre_lc_target_speed

            target_speed = base_target_speed / (1.0 + self.pre_lc_curvature_speed_factor * torch.abs(avg_curvature))
            target_speed = torch.clamp(target_speed, self.pre_lc_min_speed, base_target_speed).item()

        # 1. Find the closest point on the path to the vehicle
        distances = torch.linalg.norm(path_tensor - veh_pos_tensor, dim=1)
        closest_idx = torch.argmin(distances)

        # 2. Find the lookahead point
        lookahead_point = None
        for i in range(closest_idx, len(path_tensor)):
            dist_from_veh = torch.linalg.norm(path_tensor[i] - veh_pos_tensor)
            if dist_from_veh >= self.lookahead_dist:
                lookahead_point = path_tensor[i]
                break
        
        if lookahead_point is None:
            lookahead_point = path_tensor[-1]

        # 3. Transform the lookahead point to the vehicle's coordinate frame
        rot_inv = torch.tensor([[math.cos(veh_yaw), math.sin(veh_yaw)],
                                [-math.sin(veh_yaw), math.cos(veh_yaw)]], dtype=torch.float32, device=self.device)
        translated_point = lookahead_point - veh_pos_tensor
        local_point = torch.matmul(rot_inv, translated_point)

        # 4. Calculate the steering angle
        alpha = torch.atan2(local_point[1], local_point[0])
        actual_lookahead_dist = torch.linalg.norm(lookahead_point - veh_pos_tensor)
        steer = torch.atan2(2.0 * self.wheelbase * torch.sin(alpha), actual_lookahead_dist)
        steer = -torch.clamp(steer, -self.max_steer, self.max_steer).item()

        # 5. Throttle control
        throttle = self.kp_throttle * (target_speed - current_speed)
        throttle = np.clip(throttle, 0.0, 1.0)
        
        brake = 0.0
        if target_speed < current_speed:
            brake = 0.3

        # Normalize steering angle to [-1, 1]
        normalized_steer = steer / self.max_steer

        return throttle, normalized_steer, brake, {}

    def _compute_stanley(self, vehicle_state, path, is_fallback=False):
        """
        Computes control commands using the Stanley method.
        """
        # Unpack vehicle state
        veh_x, veh_y, veh_yaw = vehicle_state[0], vehicle_state[1], vehicle_state[2]
        current_speed = math.sqrt(vehicle_state[3]**2 + vehicle_state[4]**2)

        path_tensor = torch.tensor(path, dtype=torch.float32, device=self.device)
        veh_pos_tensor = torch.tensor([veh_x, veh_y], dtype=torch.float32, device=self.device)

        # --- 곡률 기반 목표 속도 계산 ---
        if is_fallback:
            target_speed = 4.0 # Set speed to 5.0 for fallback paths
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

            path_curvatures = self.path_planner._calculate_path_curvature(path_tensor, target_speed_params['curvature_lookahead'])
            lookahead_curvatures = path_curvatures[:10]
            avg_curvature = torch.mean(torch.tensor(lookahead_curvatures, device=self.device)) if len(lookahead_curvatures) > 0 else torch.tensor(0.0, device=self.device)
            
            base_target_speed = target_speed_params['target_speed']

            target_speed = base_target_speed / (1.0 + target_speed_params['curvature_speed_factor'] * torch.abs(avg_curvature))
            target_speed = torch.clamp(target_speed, target_speed_params['min_speed'], base_target_speed).item()
        # 1. Find the closest path point (target_idx)
        distances = torch.linalg.norm(path_tensor - veh_pos_tensor, dim=1)
        target_idx = torch.argmin(distances)

        # Ensure target_idx is not the last point of the path to calculate path heading
        if target_idx >= len(path_tensor) - 1:
            target_idx = len(path_tensor) - 2

        # 2. Calculate path heading (yaw)
        p1 = path_tensor[target_idx]
        p2 = path_tensor[target_idx + 1]
        path_yaw = torch.atan2(p2[1] - p1[1], p2[0] - p1[0])

        # 3. Calculate heading error (theta_e)
        heading_error = self.normalize_angle(path_yaw - veh_yaw)

        # 4. Calculate cross-track error (e_fa)
        # Vector from closest path point to vehicle
        vec_path_to_veh = veh_pos_tensor - p1
        # Path vector
        vec_path = p2 - p1
        vec_path_normalized = vec_path / (torch.linalg.norm(vec_path) + 1e-6)
        
        # Cross product to find the error and its sign
        cross_track_error = vec_path_normalized[0] * vec_path_to_veh[1] - vec_path_normalized[1] * vec_path_to_veh[0]
        
        # 5. Calculate steering angle (delta)
        # Cross-track steering component
        cte_steer = torch.atan2(self.k_crosstrack * cross_track_error, torch.max(torch.tensor(current_speed, device=self.device), torch.tensor(0.1, device=self.device))) # Add small epsilon to avoid division by zero

        # Total steering angle
        steer = heading_error + cte_steer
        steer = -torch.clamp(steer, -self.max_steer, self.max_steer).item()

        # 6. Throttle control (reusing the same P-controller)
        throttle = self.kp_throttle * (target_speed - current_speed)
        throttle = np.clip(throttle, 0.0, 1.0)
        
        brake = 0.0
        if target_speed < current_speed:
            brake = 0.1

        # Normalize steering angle to [-1, 1]
        normalized_steer = steer / self.max_steer

        return throttle, normalized_steer, brake, {}

    def _cost_function_torch(self, u_tensor, initial_state_tensor, ref_path_tensor, target_speed_tensor, weights_dict):
        # Ensure all inputs are tensors and on the correct device
        # u_tensor: (2 * mpc_horizon,) - accels and steers interleaved
        # initial_state_tensor: (4,) - [x, y, yaw, v]
        # ref_path_tensor: (N, 2)
        # target_speed_tensor: scalar
        # weights_dict: dictionary of scalar weights

        # Unpack control inputs
        accels = u_tensor[0::2]
        steers = u_tensor[1::2]

        # Predict states over the horizon
        predicted_states_list = [initial_state_tensor]

        x, y, yaw, v = initial_state_tensor[0], initial_state_tensor[1], initial_state_tensor[2], initial_state_tensor[3]

        for i in range(self.mpc_horizon):
            a = accels[i]
            delta = steers[i]

            # Kinematic Bicycle Model (PyTorch version)
            # Ensure all operations are on tensors
            x_new = x + v * torch.cos(yaw) * self.mpc_dt
            y_new = y + v * torch.sin(yaw) * self.mpc_dt
            yaw_new = yaw + v / self.wheelbase * torch.tan(delta) * self.mpc_dt
            v_new = v + a * self.mpc_dt
            
            predicted_states_list.append(torch.stack([x_new, y_new, yaw_new, v_new]))

            x, y, yaw, v = x_new, y_new, yaw_new, v_new
        
        predicted_states = torch.stack(predicted_states_list)

        # Calculate cost
        cost = torch.tensor(0.0, device=u_tensor.device, dtype=u_tensor.dtype)

        # Find closest reference path points for each predicted state
        for i in range(1, self.mpc_horizon + 1):
            pred_x, pred_y, pred_yaw, pred_v = predicted_states[i]
            
            # Find closest point on reference path
            # Expand dims for broadcasting: (1, 2) vs (N, 2)
            distances = torch.linalg.norm(ref_path_tensor - torch.stack([pred_x, pred_y]).unsqueeze(0), dim=1)
            closest_idx = torch.argmin(distances)
            
            # Cross-track error
            ref_p1 = ref_path_tensor[closest_idx]
            ref_p2 = ref_path_tensor[closest_idx + 1] if closest_idx < len(ref_path_tensor) - 1 else ref_p1
            
            vec_path = ref_p2 - ref_p1
            vec_path_normalized = vec_path / (torch.linalg.norm(vec_path) + 1e-6)
            vec_to_pred = torch.stack([pred_x, pred_y]) - ref_p1
            
            # Cross product for 2D vectors: z-component of (vec_path_normalized x vec_to_pred)
            cte = vec_path_normalized[0] * vec_to_pred[1] - vec_path_normalized[1] * vec_to_pred[0]

            # Heading error
            ref_yaw = torch.atan2(ref_p2[1] - ref_p1[1], ref_p2[0] - ref_p1[0])
            etheta = self.normalize_angle(pred_yaw - ref_yaw) # normalize_angle can work with tensors if math.pi is replaced by torch.pi

            cost += weights_dict['w_cte'] * cte**2
            cost += weights_dict['w_etheta'] * etheta**2
            cost += weights_dict['w_vel'] * (target_speed_tensor - pred_v)**2

        # Control input cost
        cost += weights_dict['w_accel'] * torch.sum(accels**2)
        cost += weights_dict['w_steer'] * torch.sum(steers**2)

        # Control rate cost (smoothness)
        cost += weights_dict['w_accel_rate'] * torch.sum((accels[1:] - accels[:-1])**2)
        cost += weights_dict['w_steer_rate'] * torch.sum((steers[1:] - steers[:-1])**2)
        
        return cost

    def _compute_mpc(self, vehicle_state, path, is_fallback=False):
        # Unpack vehicle state
        veh_x, veh_y, veh_yaw = vehicle_state[0], vehicle_state[1], vehicle_state[2]
        current_speed = math.sqrt(vehicle_state[3]**2 + vehicle_state[4]**2)
        initial_state = [veh_x, veh_y, veh_yaw, current_speed]

        # --- For Logging: Calculate current errors ---
        path_points_for_error = torch.tensor(path, dtype=torch.float32, device=self.device)
        veh_pos_tensor = torch.tensor([veh_x, veh_y], dtype=torch.float32, device=self.device)
        distances_error = torch.linalg.norm(path_points_for_error - veh_pos_tensor, dim=1)
        closest_idx_error = torch.argmin(distances_error)

        if closest_idx_error >= len(path_points_for_error) - 1:
            p1_error = path_points_for_error[closest_idx_error - 1]
            p2_error = path_points_for_error[closest_idx_error]
        else:
            p1_error = path_points_for_error[closest_idx_error]
            p2_error = path_points_for_error[closest_idx_error + 1]

        path_yaw_error = torch.atan2(p2_error[1] - p1_error[1], p2_error[0] - p1_error[0])
        etheta_current = self.normalize_angle(path_yaw_error - veh_yaw)
        vec_path_error = p2_error - p1_error
        vec_path_normalized_error = vec_path_error / (torch.linalg.norm(vec_path_error) + 1e-6)
        vec_to_veh_error = veh_pos_tensor - p1_error
        cte_current = vec_path_normalized_error[0] * vec_to_veh_error[1] - vec_path_normalized_error[1] * vec_to_veh_error[0]
        # --- End of Error Calculation for Logging ---

        # --- Fallback Mode vs Normal Mode ---
        if is_fallback:
            # In fallback mode, prioritize stability with safe weights and low speed
            weights = self.mpc_fallback_weights.copy()
            mpc_target_speed = self.pre_lc_min_speed # Use a predefined safe speed
            avg_curvature = torch.tensor(0.0, device=self.device) # No curvature in fallback
            rospy.logwarn_throttle(1.0, "Control: Fallback path detected. Using safe control mode.")
        else:
            # --- Normal Operation ---
            if self.track_map.is_loop_closed:
                weights = self.mpc_second_lap_weights.copy()
                target_speed_params = {
                    'target_speed': self.post_lc_target_speed,
                    'min_speed': self.post_lc_min_speed,
                    'curvature_speed_factor': self.post_lc_curvature_speed_factor,
                    'curvature_lookahead': self.post_lc_curvature_lookahead
                }
            else:
                weights = self.mpc_first_lap_weights.copy()
                target_speed_params = {
                    'target_speed': self.pre_lc_target_speed,
                    'min_speed': self.pre_lc_min_speed,
                    'curvature_speed_factor': self.pre_lc_curvature_speed_factor,
                    'curvature_lookahead': self.pre_lc_curvature_lookahead
                }

            path_curvatures = self.path_planner._calculate_path_curvature(path_points_for_error, target_speed_params['curvature_lookahead'])
            lookahead_curvatures = path_curvatures[:self.mpc_horizon]
            avg_curvature = torch.mean(torch.tensor(lookahead_curvatures, device=self.device)) if len(lookahead_curvatures) > 0 else torch.tensor(0.0, device=self.device)
            
            base_target_speed = target_speed_params['target_speed']
            mpc_target_speed = base_target_speed / (1.0 + target_speed_params['curvature_speed_factor'] * torch.abs(avg_curvature))
            mpc_target_speed = torch.clamp(mpc_target_speed, target_speed_params['min_speed'], base_target_speed).item()

        # --- MPC Solver (common for all modes) ---
        path_points = torch.tensor(path, dtype=torch.float32, device=self.device)
        distances = torch.linalg.norm(path_points - veh_pos_tensor, dim=1)
        start_idx = torch.argmin(distances)
        ref_path = path_points[start_idx:start_idx + self.mpc_horizon + 2]
        if len(ref_path) < self.mpc_horizon + 2:
            last_point = ref_path[-1]
            padding = torch.stack([last_point] * (self.mpc_horizon + 2 - len(ref_path)))
            ref_path = torch.cat([ref_path, padding])

        u0 = torch.zeros(2 * self.mpc_horizon, dtype=torch.float32, device=self.device)
        bounds = []
        for _ in range(self.mpc_horizon):
            bounds.append((self.min_accel, self.max_accel))
            bounds.append((-self.max_steer, self.max_steer))

        # Convert inputs to PyTorch tensors
        initial_state_tensor = torch.tensor(initial_state, dtype=torch.float32, device=self.device)
        ref_path_tensor = ref_path
        mpc_target_speed_tensor = torch.tensor(mpc_target_speed, dtype=torch.float32, device=self.device)
        
        # Convert weights to tensors if they are not already
        weights_tensor_dict = {k: torch.tensor(v, dtype=torch.float32, device=self.device) for k, v in weights.items()}

        # Initialize control inputs as a PyTorch tensor
        u_tensor = torch.zeros(2 * self.mpc_horizon, dtype=torch.float32, device=self.device, requires_grad=True)
        
        # Define optimizer
        optimizer = torch.optim.Adam([u_tensor], lr=0.01) # Learning rate can be tuned

        # Optimization loop
        num_iterations = 15 # Number of optimization steps, can be tuned
        for iteration in range(num_iterations):
            optimizer.zero_grad()
            cost = self._cost_function_torch(u_tensor, initial_state_tensor, ref_path_tensor, mpc_target_speed_tensor, weights_tensor_dict)
            cost.backward()
            optimizer.step()

            # Apply bounds/constraints by clipping
            with torch.no_grad():
                for i in range(self.mpc_horizon):
                    # Accel bounds
                    u_tensor[2*i].clamp_(self.min_accel, self.max_accel)
                    # Steer bounds
                    u_tensor[2*i+1].clamp_(-self.max_steer, self.max_steer)
        
        # Extract optimal controls
        optimal_accel = u_tensor[0].item()
        optimal_steer = u_tensor[1].item()
        mpc_cost_value = cost.item() # Get the final cost for logging

        # --- Prepare Debug Data for Logging ---
        debug_data = {
            'target_speed': mpc_target_speed,
            'avg_curvature': avg_curvature.item(),
            'side_slip_angle': 0.0,
            'cost': mpc_cost_value,
            'weights': weights,
            'cte': cte_current.item(),
            'etheta': etheta_current.item()
        }

        # --- Map acceleration to throttle/brake ---
        throttle = 0.0
        brake = 0.0
        if optimal_accel > 0:
            throttle = np.clip(optimal_accel / self.max_accel, 0.0, 0.5)
        else:
            brake = np.clip(-optimal_accel / abs(self.min_accel), 0.0, 0.5)

        # Normalize steering angle to [-1, 1]
        normalized_steer = np.clip(-optimal_steer / self.max_steer, -1.0, 1.0)

        return throttle, normalized_steer, brake, debug_data
