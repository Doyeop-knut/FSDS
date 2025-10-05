#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file formula_autonomous_system.py
@author Jiwon Seok (jiwonseok@hanyang.ac.kr)
@brief Formula Student Driverless Autonomous System - Python Implementation
@version 0.2
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

# Data Logger & Plotting
import os
import csv
import datetime
import multiprocessing as mp

# ==================== Path Planning (RRT*) ====================

# class RRTStarPlanner:
#     """
#     RRT* Path Planner
#     """

#     class Node:
#         def __init__(self, x, y):
#             self.x = x
#             self.y = y
#             self.parent = None
#             self.cost = 0.0

#     def __init__(self, start, goal, obstacles, obstacle_radius, play_area, max_iter=500, step_size=1.0, search_radius=5.0, goal_sample_rate=0.1):
#         self.start = self.Node(start[0], start[1])
#         self.goal = self.Node(goal[0], goal[1])
#         self.obstacles = np.array(obstacles)
#         self.obstacle_radius = obstacle_radius
#         self.play_area = play_area  # [min_x, max_x, min_y, max_y]
#         self.max_iter = max_iter
#         self.step_size = step_size
#         self.search_radius = search_radius
#         self.goal_sample_rate = goal_sample_rate
#         self.node_list = [self.start]

#     def plan(self):
#         """Main RRT* planning loop"""
#         for i in range(self.max_iter):
#             # 1. Sample a random point
#             if random.random() > self.goal_sample_rate:
#                 rnd_node = self.get_random_node()
#             else:
#                 rnd_node = self.Node(self.goal.x, self.goal.y)

#             # 2. Find the nearest node in the tree
#             nearest_node = self.get_nearest_node(rnd_node)

#             # 3. Steer from nearest to random point
#             new_node = self.steer(nearest_node, rnd_node)

#             # 4. If the path to the new node is collision-free
#             if self.is_collision_free(nearest_node, new_node):
#                 # 5. Find near nodes and choose the best parent (lowest cost)
#                 near_nodes_indices = self.find_near_nodes(new_node)
#                 self.choose_parent(new_node, near_nodes_indices)
                
#                 self.node_list.append(new_node)

#                 # 6. Rewire the tree
#                 self.rewire(new_node, near_nodes_indices)

#         # 7. Generate final path
#         return self.generate_final_path()

#     def get_random_node(self):
#         return self.Node(
#             random.uniform(self.play_area[0], self.play_area[1]),
#             random.uniform(self.play_area[2], self.play_area[3])
#         )

#     def get_nearest_node(self, node):
#         distances = [(n.x - node.x)**2 + (n.y - node.y)**2 for n in self.node_list]
#         return self.node_list[np.argmin(distances)]

#     def steer(self, from_node, to_node):
#         d, theta = self.get_distance_and_angle(from_node, to_node)
        
#         new_node = self.Node(from_node.x, from_node.y)
#         new_node.x += min(self.step_size, d) * math.cos(theta)
#         new_node.y += min(self.step_size, d) * math.sin(theta)
#         new_node.parent = from_node
#         return new_node

#     def is_collision_free(self, from_node, to_node):
#         if not self.obstacles.any():
#             return True
        
#         points = np.linspace([from_node.x, from_node.y], [to_node.x, to_node.y], num=10)
#         for p in points:
#             distances = np.sqrt(np.sum((self.obstacles - p)**2, axis=1))
#             if np.any(distances < self.obstacle_radius):
#                 return False
#         return True

#     def find_near_nodes(self, new_node):
#         n_nodes = len(self.node_list)
#         distances = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]
#         near_indices = [i for i, d in enumerate(distances) if d < self.search_radius**2]
#         return near_indices

#     def choose_parent(self, new_node, near_indices):
#         if not near_indices:
#             return

#         costs = []
#         for i in near_indices:
#             near_node = self.node_list[i]
#             d, _ = self.get_distance_and_angle(near_node, new_node)
#             if self.is_collision_free(near_node, new_node):
#                 costs.append(near_node.cost + d)
#             else:
#                 costs.append(float('inf'))
        
#         min_cost_idx = near_indices[np.argmin(costs)]
#         min_cost_node = self.node_list[min_cost_idx]

#         new_node.parent = min_cost_node
#         new_node.cost = min_cost_node.cost + self.get_distance_and_angle(min_cost_node, new_node)[0]

#     def rewire(self, new_node, near_indices):
#         for i in near_indices:
#             node = self.node_list[i]
#             d, _ = self.get_distance_and_angle(new_node, node)
#             if new_node.cost + d < node.cost and self.is_collision_free(new_node, node):
#                 node.parent = new_node
#                 node.cost = new_node.cost + d

#     def generate_final_path(self):
#         # Find the node in the tree closest to the goal
#         distances_to_goal = [(n.x - self.goal.x)**2 + (n.y - self.goal.y)**2 for n in self.node_list]
#         best_node_idx = np.argmin(distances_to_goal)
#         best_node = self.node_list[best_node_idx]

#         # If the best node is within a certain threshold of the goal, consider it reached
#         if self.get_distance_and_angle(best_node, self.goal)[0] > self.step_size * 2:
#             return None # Path not found

#         path = []
#         node = best_node
#         while node.parent is not None:
#             path.append((node.x, node.y))
#             node = node.parent
#         path.append((self.start.x, self.start.y))
#         return path[::-1]

#     @staticmethod
#     def get_distance_and_angle(from_node, to_node):
#         dx = to_node.x - from_node.x
#         dy = to_node.y - from_node.y
#         d = math.hypot(dx, dy)
#         theta = math.atan2(dy, dx)
#         return d, theta

# def plot_process_func(queue):
#     import matplotlib.pyplot as plt
#     import math
#     import numpy as np
#     import matplotlib
#     matplotlib.use('TkAgg')

#     plt.ion()
#     fig, ax = plt.subplots(figsize=(10, 10))
    
#     all_cones = set()
#     vehicle_trajectory = []

#     while True:
#         try:
#             data = queue.get()
#             if data is None:
#                 break
            
#             vehicle_state, cone_map, rrt_nodes, final_path = data
            
#             vehicle_x, vehicle_y = vehicle_state[0], vehicle_state[1]
#             vehicle_trajectory.append((vehicle_x, vehicle_y))
            
#             for cone in cone_map:
#                 all_cones.add(tuple(cone))

#             ax.clear()
            
#             # Plot Cones
#             if all_cones:
#                 cones_array = np.array(list(all_cones))
#                 ax.scatter(cones_array[:, 0], cones_array[:, 1], c='b', label='Cones')

#             # Plot Trajectory
#             if vehicle_trajectory:
#                 traj_array = np.array(vehicle_trajectory)
#                 ax.plot(traj_array[:, 0], traj_array[:, 1], c='g', linewidth=1.5, label='Trajectory')

#             # Plot RRT Tree
#             if rrt_nodes:
#                 for node in rrt_nodes:
#                     if node.parent:
#                         ax.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g", linewidth=0.5)

#             # Plot Final Path
#             if final_path:
#                 path_arr = np.array(final_path)
#                 ax.plot(path_arr[:, 0], path_arr[:, 1], "-r", linewidth=2, label="RRT* Path")

#             # Plot Vehicle
#             vehicle_yaw_rad = vehicle_state[2]
#             ax.scatter(vehicle_x, vehicle_y, c='r', marker='x', s=100, label='Vehicle')
#             ax.arrow(vehicle_x, vehicle_y, 2.0 * math.cos(vehicle_yaw_rad), 2.0 * math.sin(vehicle_yaw_rad), head_width=0.5, fc='r', ec='r')

#             ax.set_xlabel("X coordinate (m)")
#             ax.set_ylabel("Y coordinate (m)")
#             ax.set_title("RRT* Path Planning")
#             ax.legend()
#             ax.grid(True)
#             ax.set_aspect('equal', adjustable='box')
            
#             plt.draw()
#             plt.pause(0.001)
#         except (KeyboardInterrupt, ValueError):
#             break
#     plt.close(fig)
#     print("Plotting process finished.")

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
        
        self.cone_map = set()

        self.data_logger = DataLogger(
            log_directory="/home/user/fsds_ws/src/tutorial/log",
            session_name=datetime.datetime.now().strftime("%Y%m%d_%H%M%S"),
            max_lidar_points=50
        )

        self.gps_util = GPSIMUProcessor()
        self.state_machine = StateMachine()
        self.lidar_util = LiDARProcessor()

        self.plot_queue = mp.Queue(maxsize=1)
        self.plot_process = mp.Process(target=plot_process_func, args=(self.plot_queue,))
        self.plot_process.start()
        rospy.on_shutdown(self.cleanup)
        
    def cleanup(self):
        print("Shutting down plotting process...")
        if self.plot_process.is_alive():
            self.plot_queue.put(None)
            self.plot_process.join(timeout=1)
        print("Plotting process stopped.")

    def init(self):
        self.is_initialized = True
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
        self.lidar_util.publish_point_cloud(cluster)

        if go_signal_msg.mission != "None" and go_signal_msg.mission != "":
            self.state_machine.inject_go_signal(go_signal_msg.mission, go_signal_msg.track)
        autonomous_mode.data = self.state_machine.get_current_state_string()

        control_command_msg = ControlCommand()

        # Transform cluster points to map frame
        cluster_map_frame = np.empty((0, 2))
        vehicle_x, vehicle_y, vehicle_yaw_rad = self.gps_util.state[0], self.gps_util.state[1], self.gps_util.state[2]
        if cluster.size > 0:
            cos_yaw = math.cos(vehicle_yaw_rad)
            sin_yaw = math.sin(vehicle_yaw_rad)
            rot_mat = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
            cluster_xy = cluster[:, :2]
            cluster_map_frame = (rot_mat @ cluster_xy.T).T + np.array([vehicle_x, vehicle_y])
            for cone in cluster_map_frame:
                self.cone_map.add(tuple(cone))

        # RRT* Path Planning
        start_point = (vehicle_x, vehicle_y)
        # Simple goal: 15m ahead of the vehicle
        goal_point = (vehicle_x + 15 * math.cos(vehicle_yaw_rad), vehicle_y + 15 * math.sin(vehicle_yaw_rad))
        
        # Define a search area around the vehicle
        search_area = [vehicle_x - 5, vehicle_x + 20, vehicle_y - 10, vehicle_y + 10]

        planner = RRTStarPlanner(start=start_point, goal=goal_point, obstacles=list(self.cone_map), obstacle_radius=0.5, play_area=search_area, max_iter=100)
        final_path = planner.plan()

        # Logging and Visualization
        self.data_logger.log_entry(
            autonomous_mode=autonomous_mode.data, control_command=control_command_msg,
            imu_acc=acc, imu_gyro=gyro, state=self.gps_util.state,
            camera1_image=image1, camera2_image=image2, lidar_points=cluster_map_frame
        )

        if not self.plot_queue.full():
            plot_data = (self.gps_util.state, self.cone_map, planner.node_list, final_path)
            self.plot_queue.put(plot_data)

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
        for label in set(labels):
            if label == -1: continue
            cluster_points = points[labels == label]
            center = np.mean(cluster_points, axis=0)
            center_4d = np.array([center[0], center[1], center[2], 1])
            mat = self.vehicle_to_lidar_Transform()
            transform_lidar = np.dot(mat, center_4d)
            clusters.append(transform_lidar[:3])
        
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