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
## TESTING

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
from sensor_msgs.msg import PointCloud2, Image, Imu, NavSatFix, PointField
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

# 3D LiDAR
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor

# Data Logger
import os
import csv
import datetime


# 필요한 import 문을 상단에 추가하세요.
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from googleapiclient.http import MediaFileUpload
import os
import cv2
import rospy
import datetime
from fs_msgs.msg import ControlCommand
import numpy as np

import threading
import queue
# ==================== CloudDataLogger (새로운 통합 클래스) ====================

# API 권한 범위를 Drive와 Sheets 모두 포함하도록 수정합니다.
SCOPES = ['https://www.googleapis.com/auth/drive', 'https://www.googleapis.com/auth/spreadsheets']

class CloudDataLogger:
    
    """
    센서 데이터를 Google Sheets에 실시간으로 기록하고, 
    영상 파일은 종료 시 Google Drive에 업로드하는 클래스.
    """
    def __init__(self, session_name: str, video_log_directory: str, video_fps: float = 10.0, max_lidar_points: int = 50,  batch_size: int = 100, batch_timeout: float = 1.0):
        self.session_name = session_name
        self.video_session_path = os.path.join(video_log_directory, self.session_name)
        os.makedirs(self.video_session_path, exist_ok=True)

        self.max_lidar_points = max_lidar_points
        self.frame_count = 0
        # ==================== 배치 설정 추가 ====================
        self.BATCH_SIZE = batch_size      # 한 번에 보낼 최대 로그 개수
        self.BATCH_TIMEOUT = batch_timeout  # 배치를 보내기 전 최대 대기 시간 (초)
        # =======================================================
        # --- Google API 서비스 초기화 ---
        self.drive_service, self.sheets_service = self._authenticate()
        self.spreadsheet_id = None
        self.session_drive_folder_id = None

        if self.drive_service and self.sheets_service:
            self._setup_cloud_session()
        else:
            rospy.logerr("Failed to initialize Google services. Cloud logging disabled.")

        # --- 비디오 녹화 설정 (로컬 임시 저장) ---
        self.video_paths = {
            'cam1': os.path.join(self.video_session_path, "camera1.avi"),
            'cam2': os.path.join(self.video_session_path, "camera2.avi")
        }
        self.video_writers = {'cam1': None, 'cam2': None}
        self.video_fps = video_fps
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        
        rospy.loginfo(f"CloudDataLogger initialized for session: {self.session_name}")
        if self.spreadsheet_id:
            rospy.loginfo(f"Logging to Google Sheet ID: {self.spreadsheet_id}")

        # ==================== 쓰레딩 관련 설정 추가 ====================
        self.log_queue = queue.Queue()
        self.shutdown_event = threading.Event()
        self.log_thread = threading.Thread(target=self._log_worker, daemon=True)
        self.log_thread.start()
        # ============================================================
        rospy.loginfo(f"Batching CloudDataLogger initialized for session: {self.session_name}")

    def _authenticate(self):
        """OAuth 2.0 인증을 수행하고 Drive와 Sheets 서비스 객체를 반환합니다."""
        creds = None
        token_path = 'token.json'
        credentials_path = 'credentials.json'

        if os.path.exists(token_path):
            creds = Credentials.from_authorized_user_file(token_path, SCOPES)
        
        if not creds or not creds.valid:
            if creds and creds.expired and creds.refresh_token:
                creds.refresh(Request())
            else:
                flow = InstalledAppFlow.from_client_secrets_file(credentials_path, SCOPES)
                creds = flow.run_local_server(port=0)
            with open(token_path, 'w') as token:
                token.write(creds.to_json())
        
        try:
            drive_service = build('drive', 'v3', credentials=creds)
            sheets_service = build('sheets', 'v4', credentials=creds)
            rospy.loginfo("Google Drive and Sheets API services created successfully.")
            return drive_service, sheets_service
        except HttpError as error:
            rospy.logerr(f"An error occurred creating Google services: {error}")
            return None, None

    def _setup_cloud_session(self):
        """세션을 위한 구글 드라이브 폴더와 스프레드시트를 생성 및 설정합니다."""
        try:
            # 1. 최상위 로그 폴더 찾기 또는 생성
            main_log_folder_id = self._find_or_create_drive_folder("FSDS_Logs")

            # 2. 이번 세션을 위한 폴더 생성
            self.session_drive_folder_id = self._find_or_create_drive_folder(self.session_name, parent_id=main_log_folder_id)

            # 3. 새 스프레드시트 생성
            spreadsheet_body = {
                'properties': {'title': f"{self.session_name}_Log"},
                'sheets': [{'properties': {'title': 'Metadata'}}, {'properties': {'title': 'LiDAR'}}]
            }
            spreadsheet = self.sheets_service.spreadsheets().create(body=spreadsheet_body).execute()
            self.spreadsheet_id = spreadsheet['spreadsheetId']
            rospy.loginfo(f"Created new Google Sheet: {spreadsheet['properties']['title']}")

            # 4. 생성된 스프레드시트를 세션 폴더로 이동
            self.drive_service.files().update(
                fileId=self.spreadsheet_id,
                addParents=self.session_drive_folder_id,
                removeParents='root',
                fields='id, parents'
            ).execute()

            # 5. 각 시트에 헤더 추가
            self._append_to_sheet('Metadata', [
                'timestamp', 'frame_id', 'autonomous_mode',
                'control_steering', 'control_throttle', 'control_brake',
                'imu_acc_x', 'imu_acc_y', 'imu_acc_z',
                'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
                'gps_x', 'gps_y', 'gps_z', 'lidar_point_count'
            ])
            lidar_header = ['frame_id'] + [f'p{i}_{axis}' for i in range(self.max_lidar_points) for axis in ['x', 'y']]
            self._append_to_sheet('LiDAR', lidar_header)

        except HttpError as error:
            rospy.logerr(f"Failed to setup cloud session: {error}")
            self.spreadsheet_id = None
            self.session_drive_folder_id = None
            
    def _find_or_create_drive_folder(self, name, parent_id=None):
        """구글 드라이브에서 폴더를 찾거나 생성합니다."""
        query = f"name='{name}' and mimeType='application/vnd.google-apps.folder' and trashed=false"
        if parent_id:
            query += f" and '{parent_id}' in parents"
        
        response = self.drive_service.files().list(q=query, spaces='drive', fields='files(id)').execute()
        if response['files']:
            return response['files'][0]['id']
        else:
            meta = {'name': name, 'mimeType': 'application/vnd.google-apps.folder'}
            if parent_id: meta['parents'] = [parent_id]
            return self.drive_service.files().create(body=meta, fields='id').execute()['id']

    def _append_to_sheet(self, sheet_name, values):
        """스프레드시트에 여러 행(values)을 한 번에 추가합니다."""
        if not self.spreadsheet_id or not values: return
        try:
            self.sheets_service.spreadsheets().values().append(
                spreadsheetId=self.spreadsheet_id,
                range=f"'{sheet_name}'!A1",
                valueInputOption='USER_ENTERED',
                # body에 [values]가 아닌 values를 직접 전달하여 여러 행을 보냅니다.
                body={'values': values} 
            ).execute()
        except HttpError as error:
            # 에러 메시지에 몇 개의 행을 보내려다 실패했는지 추가하면 디버깅에 용이
            rospy.logwarn(f"Could not append batch of {len(values)} rows to sheet '{sheet_name}': {error}")

    
    def _upload_file_to_drive(self, file_path):
        """로컬 파일을 세션의 드라이브 폴더에 업로드합니다."""
        if not self.session_drive_folder_id:
            rospy.logerr("Session Drive folder ID not set. Cannot upload video.")
            return
        
        file_name = os.path.basename(file_path)
        rospy.loginfo(f"Uploading {file_name} to Google Drive...")
        media = MediaFileUpload(file_path, resumable=True)
        request = self.drive_service.files().create(
            body={'name': file_name, 'parents': [self.session_drive_folder_id]},
            media_body=media,
            fields='id'
        )
        response = None
        while response is None:
            status, response = request.next_chunk()
            if status:
                rospy.loginfo(f"  -> {int(status.progress() * 100)}%")
        rospy.loginfo(f"File '{file_name}' uploaded successfully.")


    def _log_worker(self):
        """
        Queue에서 로그를 가져와 배치로 묶은 뒤, 한 번의 API 호출로 전송하는 작업자 함수.
        """
        rospy.loginfo("Log worker thread with batching started.")
        
        # 각 시트별로 데이터를 모을 임시 버퍼(배치)
        metadata_batch = []
        lidar_batch = []
        last_sent_time = time.time()

        while not self.shutdown_event.is_set():
            try:
                # 타임아웃을 짧게 설정하여 루프가 너무 오래 멈추지 않도록 함
                log_item = self.log_queue.get(timeout=0.1)
                
                # 데이터 타입에 따라 각자의 배치에 추가
                if log_item['type'] == 'metadata':
                    metadata_batch.append(log_item['data'])
                elif log_item['type'] == 'lidar':
                    lidar_batch.append(log_item['data'])
                
                self.log_queue.task_done()

            except queue.Empty:
                # Queue가 비어있으면 아무것도 하지 않음
                pass

            # 배치를 보낼 조건 확인:
            # 1. 메타데이터 배치가 꽉 찼거나
            # 2. 마지막 전송 후 일정 시간이 지났고, 보낼 데이터가 있을 때
            if (len(metadata_batch) >= self.BATCH_SIZE) or \
               (time.time() - last_sent_time > self.BATCH_TIMEOUT and (metadata_batch or lidar_batch)):
                
                if metadata_batch:
                    self._append_to_sheet('Metadata', metadata_batch)
                    metadata_batch = [] # 배치 비우기
                
                if lidar_batch:
                    self._append_to_sheet('LiDAR', lidar_batch)
                    lidar_batch = [] # 배치 비우기

                last_sent_time = time.time() # 마지막 전송 시간 갱신
        
        # 종료 직전, 남아있는 모든 데이터를 전송
        rospy.loginfo("Log worker shutting down, sending remaining data...")
        if metadata_batch:
            self._append_to_sheet('Metadata', metadata_batch)
        if lidar_batch:
            self._append_to_sheet('LiDAR', lidar_batch)

    def log_entry(self, autonomous_mode: str, control_command: ControlCommand,
                  imu_acc: list, imu_gyro: list, gps_data: tuple,
                  camera1_image: np.ndarray, camera2_image: np.ndarray, lidar_points: np.ndarray):
        
        # ==================== 매우 빠르게 동작하도록 수정 ====================
        # 이 메서드는 이제 데이터를 Queue에 넣기만 하고 즉시 반환됩니다.
        
        timestamp = rospy.Time.now().to_sec()
        point_count = len(lidar_points) if lidar_points is not None else 0

        # 1. 메타데이터를 Dictionary 형태로 만들어 Queue에 넣음
        metadata_row = [
            timestamp, self.frame_count, autonomous_mode,
            control_command.steering, control_command.throttle, control_command.brake,
            imu_acc[0], imu_acc[1], imu_acc[2], imu_gyro[0], imu_gyro[1], imu_gyro[2],
            gps_data[0], gps_data[1], gps_data[2], point_count
        ]
        self.log_queue.put({'type': 'metadata', 'data': metadata_row})

        # 2. LiDAR 데이터를 만들어 Queue에 넣음
        lidar_row = [self.frame_count]
        if point_count > 0:
            points_flat = lidar_points[:self.max_lidar_points, :2].flatten().tolist()
            lidar_row.extend(points_flat)
        padding_len = (1 + self.max_lidar_points * 2) - len(lidar_row)
        lidar_row.extend([''] * padding_len)
        self.log_queue.put({'type': 'lidar', 'data': lidar_row})
        # ===================================================================

        # 3. 카메라 데이터 로깅 (로컬 저장이므로 그대로 둠)
        images = {'cam1': camera1_image, 'cam2': camera2_image}
        for cam_id, img in images.items():
            if img is None: continue
            if self.video_writers[cam_id] is None:
                h, w, _ = img.shape
                self.video_writers[cam_id] = cv2.VideoWriter(self.video_paths[cam_id], self.fourcc, self.video_fps, (w, h))
            self.video_writers[cam_id].write(img)

        self.frame_count += 1

    def close(self):
        """
        로깅 쓰레드를 안전하게 종료하고 비디오 파일을 업로드합니다.
        """
        rospy.loginfo("Closing logger...")

        # 1. 로깅 쓰레드에 종료 신호 보내기
        rospy.loginfo("Waiting for log queue to be processed...")
        self.log_queue.join()  # Queue에 쌓인 모든 아이템이 처리될 때까지 대기
        self.shutdown_event.set() # 쓰레드의 while 루프를 빠져나가도록 신호
        self.log_thread.join() # 쓰레드가 완전히 종료될 때까지 대기

        rospy.loginfo("Log thread successfully shut down.")

        # 2. 비디오 파일 핸들 닫고 드라이브에 업로드 (기존과 동일)
        for cam_id, writer in self.video_writers.items():
            if writer is not None:
                writer.release()
                rospy.loginfo(f"Locally saved video: {self.video_paths[cam_id]}")
                self._upload_file_to_drive(self.video_paths[cam_id])
        
        rospy.loginfo("CloudDataLogger session finished.")
    
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
        
        # ==================== 데이터 로거 추가 ====================
        self.data_logger = CloudDataLogger(
            session_name=datetime.datetime.now().strftime("%Y%m%d_%H%M%S"),
            video_log_directory="/home/user/fsds_ws/src/tutorial/log/video_temp", # 비디오 임시 저장 폴더
            max_lidar_points=50
        )
        # =========================================================

        # State machine: Autonomous mode
        self.gps_util = GPSProcessor()
        self.state_machine = StateMachine()
        
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

        cv2.imshow("Camera1", self.get_camera_image(camera1_msg))
        cv2.imshow("Camera2", self.get_camera_image(camera2_msg))
        acc, gyro, orientation = self.get_imu_data(imu_msg)
        lat, lon, alt = self.get_gps_data(gps_msg)
        # self.gps_util.set_origin(lat, lon, alt)  # 최초 GPS 좌표를 원점으로 설정
        self.gps_util.origin_set = True
        x,y,z = self.gps_util.gps_to_local(lat, lon, alt)
        offset = np.array([x,y,z])
        # print(offset[:2])
        image1 = self.get_camera_image(camera1_msg)
        image2 = self.get_camera_image(camera2_msg)
        cv2.waitKey(1)

        ## LiDAR Processed
        # print(f"parameters = {self.dbscan_eps, self.dbscan_points, self.ransac_distance, self.ransac_iter, self.x_min, self.x_max}")
        points=self.get_lidar_point_cloud(lidar_msg)
        LiDARProcessor().filtering_points(points, (self.x_min, self.x_max), (self.y_min, self.y_max), (self.z_min, self.z_max))
        LiDARProcessor().ransac_plane_removal(points, threshold=self.ransac_distance, max_trials=self.ransac_iter)
        # print(self.dbscan_eps)
        cluster = LiDARProcessor().cluster_points(points, eps=self.dbscan_eps, min_samples=self.dbscan_points)
        # print(cluster[:,:2])
        LiDARProcessor().publish_point_cloud(points)
        # print(go_signal_msg.mission, go_signal_msg.track)

        ## GO_SIGNAL
        if go_signal_msg.mission != "None" and go_signal_msg.mission != "":
            self.state_machine.inject_go_signal(go_signal_msg.mission, go_signal_msg.track)
        autonomous_mode.data = self.state_machine.get_current_state_string()

        # filtered_points = LiDARProcessor().filtering_points(np.array([[x,y,z]]), (1.0, 20.0), (-10.0, 10.0), (-0.5, 0.5))
        # print("Filtered Points:", filtered_points)
    
        # Control
        control_command_msg = ControlCommand()
        # print(go_signal_msg)

        # ==================== Data Logger (Test) ====================
        self.data_logger.log_entry(
            autonomous_mode=autonomous_mode.data,
            control_command=control_command_msg,
            imu_acc=acc,
            imu_gyro=gyro,
            gps_data=(x, y, z),
            camera1_image=image1,
            camera2_image=image2,
            lidar_points=cluster[:,:2] + offset[:2]
        )
        # =========================================================
        

        return True, control_command_msg, autonomous_mode

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
    
class LiDARProcessor:
    def __init__(self):
        self.lidar_publisher = rospy.Publisher("/processed_lidar", PointCloud2, queue_size=1)
        self.x_min, self.x_max = rospy.get_param("/perception/lidar_roi_extraction/x_min") , rospy.get_param("/perception/lidar_roi_extraction/x_max")
        self.y_min, self.y_max = rospy.get_param("/perception/lidar_roi_extraction/y_min") , rospy.get_param("/perception/lidar_roi_extraction/y_max")
        self.z_min, self.z_max = rospy.get_param("/perception/lidar_roi_extraction/z_min") , rospy.get_param("/perception/lidar_roi_extraction/z_max")
        self.ransac_iter = rospy.get_param("/perception/lidar_ground_removal/ransac_iterations")
        self.ransac_distance = rospy.get_param("/perception/lidar_ground_removal/ransac_distance_threshold")
        self.dbscan_eps = rospy.get_param("/perception/lidar_clustering/dbscan_eps")
        self.dbscan_points = rospy.get_param("/perception/lidar_clustering/dbscan_min_points")

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
        X = points[:, 0:2] # x, y 좌표
        y = points[:, 2]   # z 좌표
        
        if len(points) < 10: # RANSAC을 위해 최소 포인트 수 확보
            return

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

    def cluster_points(self, points: np.ndarray, eps: float = 0.5, min_samples: int = 5) -> List[np.ndarray]:
        """Cluster points using DBSCAN"""
        if len(points) == 0:
            return []
        
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = db.labels_
        unique_labels = set(labels)
        
        clusters = []
        for label in unique_labels:
            if label == -1:
                continue
            cluster = points[labels == label]
            center = np.mean(cluster, axis=0)
            clusters.append(center[:3])  # Append only x, y, z
        
        return np.array(clusters)
    
    def left_right_split(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Split points into left and right based on y-coordinate"""
        left_points = points[points[:, 1] > 0]
        right_points = points[points[:, 1] <= 0]
        return left_points, right_points
    
    ### For Debugging: Publish Processed Point Cloud ###
    def publish_point_cloud(self, points: np.ndarray):
        """Publish processed point cloud"""

        filtered = self.filtering_points(points, (self.x_min, self.x_max), (self.y_min, self.y_max), (self.z_min, self.z_max))
        removal = self.ransac_plane_removal(filtered, threshold=self.ransac_distance, max_trials=self.ransac_iter)
        clusters = self.cluster_points(removal, eps=self.dbscan_eps, min_samples=self.dbscan_points)
        left, right = self.left_right_split(np.array(clusters))
        # rospy.loginfo_throttle(1.0, f"left = {left}, right = {right}")
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "fsds/FSCar"
        
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        point_cloud_msg = pc2.create_cloud(header, fields, clusters)
        self.lidar_publisher.publish(point_cloud_msg)

# ==================== Utility Classes ====================

class GPSProcessor:
    def __init__(self):
        self.origin_set = False
        self.origin_lat = rospy.get_param("/localization/localization/ref_wgs84_latitude", 0.0)
        self.origin_lon = rospy.get_param("/localization/localization/ref_wgs84_longitude", 0.0)
        self.origin_alt = rospy.get_param("/localization/localization/ref_wgs84_altitude", 0.0)
        self.R = 6378137.0  # WGS84 타원체의 반경 (미터 단위)

    ## Set origin GPS coordinates (relative to this point)
    def set_origin(self, lat: float, lon: float, alt: float):
        self.origin_lat = lat
        self.origin_lon = lon
        self.origin_alt = alt
        self.origin_set = True

    def gps_to_local(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        if not self.origin_set:
            raise ValueError("Origin GPS coordinates not set.")
        
        # print(self.origin_lat, self.origin_lon, self.origin_alt)
        # print(lat,lon,alt)
        
        d_lat = math.radians(lat - self.origin_lat)
        d_lon = math.radians(lon - self.origin_lon)
        
        x = d_lon * self.R * math.cos(math.radians(self.origin_lat))
        y = d_lat * self.R
        z = alt - self.origin_alt
        
        return x, y, z

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
            'gps_latitude', 'gps_longitude', 'gps_altitude',
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

        self.frame_count = 0
        rospy.loginfo(f"DataLogger initialized. Saving logs to: {self.session_path}")

    def log_entry(self, autonomous_mode: str, control_command: ControlCommand,
                  imu_acc: list, imu_gyro: list, gps_data: tuple,
                  camera1_image: np.ndarray, camera2_image: np.ndarray, lidar_points: np.ndarray):
        timestamp = rospy.Time.now().to_sec()

        # 각 프레임의 실제 LiDAR 포인트 개수 계산
        point_count = len(lidar_points) if lidar_points is not None else 0  # <--- 실제 포인트 개수 계산

        # 메타데이터 로깅 (point_count 포함)
        log_row = {
            'timestamp': timestamp, 'frame_id': self.frame_count, 'autonomous_mode': autonomous_mode,
            'control_steering': control_command.steering, 'control_throttle': control_command.throttle, 'control_brake': control_command.brake,
            'imu_acc_x': imu_acc[0], 'imu_acc_y': imu_acc[1], 'imu_acc_z': imu_acc[2],
            'imu_gyro_x': imu_gyro[0], 'imu_gyro_y': imu_gyro[1], 'imu_gyro_z': imu_gyro[2],
            'gps_latitude': gps_data[0], 'gps_longitude': gps_data[1], 'gps_altitude': gps_data[2],
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
            points_flat = lidar_points[:self.max_lidar_points, :3].flatten().tolist()
            lidar_row.extend(points_flat)
        
        expected_len = 1 + self.max_lidar_points * 3
        padding_len = expected_len - len(lidar_row)
        if padding_len > 0:
            lidar_row.extend([''] * padding_len)
        self.lidar_csv_writer.writerow(lidar_row)

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