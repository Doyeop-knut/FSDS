#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

# scikit-learn 라이브러리 import
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor

class LidarToCones:
    def __init__(self):
        rospy.init_node("lidar_to_cones_node", anonymous=True)

        self.lidar_sub = rospy.Subscriber("/fsds/lidar/Lidar1", PointCloud2, self.lidar_callback)
        self.cones_pub = rospy.Publisher("/cone_locations", PointCloud2, queue_size=1)
        # (디버깅용) 제거된 지면 포인트를 발행할 Publisher
        self.ground_pub = rospy.Publisher("/ground_plane", PointCloud2, queue_size=1)

        # 필터링 파라미터
        self.z_min = -0.5
        self.z_max = 0.5
        self.x_min = 1.0
        self.x_max = 20.0

        # DBSCAN 파라미터
        self.dbscan_eps = 0.4
        self.dbscan_min_samples = 3
        
        # === RANSAC 파라미터 추가 ===
        # 추정된 평면으로부터 이 거리(미터) 안에 있는 점을 지면(inlier)으로 간주
        self.ransac_threshold = 0.05 # 5cm

        # 색상 리스트
        self.colors = [
            (255, 0, 0),    # Red
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (255, 255, 0),  # Yellow
            (0, 255, 255),  # Cyan
            (255, 0, 255),  # Magenta
        ]

        rospy.loginfo("Lidar to Cones Node with RANSAC + DBSCAN Started.")
        rospy.spin()

    def lidar_callback(self, msg):
        points_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        filtered_points = []
        for p in points_generator:
            x, y, z = p[0], p[1], p[2]
            if self.x_min < x < self.x_max and self.z_min < z < self.z_max:
                filtered_points.append([x, y, z])

        if len(filtered_points) < 10: # RANSAC을 위해 최소 포인트 수 확보
            return

        points_np = np.array(filtered_points)

        # ================================================================ #
        # ================ RANSAC을 이용한 지면 제거 ======================= #
        # ================================================================ #
        
        # X, Y 좌표로 Z값을 예측하는 평면 모델을 찾습니다.
        X = points_np[:, 0:2] # x, y 좌표
        y = points_np[:, 2]   # z 좌표

        # RANSAC Regressor 모델 생성
        ransac = RANSACRegressor(
            residual_threshold=self.ransac_threshold,
            random_state=0
        )
        ransac.fit(X, y)
        
        # inlier_mask_는 지면(inlier)에 해당하는 포인트는 True, 아니면 False
        inlier_mask = ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)

        # 지면과 객체(콘 후보) 포인트를 분리
        ground_points = points_np[inlier_mask]
        object_points = points_np[outlier_mask]

        # (디버깅용) 제거된 지면 포인트를 발행
        if len(ground_points) > 0:
            header = msg.header
            ground_cloud_msg = pc2.create_cloud_xyz32(header, ground_points)
            self.ground_pub.publish(ground_cloud_msg)
        
        # ================================================================ #
        
        # 지면이 제거된 포인트(object_points)가 충분하지 않으면 DBSCAN을 수행하지 않습니다.
        if len(object_points) < self.dbscan_min_samples:
            # 검출된 콘이 없을 때 빈 메시지를 보내 이전 마커를 지우고 싶다면 아래 주석 해제
            # empty_cloud_msg = pc2.create_cloud(msg.header, [], [])
            # self.cones_pub.publish(empty_cloud_msg)
            return

        # 지면이 제거된 포인트들에 대해서만 DBSCAN 클러스터링 수행
        db = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(object_points)
        labels = db.labels_
        unique_labels = set(labels)
        
        colored_cone_centers = []
        for label in unique_labels:
            if label == -1:
                continue

            color_rgb = self.colors[label % len(self.colors)]
            r, g, b = color_rgb
            rgb_packed = (r << 16) | (g << 8) | b

            class_member_mask = (labels == label)
            cluster_points = object_points[class_member_mask]
            center = np.mean(cluster_points, axis=0)
            
            colored_cone_centers.append([center[0], center[1], center[2], rgb_packed])
        
        if colored_cone_centers:
            header = msg.header
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.UINT32, 1),
            ]
            cones_cloud_msg = pc2.create_cloud(header, fields, colored_cone_centers)
            self.cones_pub.publish(cones_cloud_msg)

if __name__ == '__main__':
    try:
        LidarToCones()
    except rospy.ROSInterruptException:
        pass