#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

# DBSCAN을 위해 scikit-learn 라이브러리를 import 합니다.
from sklearn.cluster import DBSCAN

class LidarToCones:
    def __init__(self):
        rospy.init_node("lidar_to_cones_node", anonymous=True)

        self.lidar_sub = rospy.Subscriber("/fsds/lidar/Lidar1", PointCloud2, self.lidar_callback)
        self.cones_pub = rospy.Publisher("/cone_locations", PointCloud2, queue_size=1)

        # 필터링 파라미터
        self.z_min = -0.5
        self.z_max = 0.5
        self.x_min = 1.0
        self.x_max = 20.0

        # DBSCAN 파라미터
        self.dbscan_eps = 0.4
        self.dbscan_min_samples = 3

        # === 색상 리스트 추가 ===
        # 클러스터마다 다른 색상을 할당하기 위한 RGB 값 리스트
        # (R, G, B) - 값 범위: 0-255
        self.colors = [
            (255, 0, 0),    # Red
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (255, 255, 0),  # Yellow
            (0, 255, 255),  # Cyan
            (255, 0, 255),  # Magenta
        ]

        rospy.loginfo("Lidar to Cones Node with DBSCAN (Colored) Started.")
        rospy.spin()

    def lidar_callback(self, msg):
        points_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        filtered_points = []
        for p in points_generator:
            x, y, z = p[0], p[1], p[2]
            if self.x_min < x < self.x_max and self.z_min < z < self.z_max:
                filtered_points.append([x, y, z])

        if len(filtered_points) < self.dbscan_min_samples:
            return

        points_np = np.array(filtered_points)

        # DBSCAN 클러스터링 수행
        db = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points_np)
        labels = db.labels_
        unique_labels = set(labels)
        
        # [x, y, z, rgb] 데이터를 담을 리스트
        colored_cone_centers = []
        
        for label in unique_labels:
            if label == -1:
                continue

            # === 클러스터별 색상 할당 ===
            # 라벨 번호를 사용하여 색상 리스트에서 색을 선택 (순환하도록 % 연산자 사용)
            color_rgb = self.colors[label % len(self.colors)]
            r, g, b = color_rgb
            
            # RGB 값을 하나의 32비트 정수로 패킹합니다.
            # (R << 16) | (G << 8) | B
            rgb_packed = (r << 16) | (g << 8) | b

            class_member_mask = (labels == label)
            cluster_points = points_np[class_member_mask]
            center = np.mean(cluster_points, axis=0)
            
            # 리스트에 [x, y, z, packed_rgb] 형태로 추가
            colored_cone_centers.append([center[0], center[1], center[2], rgb_packed])
        
        if colored_cone_centers:
            header = msg.header

            # === PointCloud2 필드 정의 (rgb 추가) ===
            # 각 포인트가 어떤 데이터 필드를 갖는지 정의합니다.
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.UINT32, 1),
            ]
            
            # create_cloud 함수를 사용하여 x, y, z와 rgb 정보를 모두 포함하는 메시지를 생성합니다.
            cones_cloud_msg = pc2.create_cloud(header, fields, colored_cone_centers)
            self.cones_pub.publish(cones_cloud_msg)

if __name__ == '__main__':
    try:
        LidarToCones()
    except rospy.ROSInterruptException:
        pass