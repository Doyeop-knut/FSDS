#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
# ConeArray 관련 import는 제거합니다.
# from fs_msgs.msg import Cone, ConeArray
# from geometry_msgs.msg import Point

class LidarVisualizer:
    def __init__(self):
        rospy.init_node("lidar_visualizer_node", anonymous=True)

        # Subscriber는 그대로 유지합니다.
        self.lidar_sub = rospy.Subscriber("/fsds/lidar/Lidar1", PointCloud2, self.lidar_callback)

        # ConeArray Publisher 대신 PointCloud2 Publisher를 새로 만듭니다.
        # 이 Publisher가 RViz에서 볼 필터링된 포인트 클라우드를 발행합니다.
        self.filtered_pub = rospy.Publisher("/filtered_points", PointCloud2, queue_size=1)

        # 필터링 조건은 그대로 사용합니다.
        self.z_min = -0.5
        self.z_max = 0.5
        self.x_min = 1.0
        self.x_max = 20.0
        # 클러스터링 거리는 더 이상 필요 없습니다.
        # self.cluster_dist = 0.5

        rospy.loginfo("Lidar Visualizer Node Started.")
        rospy.spin()

    def lidar_callback(self, msg):
        # PointCloud2 메시지에서 (x, y, z) 필드를 읽어옵니다.
        points_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # 조건에 맞는 점들만 저장할 리스트
        filtered_points = []
        for p in points_generator:
            x, y, z = p[0], p[1], p[2]

            # 기존의 필터링 조건을 그대로 적용합니다.
            if self.x_min < x < self.x_max and self.z_min < z < self.z_max:
                # x, y, z 좌표를 모두 저장합니다.
                filtered_points.append([x, y, z])

        # 필터링된 점이 없으면 아무 작업도 하지 않습니다.
        if not filtered_points:
            return

        # ================================================================ #
        # === 클러스터링 및 ConeArray 생성 로직을 이 부분으로 대체합니다 === #
        # ================================================================ #

        # 1. 헤더(Header)를 원본 메시지에서 가져옵니다.
        #    - header에는 timestamp와 frame_id 정보가 있어 RViz에서 좌표계를 맞추는 데 필수적입니다.
        header = msg.header

        # 2. 필터링된 포인트 리스트를 사용하여 새로운 PointCloud2 메시지를 생성합니다.
        #    - 'create_cloud_xyz32' 함수는 (x, y, z) 데이터를 가진 PointCloud2 메시지를 쉽게 만들어줍니다.
        filtered_cloud_msg = pc2.create_cloud_xyz32(header, filtered_points)

        # 3. 새로 생성된 PointCloud2 메시지를 발행합니다.
        self.filtered_pub.publish(filtered_cloud_msg)


if __name__ == '__main__':
    try:
        LidarVisualizer()
    except rospy.ROSInterruptException:
        pass