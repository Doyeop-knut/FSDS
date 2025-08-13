#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class LidarDisplayNode(Node):
    def __init__(self):
        super().__init__('lidar_display_node')
        self.get_logger().info('Lidar Display Node started.')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/fsds/lidar_points',
            self.lidar_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        # Callback 함수는 데이터를 받기만 하고, 특별한 처리는 하지 않음
        # Rviz2가 PointCloud2 메시지를 직접 시각화하므로, 이 노드는 단순히 구독자 역할만 합니다.
        pass

def main(args=None):
    rclpy.init(args=args)
    lidar_display_node = LidarDisplayNode()
    rclpy.spin(lidar_display_node)
    lidar_display_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()