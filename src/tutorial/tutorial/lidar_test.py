#!/usr/bin/env python3
import math 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class LiDARTest(Node):
    def __init__(self):
        super().__init__("lidar_test")
        self.lidar_sub = self.create_subscription(PointCloud2,
                                                  "/fsds/lidar/Lidar1",
                                                  self.lidar_callback,
                                                  QoSProfile(depth=1))
    def lidar_callback(self,msg):
        lidar_msg = msg
        for point in pc2.read_points(lidar_msg, skip_nans=True):
            x,y,z = point[0],point[1],point[2]
            print(f"x = {x} \n y= {y} \n z = {z}")

def main(args=None):
    rclpy.init(args=args)
    node = LiDARTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

