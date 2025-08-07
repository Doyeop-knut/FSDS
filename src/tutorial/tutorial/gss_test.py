#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist
from sensor_msgs.msg import NavSatFix, Imu

class GSS_TEST(Node):
    def __init__(self):
        super().__init__("gss_test")
        self.gss_sub = self.create_subscription(TwistWithCovarianceStamped,
                                                 "/fsds/gss",
                                                 self.gss_callback,
                                                 QoSProfile(depth = 1))
        self.gps_sub = self.create_subscription(NavSatFix,
                                                 "/fsds/gps",
                                                 self.gps_callback,
                                                 QoSProfile(depth=1))
        self.imu_sub = self.create_subscription(Imu,
                                                  "/fsds/imu",
                                                  self.imu_callback,
                                                  QoSProfile(depth=1))

        self.is_gss = False
        self.is_gps = False
    def gss_callback(self,msg):
        self.is_gss = True
        gss_msg = msg
        # print(gss_msg)
    
    def gps_callback(self,msg):
        self.is_gps = True
        gps_msg = msg
        # print(gps_msg)

    def imu_callback(self,msg):
        self.is_imu = True
        imu_msg = msg
        print(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GSS_TEST()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

