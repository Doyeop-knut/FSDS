#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from fs_msgs.msg import ControlCommand

class ControlTest(Node):
    def __init__(self):
        super().__init__("control_test")
        qos_profile=QoSProfile(depth=1)
        self.control_pub = self.create_publisher(ControlCommand,
                                                 "/fsds/control_command",
                                                 qos_profile)
        control_msg = ControlCommand()
        while rclpy.ok():
            control_msg.throttle = 0.5
            control_msg.steering = -0.5
            self.control_pub.publish(control_msg)
            print(control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

