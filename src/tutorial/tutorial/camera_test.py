#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImgTest(Node):
    def __init__(self):
        super().__init__("Image_test")
        qos_profile = QoSProfile(depth=1)
        self.cam1_sub = self.create_subscription(Image,
                                                 "/fsds/cameracam1/image_color",
                                                 self.camera1_callback,
                                                 qos_profile)
        self.cam2_sub = self.create_subscription(Image,
                                                 "/fsds/cameracam2/image_color",
                                                 self.camera2_callback,
                                                 qos_profile)
        self.bridge = CvBridge()
        self.image1, self.image2 = None, None

    def camera1_callback(self,msg):
        img1 = msg
        self.image1 = self.bridge.imgmsg_to_cv2(img1,'bgr8')
           
    def camera2_callback(self,msg):
        # pass
        img2 = msg
        self.image2 = self.bridge.imgmsg_to_cv2(img2,'bgr8')

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self,timeout_sec = 0.01)
                if self.image1 is not None:
                    cv2.imshow("Camera 1", self.image1)
                if self.image2 is not None:
                    cv2.imshow("Camera 2", self.image2)
                cv2.waitKey(1)
        finally:
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ImgTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

