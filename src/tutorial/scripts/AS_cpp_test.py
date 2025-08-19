#!/usr/bin/env python3
import rospy, math, cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImgTSET:
    def __init__(self):
        rospy.init_node("Image_test",anonymous = True)
        self.gss_sub = rospy.Subscriber("/fsds/projected_cones_image",Image, self.camera1_callback)
        # self.gps_sub = rospy.Subscriber("/fsds/cameracam2",Image, self.camera2_callback)
        self.bridge = CvBridge()
        self.image1, self.image2 = None, None
        while not rospy.is_shutdown():
            if self.image1 is not None:
                cv2.imshow("Camera 1", self.image1)
            if self.image2 is not None:
                cv2.imshow("Camera 2", self.image2)
            cv2.waitKey(1)

        cv2.destroyAllWindows()
    def camera1_callback(self,msg):
        img1 = msg
        self.image1 = self.bridge.imgmsg_to_cv2(img1,'bgr8')
           
    def camera2_callback(self,msg):
        # pass
        img2 = msg
        self.image2 = self.bridge.imgmsg_to_cv2(img2,'bgr8')

if __name__ == '__main__':
    try:
        img_test = ImgTSET()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass        

