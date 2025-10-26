#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ConeColorDetector:
    def __init__(self):
        rospy.init_node('cone_color_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/fsds/cameracam1", Image, self.image_callback)
        rospy.loginfo("Final Cone Detector node has been started.")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ---  최종 튜닝값들 ---
        # 🔵 파란색 (Blue)
        lower_blue = np.array([99, 94, 135])
        upper_blue = np.array([114, 255, 255])

        # 🟡 노란색 (Yellow)
        lower_yellow = np.array([28, 25, 144])
        upper_yellow = np.array([35, 255, 255])

        # 🟠 주황색 (Orange)
        lower_orange = np.array([2, 44, 98])
        upper_orange = np.array([20, 255, 255])
        # ------------------------------------

        # 각 색상에 대한 마스크 생성
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
        
        # 모든 마스크를 하나로 합치기
        combined_mask = cv2.bitwise_or(blue_mask, yellow_mask)
        combined_mask = cv2.bitwise_or(combined_mask, orange_mask)

        # 원본 이미지에 마스크를 적용하여 결과물 생성
        result_image = cv2.bitwise_and(cv_image, cv_image, mask=combined_mask)

        # 결과 화면들을 화면에 표시
        cv2.imshow("Original View", cv_image)
        cv2.imshow("Final Combined Mask", combined_mask)
        cv2.imshow("Final Result", result_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User pressed 'q' to exit.")

if __name__ == '__main__':
    try:
        detector = ConeColorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()