#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def nothing(x):
    pass

class HSVTuner:
    def __init__(self):
        rospy.init_node('hsv_tuner', anonymous=True)
        self.bridge = CvBridge()
        
        # 콜백 함수는 최신 이미지를 self.cv_image에 저장하는 역할만 합니다.
        self.image_sub = rospy.Subscriber("/fsds/cameracam1", Image, self.image_callback)
        self.cv_image = None
        
        # 튜닝을 위한 윈도우와 트랙바 생성
        cv2.namedWindow('HSV Tuner')
        cv2.createTrackbar('H_min', 'HSV Tuner', 0, 179, nothing)
        cv2.createTrackbar('H_max', 'HSV Tuner', 179, 179, nothing)
        cv2.createTrackbar('S_min', 'HSV Tuner', 0, 255, nothing)
        cv2.createTrackbar('S_max', 'HSV Tuner', 255, 255, nothing)
        cv2.createTrackbar('V_min', 'HSV Tuner', 0, 255, nothing)
        cv2.createTrackbar('V_max', 'HSV Tuner', 255, 255, nothing)
        
        rospy.loginfo("HSV Tuner node has been started. Adjust the trackbars.")

    def image_callback(self, data):
        # ROS 메시지를 받아서 OpenCV 이미지로 변환 후 저장만 함
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(e)

    def run(self):
        # 메인 루프에서 모든 GUI 처리와 이미지 처리를 담당
        rate = rospy.Rate(30) # 30Hz
        while not rospy.is_shutdown():
            # 이미지가 한 번이라도 수신되었는지 확인
            if self.cv_image is not None:
                hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

                # 트랙바에서 현재 값들을 가져오기
                h_min = cv2.getTrackbarPos('H_min', 'HSV Tuner')
                h_max = cv2.getTrackbarPos('H_max', 'HSV Tuner')
                s_min = cv2.getTrackbarPos('S_min', 'HSV Tuner')
                s_max = cv2.getTrackbarPos('S_max', 'HSV Tuner')
                v_min = cv2.getTrackbarPos('V_min', 'HSV Tuner')
                v_max = cv2.getTrackbarPos('V_max', 'HSV Tuner')

                lower_bound = np.array([h_min, s_min, v_min])
                upper_bound = np.array([h_max, s_max, v_max])

                mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
                result_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)

                # 화면에 표시
                cv2.imshow("Original View", self.cv_image)
                cv2.imshow("Mask", mask)
                cv2.imshow("Result", result_image)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            rate.sleep()
        
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tuner = HSVTuner()
        tuner.run() # 메인 루프 실행
    except rospy.ROSInterruptException:
        pass