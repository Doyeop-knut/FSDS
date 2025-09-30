#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import message_filters

class StereoRectifier:
    def __init__(self):
        rospy.init_node('stereo_rectifier_node', anonymous=True)

        self.bridge = CvBridge()

        # --- 파라미터 불러오기 ---
        # 캘리브레이션 파일 경로를 ROS 파라미터 서버에서 가져옵니다. (launch 파일에서 설정)
        calibration_file = rospy.get_param('~calibration_file', '')
        if not calibration_file:
            rospy.logerr("캘리브레이션 파일 경로가 설정되지 않았습니다. launch 파일을 확인하세요.")
            return

        # 구독할 원본 이미지 토픽 이름
        left_topic = rospy.get_param('~left_image_topic', '/fsds/cameracam1')
        right_topic = rospy.get_param('~right_image_topic', '/fsds/cameracam2')

        # --- 캘리브레이션 데이터 로드 ---
        try:
            calib_data = np.load(calibration_file)
            self.M1, self.D1 = calib_data['M1'], calib_data['D1']
            self.M2, self.D2 = calib_data['M2'], calib_data['D2']
            self.R, self.T = calib_data['R'], calib_data['T']
        except Exception as e:
            rospy.logerr(f"캘리브레이션 파일 로드 실패: {e}")
            return

        # 렉티피케이션 맵 (처음 이미지를 받으면 계산됨)
        self.map1_left, self.map2_left = None, None
        self.map1_right, self.map2_right = None, None
        
        # --- Publisher 설정 ---
        # 개별적으로 정렬된 이미지를 발행
        self.pub_left_rect = rospy.Publisher('/stereo/left/image_rect', Image, queue_size=1)
        self.pub_right_rect = rospy.Publisher('/stereo/right/image_rect', Image, queue_size=1)
        # 확인용으로 합쳐진 이미지를 발행
        self.pub_combined = rospy.Publisher('/stereo/image_rect_combined', Image, queue_size=1)

        # --- Subscriber 설정 (Message Filters 사용) ---
        sub_left = message_filters.Subscriber(left_topic, Image)
        sub_right = message_filters.Subscriber(right_topic, Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([sub_left, sub_right], 10, 0.1)
        self.ts.registerCallback(self.callback)

        rospy.loginfo("Stereo Rectifier 노드가 시작되었습니다.")
        rospy.loginfo(f"구독 중인 토픽: {left_topic}, {right_topic}")

    def callback(self, left_msg, right_msg):
        try:
            cv_left = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
            cv_right = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 첫 프레임에서 렉티피케이션 맵을 계산 (매우 효율적)
        if self.map1_left is None:
            rospy.loginfo("첫 이미지 수신, 렉티피케이션 맵을 계산합니다.")
            h, w = cv_left.shape[:2]
            image_size = (w, h)
            R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
                self.M1, self.D1, self.M2, self.D2, image_size, self.R, self.T, alpha=0)
            self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(self.M1, self.D1, R1, P1, image_size, cv2.CV_32FC1)
            self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(self.M2, self.D2, R2, P2, image_size, cv2.CV_32FC1)

        # 맵을 적용하여 이미지 정렬
        rectified_left = cv2.remap(cv_left, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
        rectified_right = cv2.remap(cv_right, self.map1_right, self.map2_right, cv2.INTER_LINEAR)

        # --- 결과 발행 ---
        try:
            # 개별 정렬 이미지 발행
            rect_left_msg = self.bridge.cv2_to_imgmsg(rectified_left, "bgr8")
            rect_left_msg.header = left_msg.header
            self.pub_left_rect.publish(rect_left_msg)

            rect_right_msg = self.bridge.cv2_to_imgmsg(rectified_right, "bgr8")
            rect_right_msg.header = right_msg.header
            self.pub_right_rect.publish(rect_right_msg)

            # 확인용 합쳐진 이미지 발행
            combined_image = np.hstack((rectified_left, rectified_right))
            h, w, _ = combined_image.shape
            for i in range(20, h, 60):
                cv2.line(combined_image, (0, i), (w, i), (0, 255, 0), 1)
            
            combined_msg = self.bridge.cv2_to_imgmsg(combined_image, "bgr8")
            combined_msg.header = left_msg.header
            self.pub_combined.publish(combined_msg)

        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        StereoRectifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("노드가 종료됩니다.")