#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# message_filters를 import하여 메시지 동기화를 처리합니다.
import message_filters

class StereoCombiner:
    def __init__(self):
        rospy.init_node('stereo_image_combiner_node', anonymous=True)

        self.bridge = CvBridge()
        
        # 발행할 스테레오 이미지 Publisher 생성
        self.stereo_pub = rospy.Publisher("/stereo_image", Image, queue_size=1)
        
        # === 구독할 두 개의 이미지 토픽 이름 ===
        # 실제 환경에 맞게 토픽 이름을 변경해주세요.
        left_image_topic = "/fsds/cameracam1"
        right_image_topic = "/fsds/cameracam2"
        
        # --- Message Filters를 사용한 동기화 구독 ---
        
        # 1. 각 토픽에 대한 Subscriber를 생성합니다.
        left_sub = message_filters.Subscriber(left_image_topic, Image)
        right_sub = message_filters.Subscriber(right_image_topic, Image)
        
        # 2. TimeSynchronizer를 사용하여 두 토픽을 타임스탬프 기준으로 동기화합니다.
        #    - [left_sub, right_sub]: 동기화할 Subscriber 리스트
        #    - 10: 큐 사이즈
        #    - 0.1: 메시지 간의 최대 시간 차이(초). 이 시간 내에 들어온 메시지 쌍만 유효 처리
        self.ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], 10, 0.1)
        
        # 3. 동기화된 메시지를 처리할 콜백 함수를 등록합니다.
        self.ts.registerCallback(self.callback)
        
        rospy.loginfo("Stereo image combiner node started...")
        rospy.loginfo(f"Subscribing to {left_image_topic} and {right_image_topic}")

    def callback(self, left_ros_image, right_ros_image):
        """
        두 이미지가 동기화되어 수신되었을 때 호출되는 콜백 함수
        """
        rospy.loginfo("Received a synchronized pair of images!")
        try:
            # CvBridge를 사용하여 ROS Image 메시지를 OpenCV 이미지로 변환
            left_cv_image = self.bridge.imgmsg_to_cv2(left_ros_image, "bgr8")
            right_cv_image = self.bridge.imgmsg_to_cv2(right_ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 두 이미지의 높이가 다를 경우, 작은 쪽에 맞춰 리사이즈
        h1, w1 = left_cv_image.shape[:2]
        h2, w2 = right_cv_image.shape[:2]
        if h1 != h2:
            min_h = min(h1, h2)
            left_cv_image = cv2.resize(left_cv_image, (int(w1 * min_h / h1), min_h))
            right_cv_image = cv2.resize(right_cv_image, (int(w2 * min_h / h2), min_h))
            
        # OpenCV(NumPy)를 사용하여 두 이미지를 수평으로 합칩니다 (Side-by-Side).
        stereo_image_cv = np.hstack((left_cv_image, right_cv_image))
        cv2.imshow("Stereo Image", stereo_image_cv)
        cv2.waitKey(1)
        
        # 합쳐진 이미지를 다시 ROS Image 메시지로 변환하여 발행
        try:
            stereo_ros_image = self.bridge.cv2_to_imgmsg(stereo_image_cv, "bgr8")
            # 헤더의 타임스탬프를 원본 메시지 중 하나와 일치시켜줍니다.
            stereo_ros_image.header.stamp = left_ros_image.header.stamp 
            self.stereo_pub.publish(stereo_ros_image)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        StereoCombiner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass