#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class ImgTSET:
    def __init__(self):
        rospy.init_node("Image_test",anonymous = True)
        self.bridge = CvBridge()
        self.model = YOLO('yo5.pt')
        self.gss_sub = rospy.Subscriber("/fsds/cameracam1",Image, self.camera1_callback)
        self.gps_sub = rospy.Subscriber("/fsds/cameracam2",Image, self.camera2_callback)
        rospy.loginfo("YOLO started")


    @staticmethod
    def color_counter(cv_image):
        if cv_image is None or cv_image.size == 0:
            return 'unknown'
        hsv = cv2.cvtColor(cv_image , cv2.COLOR_BGR2HSV)

        red_lower1, red_upper1 = np.array([0, 70, 50]), np.array([10, 255, 255])
        red_lower2, red_upper2 = np.array([170, 70, 50]), np.array([180, 255, 255])
        # 파란색 범위
        blue_lower1, blue_upper1 = np.array([100,120, 40]), np.array([120,255,255])
        blue_lower2, blue_upper2 = np.array([120, 60,180]), np.array([140,255,255])
        # 노란색 범위
        yellow_lower1, yellow_upper1 = np.array([15,100, 40]), np.array([25,255,255])
        yellow_lower2, yellow_upper2 = np.array([25, 50,180]), np.array([40,255,255])

        red_mask = cv2.inRange(hsv, red_lower1, red_upper1) | cv2.inRange(hsv, red_lower2, red_upper2)
        blue_mask = cv2.inRange(hsv, blue_lower1, blue_upper1) | cv2.inRange(hsv, blue_lower2, blue_upper2)
        yellow_mask = cv2.inRange(hsv, yellow_lower1, yellow_upper1) | cv2.inRange(hsv, yellow_lower2, yellow_upper2)

        red_pixel_count = cv2.countNonZero(red_mask)
        blue_pixel_count = cv2.countNonZero(blue_mask)
        yellow_pixel_count = cv2.countNonZero(yellow_mask)

        color_counts = {
        'red': red_pixel_count, 'blue': blue_pixel_count, 'yellow': yellow_pixel_count
    }
        if all(count == 0 for count in color_counts.values()):
            return 'unknown'
        return max(color_counts, key=color_counts.get)
    
    def cone_detect(self, cv_image, window_name):
        results = self.model(cv_image)[0]

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf =box.conf[0]
            cls = int(box.cls[0])
            crop_box = cv_image[y1:y2, x1:x2]
            cone_color = self.color_counter(crop_box)

            label = f'{self.model.names[cls]}{cone_color}{conf:.2f}'
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow(window_name, cv_image)
        cv2.waitKey(1)
            

    def camera1_callback(self,msg):
        img1 = msg
        cv_image = self.bridge.imgmsg_to_cv2(img1, 'bgr8')
        self.cone_detect(cv_image, "Camera1")
           
    def camera2_callback(self,msg):
        # pass
        img2 = msg
        cv_image = self.bridge.imgmsg_to_cv2(img2, 'bgr8')
        self.cone_detect(cv_image, "Camera2")

if __name__ == '__main__':
    try:
        img_test = ImgTSET()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass        
    finally:
        cv2.destroyAllWindows()

