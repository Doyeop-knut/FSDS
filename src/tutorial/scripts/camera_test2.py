#!/usr/bin/env python3
import rospy, cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters

class RectifiedStereoViewer:
    def __init__(self):
        rospy.init_node("rectified_stereo_viewer", anonymous=True)

        self.bridge = CvBridge()
        self.stereo_image_pub = rospy.Publisher("/stereo_camera/image_rect_combined", Image, queue_size=1)

        # Subscribers for the two rectified camera topics
        # These topics are typically published by stereo_image_proc
        image_sub1 = message_filters.Subscriber("/stereo/left/image_rect_color", Image)
        image_sub2 = message_filters.Subscriber("/stereo/right/image_rect_color", Image)

        # Synchronize the topics
        ts = message_filters.ApproximateTimeSynchronizer([image_sub1, image_sub2], 10, 0.1)
        ts.registerCallback(self.image_callback)

        self.stereo_image = None
        rospy.loginfo("Rectified stereo viewer node started, subscribing to /stereo/left/image_rect_color and /stereo/right/image_rect_color")


    def image_callback(self, msg1, msg2):
        try:
            # Convert ROS Image messages to OpenCV images
            cv_image1 = self.bridge.imgmsg_to_cv2(msg1, "bgr8")
            cv_image2 = self.bridge.imgmsg_to_cv2(msg2, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
            return

        # Combine the two images horizontally
        self.stereo_image = np.hstack((cv_image1, cv_image2))

        # Display the stereo image
        cv2.imshow("Rectified Stereo Image", self.stereo_image)
        cv2.waitKey(1)

        try:
            # Convert the combined OpenCV image back to a ROS Image message and publish
            stereo_msg = self.bridge.cv2_to_imgmsg(self.stereo_image, "bgr8")
            stereo_msg.header.stamp = rospy.Time.now() # Use current time
            self.stereo_image_pub.publish(stereo_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing image: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        viewer = RectifiedStereoViewer()
        viewer.run()
    except rospy.ROSInterruptException:
        pass