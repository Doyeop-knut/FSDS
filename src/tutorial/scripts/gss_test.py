#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist
from sensor_msgs.msg import NavSatFix, Imu

class GSS_TEST:
    def __init__(self):
        rospy.init_node("gss_test",anonymous = True)
        self.gss_sub = rospy.Subscriber("/fsds/gss",TwistWithCovarianceStamped, self.gss_callback)
        self.gps_sub = rospy.Subscriber("/fsds/gps",NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/fsds/imu",Imu, self.imu_callback)

        self.is_gss = False
        self.is_gps = False
    def gss_callback(self,msg):
        self.is_gss = True
        gss_msg = msg
    
    def gps_callback(self,msg):
        self.is_gps = True
        gps_msg = msg
    def imu_callback(self,msg):
        self.is_imu = True
        imu_msg = msg
        print(imu_msg)



if __name__ == '__main__':
    try:
        gss_test = GSS_TEST()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


