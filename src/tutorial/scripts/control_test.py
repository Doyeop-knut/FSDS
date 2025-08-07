#!/usr/bin/env python3
import rospy, math
from fs_msgs.msg import ControlCommand

class ControlTest:
    def __init__(self):
        rospy.init_node("control_test",anonymous = True)
        self.control_pub = rospy.Publisher("/fsds/control_command",ControlCommand,1)
        control_msg = ControlCommand()
        while not rospy.is_shutdown():
            control_msg.throttle = 0.5
            control_msg.steering = -0.5
            self.control_pub.publish(control_msg)
            print(control_msg)



if __name__ == '__main__':
    try:
        gss_test = ControlTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


