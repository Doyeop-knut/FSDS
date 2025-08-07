#!/usr/bin/env python3
import rospy, math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class LiDARTest:
    def __init__(self):
        rospy.init_node("lidar_test",anonymous = True)
        self.lidar_sub = rospy.Subscriber("/fsds/lidar/Lidar1",PointCloud2, self.lidar_callback)

    def lidar_callback(self,msg):
        lidar_msg = msg
        for point in pc2.read_points(lidar_msg, skip_nans=True):
            x,y,z = point[0],point[1],point[2]
            print(f"x = {x} \n y= {y} \n z = {z}")
            
if __name__ == '__main__':
    try:
        lidar_test = LiDARTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


