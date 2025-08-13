#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from fs_msgs.msg import Cone, ConeArray
from geometry_msgs.msg import Point

class LidarToCones:
    def __init__(self):
        rospy.init_node("lidar_to_cones_node", anonymous=True)
        
        self.lidar_sub = rospy.Subscriber("/fsds/lidar/Lidar1", PointCloud2, self.lidar_callback)
        self.cones_pub = rospy.Publisher("/cones", ConeArray, queue_size=1)
        
        self.z_min = -0.5
        self.z_max = 0.5
        self.x_min = 1.0
        self.x_max = 20.0
        self.cluster_dist = 0.5
        
        rospy.loginfo("Lidar to Cones Node Started.")
        rospy.spin()

    def lidar_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        filtered_points = []
        for p in points:
            x, y, z = p[0], p[1], p[2]
            
            if self.x_min < x < self.x_max and self.z_min < z < self.z_max:
                filtered_points.append((x, y))

        if not filtered_points:
            return

        sorted_y = sorted(filtered_points, key=lambda p: p[1])
        cones_positions = []
        
        if sorted_y:
            current_cluster = [sorted_y[0]]
            for i in range(1, len(sorted_y)):
                if abs(sorted_y[i][1] - sorted_y[i-1][1]) < self.cluster_dist:
                    current_cluster.append(sorted_y[i])
                else:
                    cones_positions.append(np.mean(current_cluster, axis=0))
                    current_cluster = [sorted_y[i]]
            cones_positions.append(np.mean(current_cluster, axis=0))

        cones_msg = ConeArray()
        for cone_pos in cones_positions:
            cone = Cone()
            cone.location.x = cone_pos[0]
            cone.location.y = cone_pos[1]
            cone.location.z = 0.0
            cone.color = 4
            cones_msg.cones.append(cone)
            
        self.cones_pub.publish(cones_msg)
        
if __name__ == '__main__':
    try:
        LidarToCones()
    except rospy.ROSInterruptException:
        pass