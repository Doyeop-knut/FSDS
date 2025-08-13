#!/usr/bin/env python3
import rospy
import math
import numpy as np
from fs_msgs.msg import ControlCommand
from fs_msgs.msg import ConeArray, Cone
from visualization_msgs.msg import Marker # Marker 메시지 임포트
from geometry_msgs.msg import Point # Marker의 위치를 설정하기 위해 Point 임포트

class FSDSPathPlanner:
    def __init__(self):
        rospy.init_node("fsds_path_planner", anonymous=True)
        
        self.control_pub = rospy.Publisher("/fsds/control_command", ControlCommand, queue_size=1)
        self.cones_sub = rospy.Subscriber("/cones", ConeArray, self.cones_callback)
        # 마커를 발행할 퍼블리셔 추가
        self.marker_pub = rospy.Publisher("/path_marker", Marker, queue_size=1)
        
        self.control_msg = ControlCommand()
        self.throttle_value = 0.5
        
        rospy.loginfo("FSDS Path Planner Node Started. Waiting for cone data...")
        rospy.spin()

    def cones_callback(self, msg):
        left_cones = []
        right_cones = []
        
        for cone in msg.cones:
            x = cone.location.x
            y = cone.location.y
            
            if y > 0:
                left_cones.append((x, y))
            else:
                right_cones.append((x, y))
        
        self.plan_and_control(left_cones, right_cones)

    def plan_and_control(self, left_cones, right_cones):
        if len(left_cones) > 0 and len(right_cones) > 0:
            left_center = np.mean(left_cones, axis=0)
            right_center = np.mean(right_cones, axis=0)
            
            target_point_x = (left_center[0] + right_center[0]) / 2.0
            target_point_y = (left_center[1] + right_center[1]) / 2.0
            
            angle_to_target = math.atan2(target_point_y, target_point_x)
            
            self.control_msg.throttle = self.throttle_value
            self.control_msg.steering = angle_to_target * 1.0
            self.control_pub.publish(self.control_msg)
            
            rospy.loginfo(f"Target: ({target_point_x:.2f}, {target_point_y:.2f}) -> Steering: {self.control_msg.steering:.2f}")

            # Rviz에 마커를 발행하는 부분
            marker = Marker()
            marker.header.frame_id = "base_link" # Rviz의 Fixed Frame과 동일하게 설정
            marker.header.stamp = rospy.Time.now()
            marker.ns = "path_planner"
            marker.id = 0
            marker.type = Marker.SPHERE # 점(구) 형태로 표시
            marker.action = Marker.ADD
            marker.pose.position.x = target_point_x
            marker.pose.position.y = target_point_y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5 # 마커 크기
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0 # 투명도
            marker.color.r = 1.0 # 빨간색
            self.marker_pub.publish(marker)

        else:
            self.control_msg.throttle = 0.0
            self.control_msg.steering = 0.0
            self.control_pub.publish(self.control_msg)
            rospy.logwarn("No cones detected. Vehicle stopped.")

if __name__ == '__main__':
    try:
        FSDSPathPlanner()
    except rospy.ROSInterruptException:
        pass