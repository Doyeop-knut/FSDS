# quick_validation.py
import numpy as np
import math
import json
from collections import deque

# 만약 실제 ROS 메시지 타입이 필요하면 아래 import 사용
# import rospy
# from std_msgs.msg import String
# from sensor_msgs.msg import Image, PointCloud2
# from formula_autonomous_system import FormulaAutonomousSystem

# simple_dummy.py
class DummyHeader:
    def __init__(self, t=0.0):
        self._t = t
    def to_sec(self):
        return float(self._t)

class DummyMsg:
    def __init__(self, t=0.0, pos=(0.0, 0.0, 0.0)):
        self.header = DummyHeader(t)
        self.pos = pos

    def run_sim(system, frames=200, dt=0.05):
        # 간단한 더미 메시지 생성
        lidar_msg = DummyMsg(0.0)
        cam1_msg  = DummyMsg(dt)
        cam2_msg  = DummyMsg(dt)
        imu_msg   = DummyMsg(dt)
        gps_msg   = DummyMsg(dt, pos=(0.0,0.0,0.0))
        go_signal_msg = DummyMsg(dt)

        # KPI 수집
        stats = {
            "centerline_error_mean": [],
            "centerline_error_max": [],
            "yaw_error_mean": [],
            "yaw_error_max": [],
            "fallback_count": 0,
            "path_changes": 0,
            "frame": []
        }

        for frame in range(frames):
            # 여기에 actual system.run 호출 부분
            # 예시:
            # success, command, mode = system.run(lidar_msg, cam1_msg, cam2_msg, imu_msg, gps_msg, go_signal_msg)
            # 로그/상태를 추적하여 KPI 업데이트
            # 아래는 예시 값으로 대체
            center_err = np.random.uniform(0.0, 0.8)  # 0~0.8 m
            yaw_err = np.random.uniform(-0.15, 0.15)   # -0.15~0.15 rad
            fallback = np.random.rand() < 0.05  # 5% 확률로 fallback 발생 가정

            stats["centerline_error_mean"].append(center_err)
            stats["centerline_error_max"].append(center_err)
            stats["yaw_error_mean"].append(abs(yaw_err))
            stats["yaw_error_max"].append(abs(yaw_err))
            if fallback:
                stats["fallback_count"] += 1
            stats["frame"].append(frame)

            # 시뮬레이션 진척
            lidar_msg = DummyMsg(frame*dt)
            cam1_msg  = DummyMsg(frame*dt + 0.01)
            cam2_msg  = DummyMsg(frame*dt + 0.01)
            imu_msg   = DummyMsg(frame*dt + 0.02)
            gps_msg   = DummyMsg(frame*dt, pos=(0.0, 0.0, 0.0))

        # 요약 출력
        mean_center = float(np.mean(stats["centerline_error_mean"]))
        max_center  = float(np.max(stats["centerline_error_max"]))
        mean_yaw    = float(np.mean(stats["yaw_error_mean"]))
        max_yaw     = float(np.max(stats["yaw_error_max"]))
        total_fallback = int(stats["fallback_count"])
        print("Validation Summary:")
        print(f"  Centerline error (mean): {mean_center:.3f} m, max: {max_center:.3f} m")
        print(f"  Yaw error (mean): {mean_yaw:.3f} rad, max: {max_yaw:.3f} rad")
        print(f"  Fallback occurrences: {total_fallback}")
        return stats

    if __name__ == "__main__":
        # 실제 로직으로 대체
        # system = FormulaAutonomousSystem()
        # system.init()
        # run_sim(system, frames=200, dt=0.05)
        print("This is a template. Replace with actual ROS-based integration.")

