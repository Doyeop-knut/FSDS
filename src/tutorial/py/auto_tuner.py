
import rosbag
import numpy as np
from scipy.optimize import differential_evolution
import yaml
import os
import sys

# FSDS 프로젝트의 src 디렉토리를 Python 경로에 추가
# 이렇게 하면 formula_autonomous_system 모듈을 바로 import할 수 있습니다.
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

# formula_autonomous_system.py에서 필요한 클래스들을 가져옵니다.
# 오프라인 시뮬레이션을 위해 일부 클래스의 수정이 필요할 수 있습니다.
from formula_autonomous_system.python.formula_autonomous_system import LiDARProcessor, CameraProcessor, GPSIMUProcessor, TrackMap, PathPlanner, Control

# --- 튜닝할 파라미터 정의 ---
# scipy.optimize.differential_evolution에 맞는 형식으로 (min, max) 범위를 지정합니다.
PARAM_SPACE = {
    # MPC racing_mode 가중치
    'mpc_racing_w_ey': (5.0, 30.0),      # 횡방향 오차 가중치
    'mpc_racing_w_epsi': (10.0, 40.0),   # 헤딩 오차 가중치
    'mpc_racing_w_vy': (0.1, 5.0),       # 횡방향 속도 가중치
    'mpc_racing_w_r': (0.1, 5.0),        # 각속도 가중치
    'mpc_racing_w_v': (5.0, 20.0),       # 종방향 속도 가중치
    'mpc_racing_w_ax': (0.05, 1.0),      # 종방향 가속도 변화량 가중치
    'mpc_racing_w_delta': (0.5, 10.0),   # 조향각 변화량 가중치
    'mpc_racing_w_dax': (0.5, 10.0),     # 종방향 가속도 jerk 가중치
    'mpc_racing_w_ddelta': (5.0, 20.0),  # 조향각 jerk 가중치
}

# --- 전역 변수 ---
# 처리할 rosbag 데이터. 여러 bag 파일을 리스트로 넣어 평균 성능을 평가할 수도 있습니다.
ROSBAG_FILES = [] # 예: ['/path/to/your/first.bag', '/path/to/your/second.bag']

# 오토튜닝을 위한 시뮬레이션 클래스
class OfflineSimulator:
    def __init__(self, params):
        """
        주어진 파라미터로 오프라인 시뮬레이터를 초기화합니다.
        formula_autonomous_system.py의 클래스들을 사용하지만, ROS 토픽 대신
        rosbag에서 직접 데이터를 받아 처리합니다.
        """
        self.params = params
        
        # formula_autonomous_system.py에서 사용하는 클래스들을 초기화합니다.
        self.lidar_processor = LiDARProcessor(enable_visualization=False)
        self.track_map = TrackMap()
        self.path_planner = PathPlanner()
        self.controller = Control(self.path_planner, self.track_map)
        self.gps_imu_processor = GPSIMUProcessor()

        # 파라미터 오버라이드: MPC 가중치 설정
        # 나중에 Control 클래스가 이 값들을 사용하도록 수정해야 합니다.
        self.controller.mpc_weights = {
            'w_ey': self.params.get('mpc_racing_w_ey', 15.0),
            'w_epsi': self.params.get('mpc_racing_w_epsi', 20.0),
            'w_vy': self.params.get('mpc_racing_w_vy', 1.0),
            'w_r': self.params.get('mpc_racing_w_r', 1.0),
            'w_v': self.params.get('mpc_racing_w_v', 10.0),
            'w_ax': self.params.get('mpc_racing_w_ax', 0.2),
            'w_delta': self.params.get('mpc_racing_w_delta', 2.0),
            'w_dax': self.params.get('mpc_racing_w_dax', 2.0),
            'w_ddelta': self.params.get('mpc_racing_w_ddelta', 10.0),
        }

    def run_simulation(self, bag_file):
        """
        하나의 rosbag 파일에 대해 시뮬레이션을 실행하고 성능 지표를 반환합니다.
        """
        total_error = 0
        num_points = 0

        with rosbag.Bag(bag_file, 'r') as bag:
            # 필요한 토픽들을 여기에 지정합니다.
            topics = ['/fsds/testing_only/odom', '/fsds/perception/lidar/point_cloud', '/fsds/camera/camera_left/image_color', '/fsds/camera/camera_right/image_color', '/fsds/imu', '/fsds/gps']
            
            for topic, msg, t in bag.read_messages(topics=topics):
                # rosbag의 메시지를 각 프로세서에 전달하고 결과를 얻습니다.
                # 이 부분은 formula_autonomous_system.py의 run 메소드와 유사하게 구성되어야 합니다.
                
                # 예시: LiDAR 데이터 처리
                if topic == '/fsds/perception/lidar/point_cloud':
                    # points = self.lidar_processor.get_lidar_point_cloud(msg)
                    # ...
                    pass

                # 경로 생성 및 제어 로직 실행
                # ...

                # 성능 평가: 예시로, 생성된 경로와 실제 주행 경로(odom) 간의 오차를 계산
                # ground_truth_pose = ... (odom 메시지에서 추출)
                # planned_path = ...
                # error = calculate_path_error(planned_path, ground_truth_pose)
                # total_error += error
                # num_points += 1
                pass

        if num_points == 0:
            return float('inf') # 시뮬레이션 실패 시 매우 큰 비용 반환

        return total_error / num_points

def objective_function(params_array):
    """
    최적화 알고리즘이 호출할 목적 함수입니다.
    입력: 파라미터 값들의 numpy 배열
    출력: 해당 파라미터로 시뮬레이션 실행 시의 비용(cost)
    """
    # 파라미터 배열을 딕셔너리 형태로 변환
    params = {name: value for name, value in zip(PARAM_SPACE.keys(), params_array)}
    
    print(f"Testing parameters: {params}")

    simulator = OfflineSimulator(params)
    
    total_cost = 0
    for bag_file in ROSBAG_FILES:
        cost = simulator.run_simulation(bag_file)
        total_cost += cost
    
    average_cost = total_cost / len(ROSBAG_FILES) if ROSBAG_FILES else float('inf')
    
    print(f"Average cost: {average_cost}")
    return average_cost

def main():
    """
    오토 튜너 메인 함수
    """
    if not ROSBAG_FILES:
        print("오류: ROSBAG_FILES 리스트에 분석할 rosbag 파일 경로를 추가해주세요.")
        return

    # 파라미터 스페이스의 경계를 리스트로 변환
    bounds = [v for v in PARAM_SPACE.values()]

    # SciPy의 differential_evolution을 사용한 최적화
    # 다른 최적화기(예: basinhopping, minimize)도 사용할 수 있습니다.
    result = differential_evolution(
        objective_function, 
        bounds,
        strategy='best1bin',
        maxiter=100, # 최대 반복 횟수
        popsize=15,  # 모집단 크기
        tol=0.01,
        mutation=(0.5, 1),
        recombination=0.7,
        disp=True,   # 진행 상황 출력
        workers=-1   # 병렬 처리 활성화
    )

    print("\n--- 최적화 완료 ---")
    print(f"최적 파라미터: {result.x}")
    print(f"최소 비용: {result.fun}")

    # 최적 파라미터를 딕셔너리 형태로 보기 좋게 출력
    best_params = {name: value for name, value in zip(PARAM_SPACE.keys(), result.x)}
    print("최적 파라미터 상세:")
    print(yaml.dump(best_params, default_flow_style=False))

if __name__ == '__main__':
    main()
