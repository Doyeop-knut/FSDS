import numpy as np
import matplotlib.pyplot as plt
import random
import math
import csv
import os
from scipy.interpolate import splprep, splev
from matplotlib.path import Path
from sklearn.neighbors import KDTree

# --- 선분 교차 판별을 위한 헬퍼 함수 --- #

def on_segment(p, q, r):
    if (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
        q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
        return True
    return False

def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0: return 0
    return 1 if val > 0 else 2

def do_lines_intersect(p1, q1, p2, q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if (o1 != o2 and o3 != o4):
        return True

    if (o1 == 0 and on_segment(p1, p2, q1)): return True
    if (o2 == 0 and on_segment(p1, q2, q1)): return True
    if (o3 == 0 and on_segment(p2, p1, q2)): return True
    if (o4 == 0 and on_segment(p2, q1, q2)): return True

    return False

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class DynamicRRTStar:
    def __init__(self, start, all_cones, boundaries, corridor_path, corridor_bbox, expand_dis=3.0, goal_sample_rate=5, max_iter=500):
        self.start_node = Node(start[0], start[1])
        self.node_list = [self.start_node]
        
        self.all_cones = np.array(all_cones)
        self.cone_kdtree = KDTree(self.all_cones)
        
        self.boundaries = boundaries
        self.corridor_path = corridor_path
        self.corridor_bbox = corridor_bbox
        
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter

    def planning(self, animation=False):
        for i in range(self.max_iter):
            # 1. Get random node
            rnd_node = self.get_random_node_in_corridor()
            
            # 2. Find nearest node in the tree
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            # 3. Steer from nearest node towards random node
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            # 4. Evaluate new node based on cone distribution
            score = self.evaluate_node(new_node)
            if score < 0.5: # Penalize nodes with cones only on one side
                continue

            # 5. Check for collision
            if self.check_collision(new_node):
                continue

            # 6. Rewire the tree
            near_inds = self.find_near_nodes(new_node)
            new_node = self.choose_parent(new_node, near_inds)
            if new_node:
                self.node_list.append(new_node)
                self.rewire(new_node, near_inds)
            
            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)
        
        # 7. Find the best path from the generated tree
        best_path, _ = self.find_best_path()
        return best_path

    def evaluate_node(self, node, radius=5.0):
        """ 주변 콘 분포를 기반으로 노드를 평가. 양쪽에 콘이 있으면 높은 점수 반환 """
        points_in_radius = self.cone_kdtree.query_radius([(node.x, node.y)], r=radius)[0]
        if len(points_in_radius) < 2:
            return 0.0 # 주변에 콘이 거의 없음

        cones_in_radius = self.all_cones[points_in_radius]
        
        # 노드에서 각 콘까지의 벡터 계산
        vectors = cones_in_radius - np.array([node.x, node.y])
        angles = np.arctan2(vectors[:, 1], vectors[:, 0])
        
        # 각도 차이 계산
        angles_sorted = np.sort(angles)
        angle_diffs = np.diff(np.append(angles_sorted, angles_sorted[0] + 2 * np.pi))
        
        # 가장 큰 각도 차이가 180도(pi)에 가까우면 양쪽에 콘이 있다는 의미
        max_angle_diff = np.max(angle_diffs)
        score = max_angle_diff / np.pi # 1.0에 가까울수록 좋음
        return score

    def find_best_path(self, min_path_len=20):
        """ 트리에서 가장 길고 점수가 높은 경로를 찾음 """
        best_path = []
        best_cost = -1

        for node in self.node_list:
            path = self.generate_final_course(node)
            if len(path) > min_path_len:
                cost = sum(self.evaluate_node(n) for n in path) / len(path)
                if cost > best_cost:
                    best_cost = cost
                    best_path = path
        return best_path, best_cost

    def generate_final_course(self, node):
        path = []
        current = node
        while current is not None:
            path.append(current)
            current = current.parent
        return path[::-1]

    def get_random_node_in_corridor(self):
        # For this dynamic approach, we primarily explore, so less goal sampling
        if random.randint(0, 100) > self.goal_sample_rate:
            while True:
                x = random.uniform(self.corridor_bbox.xmin, self.corridor_bbox.xmax)
                y = random.uniform(self.corridor_bbox.ymin, self.corridor_bbox.ymax)
                if self.corridor_path.contains_point((x, y)):
                    return Node(x, y)
        else: # Sample the end of the longest path so far
            best_path, _ = self.find_best_path()
            if best_path:
                return best_path[-1]
            else: # Failsafe
                return self.node_list[0]

    def check_collision(self, node):
        if node.parent is None: return False
        p1 = (node.parent.x, node.parent.y); q1 = (node.x, node.y)
        for p2, q2 in self.boundaries: 
            if do_lines_intersect(p1, q1, p2, q2): return True
        return False

    def choose_parent(self, new_node, near_inds):
        if not near_inds: return new_node
        costs = [self.calc_new_cost(self.node_list[i], new_node) for i in near_inds]
        min_cost = min(costs)
        min_ind = near_inds[costs.index(min_cost)]
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost
        return new_node

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        new_node.cost = from_node.cost + d
        new_node.parent = from_node
        if extend_length > d: extend_length = d
        new_node.x += extend_length * math.cos(theta)
        new_node.y += extend_length * math.sin(theta)
        return new_node

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in node_list]
        return dlist.index(min(dlist))

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        dlist = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]
        return [i for i, d in enumerate(dlist) if d <= r**2]

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node: continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)
            if not self.check_collision(edge_node) and near_node.cost > edge_node.cost:
                near_node.parent = new_node

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def draw_graph(self, rnd=None):
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd: plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list: 
            if node.parent: plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")
        for p1, q1 in self.boundaries: plt.plot([p1[0], q1[0]], [p1[1], q1[1]], "-k")
        best_path, _ = self.find_best_path()
        if best_path: 
            path_coords = [(n.x, n.y) for n in best_path]
            px, py = zip(*path_coords)
            plt.plot(px, py, '-r', linewidth=2)
        plt.plot(self.start_node.x, self.start_node.y, "bs")
        plt.grid(True); plt.axis("equal"); plt.pause(0.01)

def main():
    print("Dynamic RRT* path planning based on cone distribution...")

    # 1. 데이터 로드
    blue_cones, yellow_cones = [], []
    # trajectory.py가 있는 scripts 폴더를 기준으로 경로 설정
    script_dir = os.path.dirname(os.path.realpath(__file__))
    # maps 폴더는 tutorial 폴더 아래에 있으므로, scripts에서 한 단계 위로 올라가 maps로 접근
    file_path = os.path.join(script_dir, '../maps/track_droneport.csv')
    with open(file_path, 'r') as f:
        for row in csv.reader(f):
            point = (float(row[1]), float(row[2]))
            if row[0] == 'blue': blue_cones.append(point)
            elif row[0] == 'yellow': yellow_cones.append(point)
    all_cones = blue_cones + yellow_cones

    # 2. 경계 및 코리도 생성
    def create_boundary_lines(cones):
        if not cones: return [], []
        centroid = np.mean(cones, axis=0)
        sorted_cones = sorted(cones, key=lambda p: math.atan2(p[1] - centroid[1], p[0] - centroid[0]))
        lines = list(zip(sorted_cones, sorted_cones[1:] + sorted_cones[:1]))
        return lines, sorted_cones

    blue_boundaries, sorted_blue = create_boundary_lines(blue_cones)
    yellow_boundaries, sorted_yellow = create_boundary_lines(yellow_cones)
    all_boundaries = blue_boundaries + yellow_boundaries
    corridor_path = Path(sorted_blue + sorted_yellow[::-1])
    corridor_bbox = corridor_path.get_extents()

    # 3. RRT* 실행
    plt.figure(figsize=(15, 15))
    rrt_star = DynamicRRTStar(start=(0.0, 0.0), 
                              all_cones=all_cones,
                              boundaries=all_boundaries,
                              corridor_path=corridor_path,
                              corridor_bbox=corridor_bbox,
                              expand_dis=3.0,
                              max_iter=2000)
    
    final_path_nodes = rrt_star.planning(animation=False)

    # 4. 최종 결과 표시
    print("Planning finished. Showing final path.")
    plt.clf()
    for p1, q1 in all_boundaries: plt.plot([p1[0], q1[0]], [p1[1], q1[1]], "-k")
    if final_path_nodes:
        final_path = [(n.x, n.y) for n in final_path_nodes]
        px, py = zip(*final_path)
        plt.plot(px, py, '-r', linewidth=2, label='Final Path')
    plt.plot(0.0, 0.0, "go", markersize=12, label='Start')
    plt.title("Dynamic RRT* Final Path")
    plt.legend(); plt.grid(True); plt.axis("equal"); plt.show()

if __name__ == '__main__':
    main()