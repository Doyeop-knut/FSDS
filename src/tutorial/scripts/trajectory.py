import numpy as np
import matplotlib.pyplot as plt
import random
import math
import csv
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

class RRTStar:
    def __init__(self, start, goal, boundaries, corridor_path, corridor_bbox, expand_dis=3.0, goal_sample_rate=10, max_iter=500, animation=False):
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.boundaries = boundaries
        self.corridor_path = corridor_path
        self.corridor_bbox = corridor_bbox
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = [self.start]
        self.animation = animation

    def planning(self):
        for i in range(self.max_iter):
            rnd_node = self.get_random_node_in_corridor()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node):
                continue

            near_inds = self.find_near_nodes(new_node)
            new_node = self.choose_parent(new_node, near_inds)
            if new_node:
                self.node_list.append(new_node)
                self.rewire(new_node, near_inds)
            
            if self.animation and i % 5 == 0:
                self.draw_graph(rnd_node)

        last_index = self.search_best_goal_node()
        if last_index is None:
            return None

        return self.generate_final_course(last_index)

    def get_random_node_in_corridor(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            while True:
                x = random.uniform(self.corridor_bbox.xmin, self.corridor_bbox.xmax)
                y = random.uniform(self.corridor_bbox.ymin, self.corridor_bbox.ymax)
                if self.corridor_path.contains_point((x, y)):
                    return Node(x, y)
        else:  # goal point sampling
            return Node(self.end.x, self.end.y)

    def check_collision(self, node):
        if node.parent is None:
            return False
        
        p1 = (node.parent.x, node.parent.y)
        q1 = (node.x, node.y)

        for boundary_line in self.boundaries:
            p2 = boundary_line[0]
            q2 = boundary_line[1]
            if do_lines_intersect(p1, q1, p2, q2):
                return True
        return False

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return new_node

        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and not self.check_collision(t_node):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))
        
        min_cost = min(costs)
        if min_cost == float("inf"):
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost

        return new_node

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.cost = from_node.cost + d
        new_node.parent = from_node

        if extend_length > d:
            extend_length = d

        new_node.x += extend_length * math.cos(theta)
        new_node.y += extend_length * math.sin(theta)

        return new_node

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in node_list]
        min_index = dlist.index(min(dlist))
        return min_index

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        dlist = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]
        near_inds = [i for i, d in enumerate(dlist) if d <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = not self.check_collision(edge_node)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path[::-1]

    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_distance_and_angle(node, self.end)[0] for node in self.node_list]
        goal_inds = [i for i, d in enumerate(dist_to_goal_list) if d <= self.expand_dis]
        
        if not goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in goal_inds])
        for i in goal_inds:
            if self.node_list[i].cost == min_cost:
                return i
        return None

    def draw_graph(self, rnd=None):
        plt.clf()
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")

        for p1, q1 in self.boundaries:
            plt.plot([p1[0], q1[0]], [p1[1], q1[1]], "-k")

        plt.plot(self.start.x, self.start.y, "bs")
        plt.plot(self.end.x, self.end.y, "rs")
        plt.grid(True)
        plt.axis("equal")
        plt.pause(0.01)

class CenterlinePlanner:
    def __init__(self, blue_cones, yellow_cones):
        self.blue_cones = np.array(blue_cones)
        self.yellow_cones = np.array(yellow_cones)
        self.centerline = self._calculate_centerline()
        self.centerline_kdtree = KDTree(self.centerline)

    def _calculate_centerline(self):
        blue_kdtree = KDTree(self.blue_cones)
        midpoints = []
        for y_cone in self.yellow_cones:
            dist, ind = blue_kdtree.query([y_cone], k=1)
            blue_neighbor = self.blue_cones[ind[0][0]]
            midpoints.append((y_cone + blue_neighbor) / 2.0)
        
        midpoints = np.array(midpoints)
        centroid_x = np.mean(midpoints[:, 0])
        centroid_y = np.mean(midpoints[:, 1])
        
        angles = np.arctan2(midpoints[:, 1] - centroid_y, midpoints[:, 0] - centroid_x)
        sorted_indices = np.argsort(angles)
        
        sorted_centerline = midpoints[sorted_indices]
        return np.vstack([sorted_centerline, sorted_centerline[0]]) # Close the loop

    def get_next_goal(self, current_pos, lookahead_dist):
        dist, ind = self.centerline_kdtree.query([current_pos], k=1)
        current_segment_idx = ind[0][0]

        total_dist = 0
        for i in range(len(self.centerline) - 1):
            idx = (current_segment_idx + i) % (len(self.centerline) - 1)
            next_idx = (current_segment_idx + i + 1) % (len(self.centerline) - 1)
            
            p1 = self.centerline[idx]
            p2 = self.centerline[next_idx]
            segment_dist = np.linalg.norm(p2 - p1)

            if total_dist + segment_dist >= lookahead_dist:
                remaining_dist = lookahead_dist - total_dist
                direction = (p2 - p1) / segment_dist
                return tuple(p1 + direction * remaining_dist)
            
            total_dist += segment_dist
        
        return tuple(self.centerline[-2]) # Failsafe

def main():
    print("Dynamic RRT* path planning...")

    # ==== 1. 데이터 로드 및 설정 ====
    blue_cones = []
    yellow_cones = []

    import os
    script_dir = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(script_dir, '../maps/track_droneport.csv')

    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            point = (float(row[1]), float(row[2]))
            if row[0] == 'blue':
                blue_cones.append(point)
            elif row[0] == 'yellow':
                yellow_cones.append(point)

    def create_boundary_lines(cones):
        lines = []
        if not cones: return lines, []
        centroid_x = sum(p[0] for p in cones) / len(cones)
        centroid_y = sum(p[1] for p in cones) / len(cones)
        sorted_cones = sorted(cones, key=lambda p: math.atan2(p[1] - centroid_y, p[0] - centroid_x))
        for i in range(len(sorted_cones) - 1): lines.append((sorted_cones[i], sorted_cones[i+1]))
        lines.append((sorted_cones[-1], sorted_cones[0]))
        return lines, sorted_cones

    blue_boundaries, sorted_blue_cones = create_boundary_lines(blue_cones)
    yellow_boundaries, sorted_yellow_cones = create_boundary_lines(yellow_cones)
    all_boundaries = blue_boundaries + yellow_boundaries

    corridor_vertices = sorted_blue_cones + sorted_yellow_cones[::-1]
    corridor_path_obj = Path(corridor_vertices)
    corridor_bbox = corridor_path_obj.get_extents()

    # ==== 2. 자율 주행 루프 ====
    plt.figure(figsize=(15, 15))
    full_path = []
    all_nodes = []
    
    start_node = (0.0, 0.0)
    current_pos = start_node
    centerline_planner = CenterlinePlanner(blue_cones, yellow_cones)
    lookahead_distance = 15.0
    lap_finished = False

    while not lap_finished:
        goal_point = centerline_planner.get_next_goal(current_pos, lookahead_distance)
        print(f"Planning from {current_pos} to {goal_point}")

        rrt_star = RRTStar(start=current_pos, 
                               goal=goal_point, 
                               boundaries=all_boundaries,
                               corridor_path=corridor_path_obj,
                               corridor_bbox=corridor_bbox,
                               expand_dis=3.0,
                               max_iter=1000, 
                               animation=True)
        path_segment = rrt_star.planning()
        
        if path_segment:
            if not full_path: full_path.extend(path_segment)
            else: full_path.extend(path_segment[1:])
            all_nodes.extend(rrt_star.node_list)
            current_pos = path_segment[-1]
        else:
            print(f"Could not find path to {goal_point}. Aborting.")
            break

        # 랩 완료 조건 확인
        dist_to_start = np.linalg.norm(np.array(current_pos) - np.array(start_node))
        if len(full_path) > 10 and dist_to_start < 5.0:
            print("Lap finished!")
            lap_finished = True

    # ==== 3. 최종 경로 시각화 ====
    print("Path generation complete. Showing final path.")
    plt.clf()
    for p1, q1 in all_boundaries:
        plt.plot([p1[0], q1[0]], [p1[1], q1[1]], "-k")
    
    plt.plot(centerline_planner.centerline[:, 0], centerline_planner.centerline[:, 1], 'c--', label='Centerline')

    for node in all_nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g", alpha=0.2)
    
    if full_path:
        px, py = zip(*full_path)
        plt.plot(px, py, '-r', linewidth=2, label='Final Path')
    
    plt.plot(start_node[0], start_node[1], "go", markersize=12, label='Start/Finish')
    
    plt.title("Autonomous RRT* Path Planning")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

if __name__ == '__main__':
    main()