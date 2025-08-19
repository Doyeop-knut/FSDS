import numpy as np
import matplotlib.pyplot as plt
import random
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTStar:
    def __init__(self, start, goal, rand_area, expand_dis=3.0, goal_sample_rate=10, max_iter=500):
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = [self.start]

    def planning(self):
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if not self.check_inside_bounds(new_node):
                continue

            near_inds = self.find_near_nodes(new_node)
            self.node_list.append(new_node)
            self.rewire(new_node, near_inds)

        # find best goal path
        last_index = self.search_best_goal_node()
        if last_index is None:
            return None

        return self.generate_final_course(last_index)

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        extend_length = min(extend_length, d)
        new_node.x += extend_length * math.cos(theta)
        new_node.y += extend_length * math.sin(theta)
        new_node.parent = from_node
        new_node.cost = from_node.cost + extend_length
        return new_node

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = Node(random.uniform(self.min_rand, self.max_rand),
                       random.uniform(self.min_rand, self.max_rand))
        else:
            rnd = Node(self.end.x, self.end.y)
        return rnd

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
            d, _ = self.calc_distance_and_angle(new_node, near_node)
            scost = new_node.cost + d

            if scost < near_node.cost:
                near_node.parent = new_node
                near_node.cost = scost

    def generate_final_course(self, goal_ind):
        path = []
        node = self.node_list[goal_ind]
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
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

    def check_inside_bounds(self, node):
        if node.x < self.min_rand or node.x > self.max_rand or node.y < self.min_rand or node.y > self.max_rand:
            return False
        return True

def main():
    print("RRT* path planning...")

    start = [10, 10]
    goal = [90, 90]
    rrt_star = RRTStar(start, goal, rand_area=[0, 100])
    path = rrt_star.planning()

    # 시각화
    plt.figure(figsize=(8, 8))
    for node in rrt_star.node_list:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")
    if path:
        px, py = zip(*path)
        plt.plot(px, py, '-r', linewidth=2)
    plt.plot(start[0], start[1], "bo")
    plt.plot(goal[0], goal[1], "ko")
    plt.title("RRT* Path Planning")
    plt.grid(True)
    plt.axis("equal")
    plt.show()

if __name__ == '__main__':
    main()