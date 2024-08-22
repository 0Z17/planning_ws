"""
RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import numpy as np

base_path = os.path.dirname(os.path.abspath(__file__)) + "/../../../../../"

sys.path.append(base_path + "src/control_aware_planner/src/")

sys.path.append(base_path + "src/PathPlanning/src/Sampling_based_Planning/")

print(sys.path)

from rrt_2D import env, plotting, utils
from pathGen import PathGen


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.vx = n[2]
        self.vy = n[3]
        self.ax = n[4]
        self.ay = n[5]
        self.parent = None
        self.traj = []
        self.duration = None # time from the parent node to this node

    def get_traj(self, pg, t):

        t_new = 0
        while t_new < t:
            self.traj.append(pg.get_waypoints_with_time(t_new))
            t_new += 0.1
        # self.traj.append(pg.get_waypoints_with_time(t))
        


class RrtState:
    def __init__(self, s_start, s_goal, step_len, time_step, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.time_step = time_step
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.vx_range = (-1,1)
        self.vy_range = (-1,1)
        self.ax_range = (-1,1)
        self.ay_range = (-1,1)
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.pg = PathGen(2)

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new, t = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                node_new.get_traj(self.pg,t)
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len and not self.utils.is_collision(node_new, self.s_goal):
                    if node_new is not self.s_goal:
                        node_new,t = self.new_state(node_new, self.s_goal)
                        node_new.get_traj(self.pg,t)
                    return self.extract_path(node_new), self.extract_traj(node_new)

        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta),
                         np.random.uniform(self.vx_range[0] + delta, self.vx_range[1] - delta),
                         np.random.uniform(self.vy_range[0] + delta, self.vy_range[1] - delta),
                         np.random.uniform(self.ax_range[0] + delta, self.ax_range[1] - delta),
                         np.random.uniform(self.ay_range[0] + delta, self.ay_range[1] - delta)))

        return self.s_goal

    def nearest_neighbor(self,node_list, n):
        return node_list[int(np.argmin([self.get_duration(nd, n) for nd in node_list]))]

    def get_duration(self, node_start, node_end):
        start_pos = [node_start.x, node_start.y]
        start_vel = [node_start.vx, node_start.vy]
        start_acc = [node_start.ax, node_start.ay]

        end_pos = [node_end.x, node_end.y]
        end_vel = [node_end.vx, node_end.vy]
        end_acc = [node_end.ax, node_end.ay]

        self.pg.generate_path(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc)
        return self.pg.get_duration()

    def new_state(self, node_start, node_end):
        t = self.get_duration(node_start, node_end)

        t_new = min(self.time_step, t)

        if t_new == t:
            node_new = node_end
            print("duration time less than the time step")
        else:
            n = np.array(self.pg.get_waypoints_with_time(self.time_step))
            node_new = Node(np.ravel(n))

        node_new.parent = node_start
        node_new.duration = t_new

        return node_new, t_new

    def extract_path(self, node_end):
        # path = [(self.s_goal.x, self.s_goal.y)]
        path = [(node_end.x, node_end.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path


    def extract_traj(self, node_end):
        traj = [([node_end.x, node_end.y],[node_end.vx, node_end.vy], [node_end.ax, node_end.ay])]
        node_now = node_end

        while node_now.parent is not None:
            traj.extend(node_now.traj[::-1])
            node_now = node_now.parent


        return traj

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def main():
    x_start = (2, 2, 0, 0, 0, 0)  # Starting node
    x_goal = (10, 10, 0, 0, 0, 0)  # Goal node

    rrt = RrtState(x_start, x_goal, step_len=0.5, time_step=2, goal_sample_rate=0.05, iter_max=10000)
    path, traj = rrt.planning()

    if path:
        rrt.plotting.plot_traj(traj)
        rrt.plotting.animation(rrt.vertex, path, "RRT", True)
        export_traj(traj)
        print("Path Found!")
    else:
        print("No Path Found!")

def export_traj(traj):
    traj.reverse()  
    np.save(base_path + "data/uv_traj.npy", traj)
    

if __name__ == '__main__':
    main()
