"""
Plotting tools for Sampling-based algorithms
@author: huiming zhou
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from rrt_2D import env


class Plotting:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.env = env.Env()
        self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle

    def animation(self, nodelist, path, name, animation=False):
        self.plot_grid(name)
        self.plot_visited(nodelist, animation)
        self.plot_path(path)

    def animation_connect(self, V1, V2, path, name):
        self.plot_grid(name)
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)

    def plot_grid(self, name):
        fig, ax = plt.subplots()

        for (ox, oy, w, h) in self.obs_bound:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        # for (ox, oy, w, h) in self.obs_rectangle:
        #     ax.add_patch(
        #         patches.Rectangle(
        #             (ox, oy), w, h,
        #             edgecolor='black',
        #             facecolor='gray',
        #             fill=True
        #         )
        #     )

        # for (ox, oy, r) in self.obs_circle:
        #     ax.add_patch(
        #         patches.Circle(
        #             (ox, oy), r,
        #             edgecolor='black',
        #             facecolor='gray',
        #             fill=True
        #         )
        #     )

        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3)
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    @staticmethod
    def plot_visited(nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)

    @staticmethod
    def plot_path(path):
        if len(path) != 0:
            plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
            plt.pause(0.01)
        plt.show()

    @staticmethod
    def plot_traj(traj):
        t = len(traj)

        plt.subplot(3, 1, 1)
        # plot the position of traj, x-axis is time, y-axis is position
        plt.plot(range(t), [x[0][0] for x in traj], '-r', label='x')
        plt.plot(range(t), [x[0][1] for x in traj], '-b', label='y')
        plt.legend()
        plt.xlabel('time')
        plt.ylabel('position')

        
        plt.subplot(3, 1, 2)
        # plot the velocity of traj, x-axis is time, y-axis is velocity
        plt.plot(range(t), [x[1][0] for x in traj], '-r', label='vx')
        plt.plot(range(t), [x[1][1] for x in traj], '-b', label='vy')
        # ///////////////////////////// Debug //////////////////////////
        diff_pos_x = [(traj[i][0][0] - traj[i-1][0][0])*10 for i in range(1, t-1)]
        diff_pos_y = [(traj[i][0][1] - traj[i-1][0][1])*10 for i in range(1, t-1)]
        plt.plot(range(1, t-1), diff_pos_x, '--r', label='diff_pos_x')
        plt.plot(range(1, t-1), diff_pos_y, '--b', label='diff_pos_y')
        # //////////////////////////// Debug ///////////////////////////
        plt.legend()
        plt.xlabel('time')
        plt.ylabel('velocity')
 
        
        plt.subplot(3, 1, 3)
        # plot the acceleration of traj, x-axis is time, y-axis is acceleration
        plt.plot(range(t), [x[2][0] for x in traj], '-r', label='ax')
        plt.plot(range(t), [x[2][1] for x in traj], '-b', label='ay')
        # ///////////////////////////// Debug //////////////////////////
        diff_vel_x = [(traj[i][1][0] - traj[i-1][1][0])*10 for i in range(1, t-1)]
        diff_vel_y = [(traj[i][1][1] - traj[i-1][1][1])*10 for i in range(1, t-1)]
        plt.plot(range(1, t-1), diff_vel_x, '--r', label='diff_vel_x')
        plt.plot(range(1, t-1), diff_vel_y, '--b', label='diff_vel_y')
        # //////////////////////////// Debug ///////////////////////////
        plt.legend()
        plt.xlabel('time')
        plt.ylabel('acceleration')
        plt.show()  