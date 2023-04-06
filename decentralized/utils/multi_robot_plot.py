"""
Plotting tool for 2D multi-robot system

author: Ashwin Bose (@atb033)
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle,Wedge
import numpy as np


def plot_robot_and_obstacles(robot, obstacles, robot_radius, num_steps, sim_time, filename):
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(0, 10), ylim=(0, 10))
    ax.set_aspect('equal')
    ax.grid()
    line, = ax.plot([], [], '--r')

    robot_patch = Circle((robot[0, 0], robot[1, 0]),
                         robot_radius, facecolor='green', edgecolor='black')
    obstacle_list = []
    for obstacle in range(np.shape(obstacles)[2]):
        obstacle = Circle((0, 0), robot_radius,
                          facecolor='aqua', edgecolor='black')
        obstacle_list.append(obstacle)

    def init():
        ax.add_patch(robot_patch)
        for obstacle in obstacle_list:
            ax.add_patch(obstacle)
        line.set_data([], [])
        return [robot_patch] + [line] + obstacle_list

    def animate(i):
        robot_patch.center = (robot[0, i], robot[1, i])
        for j in range(len(obstacle_list)):
            obstacle_list[j].center = (obstacles[0, i, j], obstacles[1, i, j])
        line.set_data(robot[0, :i], robot[1, :i])
        return [robot_patch] + [line] + obstacle_list

    init()
    step = (sim_time / num_steps)
    for i in range(num_steps):
        animate(i)
        plt.pause(step)

    # Save animation
    if not filename:
        return

    ani = animation.FuncAnimation(
        fig, animate, np.arange(1, num_steps), interval=200,
        blit=True, init_func=init)

    ani.save(filename, "ffmpeg", fps=30)


def plot_robot_obstacles_and_vos(robot, obstacles, robot_radius, num_steps, sim_time, filename,vo_Amat_hist,vo_bvec_hist,vo_pt_hist,vo_dist_hist):
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(0, 10), ylim=(0, 10))
    ax.set_aspect('equal')
    ax.grid()
    line, = ax.plot([], [], '--r')

    robot_patch = Circle((robot[0, 0], robot[1, 0]),
                         robot_radius, facecolor='green', edgecolor='black')
    obstacle_list = []
    for _ in range(np.shape(obstacles)[2]):
        obstacle = Circle((0, 0), robot_radius,
                          facecolor='aqua', edgecolor='black')
        obstacle_list.append(obstacle)

    vo_hist_list=[]
    for i in range(np.shape(vo_Amat_hist)[-1]):
        vo_i = []
        vo_Amat = vo_Amat_hist[:,:,i]
        vo_bvec = vo_bvec_hist[:,i]
        vo_pt = vo_pt_hist[:,:,i]
        vo_dist = vo_dist_hist[:,:,i]
        for j in range(np.shape(vo_pt)[0]):
            vo_cone = Wedge(vo_pt[j,:],vo_dist[j,0],np.rad2deg(np.arctan2(vo_Amat[2*j+1,1],vo_Amat[2*j+1,0])),np.rad2deg(np.arctan2(vo_Amat[2*j,1],vo_Amat[2*j,0])),ec="None")
            vo_i.append(vo_cone)
        vo_hist_list.append(vo_i)
    vo_cone_list = []
    for _ in range(np.shape(vo_pt_hist)[0]):
        vo_cone_obj = Wedge((0,0),0.1,0,0,ec="None")
        vo_cone_list.append(vo_cone_obj)

    def init():
        for cone in vo_cone_list:
            ax.add_patch(cone)
        ax.add_patch(robot_patch)
        for obstacle in obstacle_list:
            ax.add_patch(obstacle)

        line.set_data([], [])
        return [robot_patch] + [line] + obstacle_list

    def animate(i):
        robot_patch.center = (robot[0, i], robot[1, i])

        for j in range(len(vo_cone_list)):
            vo_cone_handle=vo_cone_list[j]
            assert isinstance(vo_cone_handle,Wedge)
            vo_cone_handle.set_center((vo_pt_hist[j,0,i],vo_pt_hist[j,1,i]))
            vo_cone_handle.set_radius(vo_dist_hist[j,0,i]*100)
            vo_cone_handle.set_alpha(0.5)
            angle_2 =np.rad2deg(np.arctan2(vo_Amat_hist[2*j+1,1,i],vo_Amat_hist[2*j+1,0,i]))
            angle_1 =np.rad2deg(np.arctan2(vo_Amat_hist[2*j,1,i],vo_Amat_hist[2*j,0,i]))
            angle_not_reverse = angle_2-angle_1<180 if (angle_2-angle_1>=0) else angle_2-angle_1+360 < 180
            if(1):
                vo_cone_handle.set_theta2(np.rad2deg(np.arctan2(vo_Amat_hist[2*j+1,1,i],vo_Amat_hist[2*j+1,0,i])))
                vo_cone_handle.set_theta1(np.rad2deg(np.arctan2(vo_Amat_hist[2*j,1,i],vo_Amat_hist[2*j,0,i])))
            else:
                vo_cone_handle.set_theta1(np.rad2deg(np.arctan2(vo_Amat_hist[2*j+1,1,i],vo_Amat_hist[2*j+1,0,i])))
                vo_cone_handle.set_theta2(np.rad2deg(np.arctan2(vo_Amat_hist[2*j,1,i],vo_Amat_hist[2*j,0,i])))
        for j in range(len(obstacle_list)):
            obstacle_list[j].center = (obstacles[0, i, j], obstacles[1, i, j])
        line.set_data(robot[0, :i], robot[1, :i])
        ax.set_title("Epoch:{}".format(i))
        return [robot_patch] + [line] + obstacle_list

    init()
    step = (sim_time / num_steps)
    for i in range(num_steps):
        animate(i)
        plt.pause(step+0.5)


    # Save animation
    if not filename:
        return

    ani = animation.FuncAnimation(
        fig, animate, np.arange(1, num_steps), interval=100,
        blit=True, init_func=init)

    ani.save(filename, "ffmpeg", fps=30)




def plot_robot(robot, timestep, radius=1, is_obstacle=False):
    if robot is None:
        return
    center = robot[:2, timestep]
    x = center[0]
    y = center[1]
    if is_obstacle:
        circle = plt.Circle((x, y), radius, color='aqua', ec='black')
        plt.plot(robot[0, :timestep], robot[1, :timestep], '--r',)
    else:
        circle = plt.Circle((x, y), radius, color='green', ec='black')
        plt.plot(robot[0, :timestep], robot[1, :timestep], 'blue')

    plt.gcf().gca().add_artist(circle)
