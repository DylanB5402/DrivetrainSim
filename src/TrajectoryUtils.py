import pathfinder as pf
import math
import NerdyMath
import matplotlib.pyplot as plt

def graph_trajectory_x_y(traj):
    x_list = []
    y_list = []
    for seg in traj:
        x_list.append(seg.x)
        y_list.append(seg.y)
    plt.plot(x_list, y_list)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()

def get_x_y_lists(traj):
    x_list = []
    y_list = []
    for seg in traj:
        x_list.append(seg.x)
        y_list.append(seg.y)
    return x_list, y_list

def graph_trajectory_velocity(traj):
    time_list = []
    vel_list = []
    time = 0
    for seg in traj:
        time += seg.dt
        time_list.append(time)
        vel_list.append(seg.velocity)
    plt.plot(time_list , vel_list)
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.axis([0, 20, 0, 20])
    plt.show()
        
def get_closest_segment(x, y, trajectory):
    min = 10000
    for seg in trajectory:
        dist = NerdyMath.distance_formula(x, y, seg.x, seg.y)
        if dist < min:
            min = dist
            closest_seg = seg
    return closest_seg
    
def get_closer_segment(x, y, seg_1, seg_2):
    dist_1 = NerdyMath.distance_formula(x, y, seg_1.x, seg_2.y)
    dist_2 = NerdyMath.distance_formula(x, y, seg_2.x, seg_2.y)
    if dist_1 <= dist_2:
        return seg_1
    else: 
        return seg_2

def get_closer_segment_range(x, y, trajectory, index, range):
    min = 100000
    counter = index - range
    max = index + range
    if max > len(trajectory) - 1:
        max = len(trajectory) - 1
    while counter != max:
        seg = trajectory[counter]
        dist = NerdyMath.distance_formula(x, y, seg.x, seg.y)
        if dist < min:
            min = dist
            closest_seg = seg
        counter += 1
    return closest_seg
