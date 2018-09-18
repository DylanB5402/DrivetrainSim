import math
import matplotlib.pyplot as plt
import NerdyMath
import numpy
from BezierCurve import  BezierCurve
import TrajectoryUtils

class DifferentialDrivetrain():

    def __init__(self, width, dt, x_offset, y_offset):
        self.width = width
        self.robot_angle = 0
        self.left_vel = 0
        self.right_vel = 0
        self.left_pos = 0
        self.right_pos = 0
        self.robot_x = x_offset
        self.robot_y = y_offset
        self.dt = dt
        self.robot_pos = 0
        self.x_list = []
        self.y_list = []
        self.robot_angle_deg = 0
        self.time = 0
        self.linear_vel = 0
        self.angular_vel = 0


    def update(self, left_vel, right_vel):
        # print('update')
        self.left_vel = left_vel
        self.right_vel = right_vel
        self.linear_vel = (left_vel + right_vel)/2
        self.angular_vel = (left_vel - right_vel)/self.width
        delta_pos = self.linear_vel * self.dt
        delta_theta = self.angular_vel * self.dt
        self.robot_pos += delta_pos
        self.robot_angle += delta_theta
        self.right_pos += self.right_vel * self.dt
        self.left_pos += self.left_vel * self.dt
        self.robot_x += delta_pos * math.sin(self.robot_angle)
        self.robot_y += delta_pos * math.cos(self.robot_angle)
        self.x_list.append(self.robot_x)
        self.y_list.append(self.robot_y)
        self.robot_angle_deg = math.degrees(self.robot_angle)
        self.time += self.dt


    def set_position(self, x, y):
        self.robot_x = x
        self.robot_y = y

    def graph(self):
        plt.plot(self.x_list, self.y_list)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis([-100, 100, -100, 100])
        plt.show()
    
    def graph_with_path(self, path_x, path_y):
        plt.plot(self.x_list, self.y_list)
        plt.plot(path_x, path_y)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend(['robot position', 'path'])
        plt.show()