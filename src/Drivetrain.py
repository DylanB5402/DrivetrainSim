import math
import matplotlib.pyplot as plt
import NerdyMath
import numpy
from BezierCurve import  BezierCurve
import pathfinder
import TrajectoryUtils
from pathfinder import modifiers



class Drivetrain():

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
        self.angular_vel = -(left_vel - right_vel)/self.width
        delta_pos = self.linear_vel * self.dt
        delta_theta = self.angular_vel * self.dt
        self.robot_pos += delta_pos
        self.robot_angle += delta_theta
        self.right_pos += self.right_vel * self.dt
        self.left_pos += self.left_vel * self.dt
        self.robot_x += delta_pos * math.cos(self.robot_angle)
        self.robot_y += delta_pos * math.sin(self.robot_angle)
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

    def drive_forward_PID(self, target, kP, kD):
        error = target - self.robot_pos
        prev_error = error
        while abs(error) > 0.02:
            error = target - self.robot_pos
            velocity = error * kP + ((error - prev_error)/self.dt) * kD
            self.update(velocity, velocity)
            prev_error = error

    def turn_to_angle_PID(self, target, kP, kD):
        # angle = -(360 - self.robot_angle_deg) % 360
        error = target - self.robot_angle_deg
        # if error >= 180:
        #     error -= 360
        # if error <= -180:
        #     error += 360
        prev_error = error
        while abs(error) > 0.000002:
            # angle = -(360 - self.robot_angle_deg) % 360
            error = pathfinder.boundHalfDegrees(target - self.robot_angle_deg)
            # if error >= 180:
            #     error -= 360
            # elif error <= -180:
            #     error += 360
            velocity = error * kP + ((error - prev_error) / self.dt) * kD
            self.update(-velocity, velocity)
            prev_error = error



    def drive_at_heading(self, target_angle, kP, distance, straight_velocity):
        angle = -(360 - self.robot_angle_deg) % 360
        error = target_angle - angle
        if error >= 180:
            error -= 360
        if error <= -180:
            error += 360
        straight_error = distance - self.robot_pos
        while abs(straight_error) > 0.02:
            straight_error = distance - self.robot_pos
            angle = -(360 - self.robot_angle_deg) % 360
            error = target_angle - angle
            if error >= 180:
                error -= 360
            elif error <= -180:
                error += 360
            rot_velocity = error * kP
            self.update(straight_velocity + rot_velocity, straight_velocity - rot_velocity)

    def arc_turn(self, target, kP, power_right, direction):
        angle = -(360 - self.robot_angle_deg) % 360
        error = target - angle
        if error >= 180:
            error -= 360
        if error <= -180:
            error += 360
        while abs(error) > 0.02:
            angle = -(360 - self.robot_angle_deg) % 360
            error = target - angle
            if error >= 180:
                error -= 360
            elif error <= -180:
                error += 360
            velocity = abs(error * kP) * numpy.sign(direction)
            # velocity = error * kP
            if power_right:
                self.update(0, velocity)
            else:
                self.update(velocity, 0)

    def radius_turn(self, radius, velocity, distance, turn_right):
        error = distance - self.robot_pos
        while abs(error) > 0.02:
            error = distance - self.robot_pos
            inner_vel = velocity * (radius - (self.width/2))/(radius + (self.width/2))
            if turn_right:
                self.update(velocity, inner_vel)
            else:
                self.update(inner_vel, velocity)

    def drive_motion_profile(self, target_distance, cruise_vel, max_accel):
        max_accel_time = cruise_vel / max_accel
        max_accel_dist = 0.5 * max_accel * max_accel_time ** 2
        accel_time = math.sqrt(numpy.abs(target_distance)/ max_accel)
        if target_distance < 2 * max_accel_dist:  # if motion profile is triangular
            is_trapezoidal = False
            accel_time = math.sqrt(numpy.abs(target_distance) / max_accel)
            cruise_time = 0
        else:  # if motion profile is trapezoidal
            accel_time = math.sqrt((2 * max_accel_dist) / max_accel)
            cruise_distance = target_distance - (2 * max_accel_dist)
            cruise_time = (cruise_distance / cruise_vel) + accel_time
            is_trapezoidal = True
        start_time = self.time
        time = self.time - start_time
        # print('total time', cruise_time + 2 * accel_time)
        while time < cruise_time + 2 * accel_time :
            time = self.time - start_time
            if is_trapezoidal:
                if time <= accel_time:
                    velocity_output = max_accel * time
                elif time > accel_time and time < cruise_time:
                    velocity_output = cruise_vel
                elif time >= cruise_time:
                    velocity_output = -max_accel * (time - accel_time - cruise_time)
                velocity_output
            else:
                if time <= accel_time:
                    velocity_output = max_accel * time
                elif time > accel_time:
                    velocity_output = - max_accel * (time - accel_time) + (max_accel * accel_time)
            if numpy.sign(target_distance) == -1:
                velocity_output = -velocity_output
            self.update(velocity_output, velocity_output)
            print(velocity_output)

    def drive_to_xy(self, x, y, straight_velocity, kP):
        start_time = self.time
        while NerdyMath.distance_formula(x, y, self.robot_x, self.robot_y) >= 2 and self.time - start_time < 1000:
            target_angle = math.degrees(math.atan2(x - self.robot_x, y - self.robot_y))
            angle = -(360 - self.robot_angle_deg) % 360
            error = target_angle - angle
            if error >= 180:
                error -= 360
            elif error <= -180:
                error += 360
            rot_velocity = error * kP
            self.update(straight_velocity + rot_velocity, straight_velocity - rot_velocity)

    def follow_path(self, path : BezierCurve, velocity, kP, kD):
        start_time = self.time + 0.00001
        time = self.time - start_time
        t = 1
        last_time = 0
        last_error = 0
        # while self.robot_pos < path.distance:
        # while time < 3000:
        goal_x = path.x_list[1]
        goal_y = path.y_list[1]
        while (NerdyMath.distance_formula(self.robot_x, self.robot_y, path.get_last_x(), path.get_last_y()) > 2) and t != len(path.y_list):
            print(time)
            print(t)
            time = self.time - start_time
            if t >= len(path.x_list):
                t = len(path.x_list) - 1
            goal_x = path.x_list[t]
            goal_y = path.y_list[t]
            distance = path.dist_list[t]
            target_angle = math.degrees(math.atan2(goal_x - self.robot_x, goal_y - self.robot_y))
            angle = -(360 - self.robot_angle_deg) % 360
            error = target_angle - angle
            if error >= 180:
                error -= 360
            elif error <= -180:
                error += 360
            rot_velocity = error * kP + (error - last_error)/(time - last_time) * kD
            self.update(velocity + rot_velocity, velocity - rot_velocity)
            # if self.robot_pos > distance:
            if NerdyMath.distance_formula(self.robot_x, self.robot_y, goal_x, goal_y) < 2:
                t += 1
            last_error = error
            last_time = time
            # print(t, goal_x, goal_y)
            # print(self.robot_x, self.robot_y)
        # print("taco", angular)
        plt.plot(self.x_list, self.y_list)
        plt.plot(path.x_list, path.y_list)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis([-100, 100, -100, 100])
        plt.legend(['robot position', 'path ' + str(kP) + ',' + str(kD)])
        plt.show()

    def drive_pure_pursuit(self, trajectory, lookahead, kP, going_forwards):
        start_time = self.time
        time = self.time - start_time
        index = 0
        current_seg = trajectory[0]
        while index != (len(trajectory) - 2):
            time = self.time - start_time
            current_seg = TrajectoryUtils.get_closer_segment_range(self.robot_x, self.robot_y, trajectory, index, 5)
            index = trajectory.index(current_seg)
            seg_2 = TrajectoryUtils.get_closer_segment(self.robot_x, self. robot_y, trajectory[index + 1], trajectory[index - 1])
            x1 = current_seg.x
            y1 = current_seg.y
            x2 = seg_2.x
            y2 = seg_2.y      
            if x2 == x1:
                a = 1
                b = -2 * self.robot_y
                c = (self.robot_y **2) - (lookahead**2) + (x1 - self.robot_x)**2
                goal_y1 = (-b - math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                goal_y2 = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                goal_x1 = x1
                goal_x2 = x2
            else:
                slope = (y2 - y1) / (x2 - x1)
                y_int = y2 - slope * x2
                a = (1 + slope ** 2)
                b = (-2 * self.robot_x) + (2 * slope * (y_int - self.robot_y))
                c = (self.robot_x ** 2) + (y_int - self.robot_y) ** 2 - lookahead ** 2
                goal_x1 = (-b - math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                goal_x2 = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
                goal_y1 = slope * goal_x1 + y_int
                goal_y2 = slope * goal_x2 + y_int
            # calculate intersections of circle from robot and line segment to follow
            
            angle = -(360 - self.robot_angle_deg) % 360
            target_angle1 = math.degrees(math.atan2(goal_x1 - self.robot_x, goal_y1 - self.robot_y))
            error1 = target_angle1 - angle
            if error1 >= 180:
                error1 -= 360
            elif error1 <= -180:
                error1 += 360
            target_angle2 = math.degrees(math.atan2(goal_x2 - self.robot_x, goal_y2 - self.robot_y))
            error2 = target_angle2 - angle
            if error2 >= 180:
                error2 -= 360
            elif error2 <= -180:
                error2 += 360
            # go to the lookahead point where there's less angular erorr
            if abs(error1) <= abs(error2):
                error = error1
                goal_x = goal_x1
                goal_y = goal_y1
            else:
                error = error2
                goal_x = goal_x2
                goal_y = goal_y2
            x_offset = NerdyMath.distance_formula(self.robot_x, self.robot_y, goal_x, goal_y) * math.cos(math.radians(error))
            #get x offset of looakead point relative to the robot   
            velocity = current_seg.velocity
            if x_offset > 0:
                drive_radius = (lookahead ** 2) / (2 * x_offset)
                # velocity = kP * velocity / drive_radius
                # drive at the calculated radius
                inner_vel = velocity * (drive_radius - (self.width/2))/(drive_radius + (self.width/2))
                if going_forwards:
                    if numpy.sign(error) == -1: 
                        self.update(inner_vel, velocity)
                    else:
                        self.update(velocity, inner_vel)
                else:
                    if numpy.sign(error) == 1: 
                        self.update(-inner_vel, -velocity)
                    else:
                        self.update(-velocity, -inner_vel)
            elif x_offset == 0:
                self.update(velocity, velocity)
                print("taco")
            # print(self.robot_angle_deg)
        print(time)
        plt.plot(self.x_list, self.y_list)
        x_list, y_list = TrajectoryUtils.get_x_y_lists(trajectory)
        plt.plot(x_list, y_list)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend(['robot position', 'path'])
        plt.axis([-100, 100, -100, 100])
        plt.show()

    def drive_pure_pursuit_poof(self, trajectory, lookahead, going_forwards):
            start_time = self.time
            time = self.time - start_time
            robot_index = 0
            index = 0
            current_seg = trajectory[0]
            vel_list = []
            time_list = []
            des_vel_list = []
            while time < 10.7:
            # while current_seg != trajectory[len(trajectory) - 2]:
                time = self.time - start_time
                current_seg = TrajectoryUtils.get_closer_segment_range(self.robot_x, self.robot_y, trajectory, robot_index, 5)
                robot_index = trajectory.index(current_seg)
                index = trajectory.index(current_seg) + lookahead
                if index > len(trajectory) - 1:
                    index = len(trajectory) - 1
                seg_2 = trajectory[index]
                goal_x = seg_2.x
                goal_y = seg_2.y
                angle = -(360 - self.robot_angle_deg) % 360
                target_angle = math.degrees(math.atan2(goal_x - self.robot_x, goal_y - self.robot_y))
                error = target_angle - angle
                lookahead_dist = NerdyMath.distance_formula(self.robot_x, self.robot_y, goal_x, goal_y)
                x_offset = NerdyMath.distance_formula(self.robot_x, self.robot_y, goal_x, goal_y) * math.cos(math.radians(error))
                #get x offset of looakead point relative to the robot   
                # print(math.cos(math.radians(error)))
                velocity = current_seg.velocity
                if abs(x_offset) > 0 and index != len(trajectory) - 1:
                # if abs(x_offset) > 0:
                    drive_radius = abs((lookahead_dist ** 2) / (2 * x_offset))
                    # drive at the calculated radius
                    # inner_vel = velocity * (drive_radius - (self.width/2))/(drive_radius + (self.width/2))
                   
                    vel_ratio = (drive_radius + (self.width/2))/(drive_radius - (self.width/2))
                    inner_vel = 2 * velocity / (vel_ratio + 1)
                    velocity = vel_ratio * inner_vel
                    if going_forwards:
                        if numpy.sign(x_offset) == -1: 
                            self.update(inner_vel, velocity)
                        else:
                            self.update(velocity, inner_vel)
                    else:
                        if numpy.sign(error) == 1: 
                            self.update(-inner_vel, -velocity)
                        else:
                            self.update(-velocity, -inner_vel)
                else:
                    self.update(velocity, velocity)
                    # print("taco")
                vel_list.append(self.left_vel)
                time_list.append(self.time)
                des_vel_list.append(self.right_vel)
                # print(self.linear_vel, velocity)
                # print("taco", error)
                # print(self.linear_vel, current_seg.velocity)
                print(velocity, inner_vel, self.linear_vel)
                print(drive_radius)
            plt.plot(self.x_list, self.y_list)
            x_list, y_list = TrajectoryUtils.get_x_y_lists(trajectory)
            plt.plot(x_list, y_list)
            plt.xlabel('x')
            plt.ylabel('y')
            plt.legend(['robot position', 'path'])
            plt.axis([-100, 100, -100, 100])
            plt.show()

            plt.plot(time_list, vel_list)
            plt.plot(time_list, des_vel_list)
            plt.xlabel('time')
            plt.ylabel('vel')
            plt.legend(['velocity', 'desired velocity'])
            plt.show()
            
    def drive_ramsete(self, trajectory, kB, kZeta):
        # kB > 0
        # 0 < kZeta < 1 
        start_time = self.time
        time = self.time - start_time
        index = 0
        current_seg = trajectory[0]
        while time < 5:
            if self.robot_pos > current_seg.position:
                index += 1
            time = self.time - start_time
            if index > len(trajectory) -1:
                index = len(trajectory) - 1
            current_seg = trajectory[index]
            x1 = current_seg.x
            y1 = current_seg.y
            x_error = x1 - self.robot_x
            y_error = y1 - self.robot_y
            robot_angle = pathfinder.d2r(navx_to_pf(pathfinder.boundHalfDegrees(self.robot_angle)))
            desired_heading = trajectory[index].heading
            print(index)
            print("desired angle", desired_heading)
            print("actual", robot_angle)
            print("unfiltered", self.robot_angle_deg)
            desired_angular_vel = (desired_heading - trajectory[index - 1].heading)/self.dt
            desired_vel = current_seg.velocity

            theta_error = desired_heading - robot_angle
            k1 = 2 * kZeta * math.sqrt((desired_angular_vel ** 2) + (kB * desired_vel ** 2))
            k2 = kB 
            vel = desired_vel * math.cos(theta_error) + k1 * ((x_error * math.cos(robot_angle)) + (y_error * math.sin(robot_angle)))
            angular_vel = desired_angular_vel + k2 * desired_vel * (math.sin(theta_error)/theta_error) * (y_error * math.cos(robot_angle) - x_error * math.sin(robot_angle)) + (k1 * theta_error)
            left_vel = (2* vel - (angular_vel * self.width))/2
            right_vel = 2 * vel - left_vel
            # self.update(right_vel, left_vel)
            self.update(left_vel, right_vel)
            
        plt.plot(self.x_list, self.y_list)
        x_list, y_list = TrajectoryUtils.get_x_y_lists(trajectory)
        plt.plot(x_list, y_list)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend(['robot position', 'path'])
        plt.show()
    
    def drive_pure_pursuit_nerd(self, trajectory, lookahead, going_forwards, kP, kD):
        start_time = self.time
        time = self.time - start_time
        robot_index = 0
        index = 0
        current_seg = trajectory[0]
        last_error = 0
        vel_list = []
        time_list = []
        des_vel_list = []
        while index != len(trajectory) - 1:
        # while time < 40:
            time = self.time - start_time
            current_seg = TrajectoryUtils.get_closer_segment_range(self.robot_x, self.robot_y, trajectory, robot_index, 5)
            robot_index = trajectory.index(current_seg)
            index = trajectory.index(current_seg) + lookahead
            if index > len(trajectory) - 1:
                index = len(trajectory) - 1
            seg_2 = trajectory[index]
            # goal_x = seg_2.x
            # goal_y = seg_2.y
            # angle = -(360 - self.robot_angle_deg) % 360
            # target_angle = math.degrees(math.atan2(goal_x - self.robot_x, goal_y - self.robot_y))
            target_angle = math.degrees(seg_2.heading)

            # current_angle = navx_to_pf(self.robot_angle_deg)
            # print(target_angle, current_angle)
            velocity = current_seg.velocity
            angle = self.robot_angle_deg
            if not going_forwards:
                velocity = -velocity
                angle += 180
            error = pathfinder.boundHalfDegrees(target_angle - angle)
            turn = error * kP + (error - last_error)/self.dt * kD
            self.update(velocity - turn, velocity + turn)
            last_error = error
            # print(index)
            vel_list.append(self.left_vel)
            time_list.append(self.time)
            des_vel_list.append(self.right_vel)
        plt.plot(self.x_list, self.y_list)
        x_list, y_list = TrajectoryUtils.get_x_y_lists(trajectory)
        plt.plot(x_list, y_list)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend(['robot position', 'path'])
        plt.axis([-100, 100, -100, 100])
        plt.show()
        # plt.plot(time_list, vel_list)
        # plt.plot(time_list, des_vel_list)
        # plt.xlabel('time')
        # plt.ylabel('vel')
        # plt.legend(['velocity', 'desired velocity'])
        # plt.show()

def navx_to_pf(a):
    # a = pathfinder.r2d(a)
    return (-(a +90) % 360) - 180

# print(navx_to_pf(0))

