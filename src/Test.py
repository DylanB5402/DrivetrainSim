import math
# import NerdyMath
from BezierCurve import BezierCurve
# from MotionProfile import *
# from NerdyTrajectory import NerdyTrajectory
# from TestTrajectory import TestTrajectory
from Drivetrain import Drivetrain
import pathfinder as pf
import TrajectoryUtils
from TrajectoryFactory import TrajectoryFactory

drivetrain = Drivetrain(2.5, 0.02, 28, 0)
# drivetrain.turn_to_angle_PID(45, 0.1, 0)
# drivetrain.drive_forward_PID(100, 0.1, 0)
# drivetrain.drive_at_heading(90, 0.01, 1000, 2)
# drivetrain.arc_turn(45, 0.1, False, 1)
# drivetrain.radius_turn(20, 1, 126, False)
# drivetrain.drive_motion_profile(-100, 10, 1)
# drivetrain.drive_to_xy(0, -100, 0.5, 0.01)
# drivetrain.drive_pure_pursuit(bezier, 5, 1, 5, 1, True, 1)
# drivetrain.set_position(100, 0)
# drivetrain.follow_path(bezier, 1, 0.01, 0)
# drivetrain.drive_pure_pursuit_2(opposite_scale_auto, 10, 5, 0.1, True)
# drivetrain.drive_forward_PID(100, 0.1, 0)
# print(1)
# drivetrain.turn_to_angle_PID(180, 0.1, 0)
# drivetrain.drive_pure_pursuit_4(bezier, 10, 5, 0, True)
# drivetrain.graph()
# print(1) 
# print(bezier_2.get_closest_point(0, 0))



right_switch_auto = pf.generate( 
                                [pf.Waypoint(16, 0, math.radians(90)),
                                pf.Waypoint(21.5, 10, math.radians(90)) ], 
                                pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
                               dt=0.05, # 50WWms
                               max_velocity=1.7,
                               max_acceleration=2.0,
                               max_jerk=60.0)[1]

right_to_right_scale_auto = pf.generate( 
                                        [ pf.Waypoint(28, 0, math.radians(90)),
                                        pf.Waypoint(28, 14, math.radians(90)),
                                        pf.Waypoint(25, 21, math.radians(120))],
                                        pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
                               dt=0.02, # 50WWms
                               max_velocity= 5,
                               max_acceleration=2.0,
                               max_jerk=60.0)[1]

test_auto = pf.generate( 
                                        [ pf.Waypoint(28, 0, math.radians(90)),
                                        pf.Waypoint(28, 10, math.radians(90)),
                                        pf.Waypoint(25, 12, math.radians(0)),
                                        pf.Waypoint(23, 10, math.radians(-90)),
                                        pf.Waypoint(23, 6, math.radians(-90))],
                                        pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
                               dt=0.02, # 50WWms
                               max_velocity= 5,
                               max_acceleration=2.0,
                               max_jerk=60.0)[1]
# drivetrain.turn_to_angle_PID(180, 0.1, 0)
# drivetrain.drive_forward_PID(-100, 0.1, 0)
# drivetrain.drive_pure_pursuit_pathfinder(right_switch_auto, 3, True)
# print(len(right_switch_auto))
# print(right_switch_auto[155])
drivetrain.drive_pure_pursuit(test_auto, 3, True)
# print(drivetrain.robot_x, drivetrain.robot_y)
# drivetrain.graph()
# drivetrain.set_position(17, 0)
# drivetrain.drive_ramsete(right_to_right_scale_auto, 0, 0)
# pf.serialize("src/right_switch_auto.traj", right_switch_auto)
# print(pf.deserialize("right_switch_auto.traj"))
# traj_factory = TrajectoryFactory("paths/", 2)
# traj_factory.add_trajectory("right_switch_auto", right_switch_auto)
# traj_factory.add_trajectory("right_to_right_scale_auto", right_to_right_scale_auto)
# traj_factory.save_trajectories()