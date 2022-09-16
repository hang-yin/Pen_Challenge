from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

import modern_robotics as mr
import pyrealsense2 as rs
import numpy as np
import time
import math

robot = InterbotixManipulatorXS("px100", "arm", "gripper")

def shoulder_motion(radian):
    robot.arm.set_single_joint_position("shoulder", radian)
    return

def elbow_motion(radian):
    robot.arm.set_single_joint_position("elbow", radian)
    return

def waist_motion(x_coord, depth):
    radian = np.arctan2(depth - 0.17, x_coord + 0.35)
    robot.arm.set_single_joint_position("waist", radian)
    return

def move_towards_pen(x_coord, depth):
    dist = calculate_distance(x_coord + 0.2, depth - 0.17)
    state = robot.arm.set_ee_cartesian_trajectory(x=dist - 0.1, z = -0.02)
    return state

def calculate_distance(edge1, edge2):
    distance = (edge1**2 + edge2**2)**(1/2)
    return distance

def grab_pen():
    robot.gripper.set_pressure(2.0)
    robot.gripper.grasp()
    robot.arm.go_to_sleep_pose()
    time.sleep(2)
    robot.gripper.release()
    return

def wrist_motion(radian):
    robot.arm.go_to_home_pose()
    robot.arm.set_single_joint_position("wrist_angle", radian)
    return

def go_to_home_pos():
    robot.arm.go_to_home_pose()
    return

def go_to_sleep_pos():
    robot.arm.go_to_sleep_pose()
    return