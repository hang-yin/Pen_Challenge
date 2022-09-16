from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

import modern_robotics as mr
import pyrealsense2 as rs
import numpy as np
import time

robot = InterbotixManipulatorXS("px100", "arm", "gripper")

def waist_motion(x_coord, depth):
    radian = np.arctan2((depth - 0.17), (x_coord + 0.3))
    robot.arm.set_single_joint_position("waist", radian)
    return

def grab_pen():
    robot.gripper.set_pressure(1.0)
    robot.gripper.grasp()
    robot.arm.go_to_sleep_pose()
    time.sleep(2)
    robot.gripper.release()
    return

def wrist_motion(radian):
    robot.arm.go_to_home_pose()
    robot.arm.set_single_joint_position("wrist_angle", radian)
    return