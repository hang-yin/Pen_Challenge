from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
# from interbotix_xs_modules.xs_robot.arm import InterbotixTurretXS
# The robot object is what you use to control the robot
robot = InterbotixManipulatorXS("px100", "arm", "gripper")
# turret = InterbotixTurretXS("px100")
mode = 'h'
# Let the user select the position

while mode != 'q':
    mode=input("[h]ome, [s]leep, [g]rasp, [r]elease, [f]orward, [b]ackward, [u]p, [d]own, [r]otate, [t]urn, [q]uit ")
    if mode == "h":
        robot.arm.go_to_home_pose()
    elif mode == "s":
        robot.arm.go_to_sleep_pose()
    elif mode == "g":
        robot.gripper.grasp()
    elif mode == "r":
        robot.gripper.release()
    elif mode == "f":
        # robot.arm.set_ee_cartesian_trajectory(x = )
        # print(robot.arm.get_ee_pose())
        pass
    elif mode == "t":
        robot.arm.go_to_home_pose()
        # turn turret
        robot.arm.set_single_joint_position("waist", 100, moving_time=5)