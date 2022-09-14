from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
# This script closes and opens the gripper twice, changing the gripper pressure half way through
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx200'
# Then change to this directory and type 'python gripper_control.py'

def main():
    bot = InterbotixManipulatorXS("px100", "arm", "gripper")
    bot.gripper.grasp()
    bot.gripper.release()

if __name__=='__main__':
    main()