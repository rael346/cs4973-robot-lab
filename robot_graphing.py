from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
import matplotlib.pyplot as plt
from point_generation import *
import sys

"""
Launch command:
roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s

Enable Torque
rosservice call /wx250s/torque_enable "{cmd_type: 'group', name: 'all', enable: true}"


Joints list:
    wrist_rotate
    wrist_angle
    elbow_shadow
    elbow
    forearm_roll
    shoulder
    gripper
    shoulder_shadow
    waist
"""

STEP_SIMILARITY_TOLERANCE = 1 * (np.pi / 180.0)


def draw_figure_with_robot():
    shape_name = sys.argv[1]
    points = generate_points_from_shape(shape_name)
    save_points_figure(points, "graphs/" + shape_name + ".png")

    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")

    steps = steps_from_points(points)
    print("Before cleaning: ")
    for s in steps:
        print("[{:.2f}".format(s[0]), ", ", "{:.2f}".format(s[1]), "]")

    prev_steps = steps
    cleaner_steps = clean_steps(prev_steps, STEP_SIMILARITY_TOLERANCE)

    # This was an attempt to recursively clean steps to only have meaningful moves left over, but it
    # ended up just removing all the steps
    # while len(cleaner_steps) < len(prev_steps):
    #     print(len(cleaner_steps))
    #     prev_steps = cleaner_steps
    #     cleaner_steps = clean_steps(cleaner_steps)

    print("After cleaning: ")
    for s in cleaner_steps:
        print("[{:.2f}".format(s[0]), ", ", "{:.2f}".format(s[1]), "]")

    grab_pen(bot)
    move_to_writing_position(bot)
    _ = input("Give the robot the pen. Press Enter to continue...")
    draw_shape_from_steps(bot)
    _ = input("Take the pen from the robot, it can't be trusted. Enter to Continue...")
    move_to_final_position(bot)


def generate_points_from_shape(shape_to_draw):
    """
    Create list of points in order for the desired shape (specified as a String)
    """
    if shape_to_draw == "parabola":
        return parabola_points()
    elif shape_to_draw == "sine":
        return sine_wave_points()
    elif shape_to_draw == "heart":
        return generate_heart_points()
    else:
        raise RuntimeError("Invalid shape name " + shape_to_draw)


def draw_shape_from_steps(bot, steps):
    SCALE = 30
    for dy, dx in steps:
        print(dy, dx)
        dy_scale = dy / SCALE
        dx_scale = dx / SCALE

        # This function will move from a point to another similar to set_ee_pose_components
        # The difference is that it will move the end effector relative to the current position
        # and the ee path is not guaranteed to be a straight line
        bot.arm.set_ee_cartesian_trajectory(dx_scale, dy_scale, 0)

    bot.arm.set_ee_cartesian_trajectory(0, 0, 0.1)


def grab_pen(bot):
    """
    Get the robot to grab the pen
    Note: for some reason the robot doesn't open the gripper again after initial starts
    """
    bot.gripper.open()
    _ = input("Enter to continue...")
    bot.gripper.set_pressure(3)
    bot.gripper.close()


def move_to_writing_position(bot):
    # Hard-coded coordinates to get the robot arm in the right position to begin writing
    bot.arm.set_ee_pose_components(x=0.3, y=0, z=0.26)
    bot.arm.set_ee_pose_components(x=0.38, y=0, z=0.26)
    bot.arm.set_ee_pose_components(x=0.38, y=0, z=0.21)


def move_to_final_position(bot):
    # Commands to release the gripper and return to sleep pose one done drawing
    bot.gripper.set_pressure(0)
    bot.gripper.open()
    bot.arm.go_to_sleep_pose()


if __name__ == "__main__":
    draw_figure_with_robot()
