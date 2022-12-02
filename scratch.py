from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
import matplotlib.pyplot as plt

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

def main():
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    SCALE = 30


    # points = [(-1, 1, 0), (0, -2, 0), (1, 1, -0.005)]
    #points = generate_points(-1.8, 1.8, .599, lambda x: quad(1, 0, 0, x))

    # Parabola
    #points = generate_points(-1.3, 1.3, .2, lambda x: quad(1.2, 0, 0, x))

    # Sine Wave
    points = generate_points(-np.pi / 2, np.pi / 2, .1, lambda x: np.sin(x * 1.5 * np.pi))

    # Heart
    #points = generate_heart_points()

    plt.plot([pt[0] for pt in points], [pt[1] for pt in points], 'o', color='black')
    plt.savefig("graph.png")
    
    # points = [(0, 1), (1, 0), (-1, 0)]
    steps = steps_from_points(points)
    print("Before cleaning: ")
    for s in steps:
        print("[{:.2f}".format(s[0]), ", ", "{:.2f}".format(s[1]), "]")
    
    prev_steps = steps
    cleaner_steps = clean_steps(prev_steps)
    # while len(cleaner_steps) < len(prev_steps):
    #     print(len(cleaner_steps))
    #     prev_steps = cleaner_steps
    #     cleaner_steps = clean_steps(cleaner_steps)

    print("After cleaning: ")
    for s in cleaner_steps:
        print("[{:.2f}".format(s[0]), ", ", "{:.2f}".format(s[1]), "]")

    grab_pen(bot)

    # starting position
    bot.arm.set_ee_pose_components(x=0.3, y=0, z=0.26)
    bot.arm.set_ee_pose_components(x=0.38, y=0, z=0.26)
    bot.arm.set_ee_pose_components(x=0.38, y=0, z=0.21)

    _ = input("Give the robot the pen. Press Enter to continue...")


    for dy, dx in steps:
        print(dy, dx)
        dy_scale = dy / SCALE
        dx_scale = dx / SCALE
        # dz = 0.001 if dy < 0 else -0.001

        # This function will move from a point to another similar to set_ee_pose_components
        # The difference is that it will move the end effector relative to the current position
        # and the ee path is not guaranteed to be a straight line
        bot.arm.set_ee_cartesian_trajectory(dx_scale, dy_scale, 0)
    
    bot.arm.set_ee_cartesian_trajectory(0, 0, 0.1)

    _ = input("Take the pen from the robot, it can't be trusted. Press Enter to continue...")

    bot.gripper.set_pressure(0)
    bot.gripper.open()
    bot.arm.go_to_sleep_pose()

# def get_params():
#     start = float(input("Enter start of domain."))
#     end = float(input("Enter end of domain."))
#     step = float(input("Enter domain step size."))

#     func_name = input("Enter function name")
#     if func_name == "sine":
#         amplitude = input("Enter amplitude")
#         period
#     elif func_name == "quadratic":
#         pass
#     else:
#         raise NameError("Invalid function name")

#     return start, end, step, func
    

"""UTILS"""

def grab_pen(bot):
    """
    Get the robot to grab the pen 
    Note: for some reason the robot doesn't open the gripper again after initial starts
    """
    bot.gripper.open()
    _ = input("Enter to continue...")
    bot.gripper.set_pressure(3)
    bot.gripper.close()

def write_0(bot):
    # starting position
    bot.arm.set_ee_pose_components(x=0.4, y=0.1, z=0.065)
    bot.arm.set_ee_cartesian_trajectory(y=-0.025)
    bot.arm.set_ee_cartesian_trajectory(x=-0.025)
    bot.arm.set_ee_cartesian_trajectory(y=0.025)
    bot.arm.set_ee_cartesian_trajectory(x=0.025)

def write_1(bot):
    bot.arm.set_ee_pose_components(x=0.4, z=0.065)
    bot.arm.set_ee_cartesian_trajectory(y=0.05)

def quad(a, b, c, x):
    """
    Quadratic function with coefficients a, b, c and parameter x
    """
    return a * x**2 + b * x + c

def generate_points(s, l, step, function):
    """
    Generate points from the given function with step between x
    """
    ans = []
    for x in np.linspace(s, l, ((l - s) / step)):
        ans.append((x, function(x)))

    return ans

def generate_heart_points():
    HEART_SCALE = 10
    t = np.linspace(0, 2 * np.pi, 30)
    x = 16 * np.power(np.sin(t), 3)
    y = 13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4*t)

    x = x / HEART_SCALE
    y = y / HEART_SCALE
    return list(zip(x, y))

def steps_from_points(points):
    """
    Generate the difference between given points for set_ee_cartesian_trajectory
    """
    result = []
    prev = points[0]
    for point in points[1:]:
        result.append((point[0] - prev[0], point[1] - prev[1]))
        prev = point
    return result

def clean_steps(steps):
    result = []
    first_step_index = 0
    while first_step_index < len(steps) - 1:
        first_step = steps[first_step_index]
        next_step = steps[first_step_index + 1]
        # if curr_step is almost in the same direction as previous step, 
        # combine into one big step
        if (angle_between_steps(first_step, next_step)) < STEP_SIMILARITY_TOLERANCE:
            result.append((first_step[0] + next_step[0], first_step[1] + next_step[1]))
            first_step_index += 2
        else:
            result.append(first_step)
            first_step_index += 1 
    return result

def angle_between_steps(step_1, step_2):
    v1_u = step_1 / np.linalg.norm(step_1)
    v2_u = step_2 / np.linalg.norm(step_2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

if __name__=='__main__':
    main()