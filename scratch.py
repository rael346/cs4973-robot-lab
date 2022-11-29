from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np

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

def main():
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    SCALE = 30
    grab_pen(bot)

    # points = [(-1, 1, 0), (0, -2, 0), (1, 1, -0.005)]
    #points = generate_points(-1.8, 1.8, .599, lambda x: quad(1, 0, 0, x))

    points = generate_points(-1.8, 1.8, .12, lambda x: np.sin(x * 2 * np.pi))

    # points = [(0, 1), (1, 0), (-1, 0)]
    steps = steps_from_points(points)
    
    # starting position
    bot.arm.set_ee_pose_components(x=0.4, y=0, z=0.18)

    for dy, dx in steps:
        print(dy, dx)
        dy_scale = dy / SCALE
        dx_scale = dx / SCALE
        # dz = 0.001 if dy < 0 else -0.001

        # This function will move from a point to another similar to set_ee_pose_components
        # The difference is that it will move the end effector relative to the current position
        # and the ee path is not guaranteed to be a straight line
        bot.arm.set_ee_cartesian_trajectory(dx_scale, dy_scale, 0)
    
    bot.gripper.set_pressure(0)
    bot.gripper.open()
    bot.arm.go_to_sleep_pose()
    

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
    for x in np.arange(s, l, step):
        ans.append((x, function(x)))

    return ans

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

if __name__=='__main__':
    main()