def write_0(bot):
    # Hardcoded coordinates to draw a '0', NOT USED
    bot.arm.set_ee_pose_components(x=0.4, y=0.1, z=0.065)
    bot.arm.set_ee_cartesian_trajectory(y=-0.025)
    bot.arm.set_ee_cartesian_trajectory(x=-0.025)
    bot.arm.set_ee_cartesian_trajectory(y=0.025)
    bot.arm.set_ee_cartesian_trajectory(x=0.025)


def write_1(bot):
    # Hardcoded coordinates to draw a '1', NOT USED
    bot.arm.set_ee_pose_components(x=0.4, z=0.065)
    bot.arm.set_ee_cartesian_trajectory(y=0.05)
