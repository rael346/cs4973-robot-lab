import numpy as np
import matplotlib.pyplot as plt


def sine_wave_points():
    print("Generating sine wave")
    return generate_points(
        -np.pi / 2, np.pi / 2, 0.1, lambda x: np.sin(x * 1.5 * np.pi)
    )


def parabola_points():
    return generate_points(-1.3, 1.3, 0.2, lambda x: quad(1.2, 0, 0, x))


def generate_heart_points():
    """Points for a heart.
    https://www.wolframalpha.com/input/?i=x+%3D+16+sin%5E3+t%2C+y+%3D+%2813+cos+t+-+5+cos+2t+-+2+cos+3t+-+cos+4t%29
    """
    HEART_SCALE = 10
    t = np.linspace(0, 2 * np.pi, 30)
    x = 16 * np.power(np.sin(t), 3)
    y = 13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)

    x = x / HEART_SCALE
    y = y / HEART_SCALE
    return list(zip(x, y))


def generate_points(s, l, step, function):
    """
    Generate points from the given function with step between x
    """
    ans = []
    for x in np.linspace(s, l, round(((l - s) / step))):
        ans.append((x, function(x)))

    return ans


def quad(a, b, c, x):
    """
    Quadratic function with coefficients a, b, c and parameter x
    """
    return a * x ** 2 + b * x + c


def save_points_figure(points, fname):
    """
    Save points to a ply.plot .png file
    """
    plt.plot([pt[0] for pt in points], [pt[1] for pt in points], "o", color="black")
    plt.savefig(fname)


def steps_from_points(points):
    """
    Generate the difference between given points for set_ee_cartesian_trajectory.
    Given ordered list of absolute points, computed the list of vectors to move from the first point
    to the next point, and so on until the last one.

    Returns empty list if given < 2 points.
    """
    if len(points) <= 1:
        return []

    result = []
    prev = points[0]
    for point in points[1:]:
        result.append((point[0] - prev[0], point[1] - prev[1]))
        prev = point
    return result


def clean_steps(steps, step_similarity_tolerance):
    """
    Attempts to merge steps in nearly identical directions to improve speed of drawing. Two adjacent steps
    will be represented as a single merged step in the result if their directions are within the given angle tolerance (in degrees).
    """
    result = []
    first_step_index = 0
    while first_step_index < len(steps) - 1:
        first_step = steps[first_step_index]
        next_step = steps[first_step_index + 1]
        # if curr_step is almost in the same direction as previous step,
        # combine into one big step
        if (angle_between_steps(first_step, next_step)) < step_similarity_tolerance:
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
