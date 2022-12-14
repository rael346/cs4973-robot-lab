# Robotics Lab Project: Desmos Graphing Robot (featuring Finneas the one finger robot)

## Overview

Team: Ziggy Stardust (Duy Tran, William Cutler, Sam Ji)

The goal of the robot is to graph a given function using pen and paper in its hand. We use an Interbotix WX250s robotic arm https://www.trossenrobotics.com/widowx-250-robot-arm.aspx.

To run the program, use "python robot_graphing.py [SHAPE]" where SHAPE is either 'sine', 'parabola', or 'heart' to specify the desired function.

## Video of Robot Arm Drawing Sine Wave

https://user-images.githubusercontent.com/38445583/207697561-b69bbab9-77f8-4afa-a94e-c3c3fff530fa.mp4



## How it Works

Human participation is involved in the robot drawing workflow, specifically to give and take away the pen as well as setting up the paper for it to draw on. The program will print instructions to the terminal and pause to allow the human to complete them before the robot takes further actions.

As a side-effect useful for debugging and showing to friends, the points that the robot is attempting to draw are saved to the 'graphs/' folder, depending on the function specified.

Main Steps of the Program

1. Convert user-specified function to a list of points, (x, y), by applying the function to a predetermined np.linspace domain. This is like creating the "dots" of a "connect-the-dots".

2. Compute relative motion directions as the differences between two adjacent points in the list. These are what the robot arm uses to actually connect the dots. Much like Maps telling you "in 0.25 miles, turn right".

3. Move the robot arm into starting position, a human will insert the pen into its holder, and then the robot will move its arm using the relative motion directions to draw the figure (takes a minute or so depending on figure complexity).

4. Robot arm returns to home position.
