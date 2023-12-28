#!/usr/bin/python3
import roslaunch
import rospy
import numpy as np

value = 'tb3_0'
value1 = '1.0'

arg_1 = 'tb3:=' + value
arg_2 = 'tb3_x_pos' + value1

value2 = 'tb3_2'
value3 = '3.0'
arg_3 = 'tb3:=' + value2
arg_4 = 'tb3_x_pos' + value3
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args = ['robotics_2_project', 'pylauncher.launch', arg_1, arg_2]
roslaunch_args = cli_args[2:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

cli_args_1 = ['robotics_2_project', 'pylauncher.launch', arg_3, arg_4]
roslaunch_args_1 = cli_args_1[2:]
roslaunch_file_1 = [(roslaunch.rlutil.resolve_launch_arguments(cli_args_1)[0], roslaunch_args_1)]
launch_files = [roslaunch_file,roslaunch_file_1]

for launch in launch_files:
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file, roslaunch_file_1)

    parent.start()

    try:
        parent.spin()
    finally:
        parent.shutdown()
