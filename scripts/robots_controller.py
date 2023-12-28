#!/usr/bin/python3
from robot import *
import numpy as np
import rospy
import argparse



position_robot = dict(R1=[[0, 0, 0], '/tb3_0/cmd_vel'],
                      R2=[[2, -2, 0], '/tb3_1/cmd_vel'],
                      R3=[[-2, -2, 0], '/tb3_2/cmd_vel'],
                      R4=[[0, -4, 0], '/tb3_3/cmd_vel'])

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description='Spawn a model in gazebo using the ROS API')
        parser.add_argument('-x', type=float, default=0, help='x component of initial position, meters')
        parser.add_argument('-y', type=float, default=0, help='y component of initial position, meters')
        parser.add_argument('-theta', type=float, default=0, help='roll angle of initial orientation, radians')
        parser.add_argument('-topic', type=str, default="", help='topic name of robot, string')
        parser.add_argument('-action', type=str, default="", help='action of robot Leader or Follower, string')
        parser.add_argument('-name', type=str, default="", help='name of robot, string')
        parser.add_argument('-name_leader', type=str, default="", help='name of robot_leader, string')
        parser.add_argument('-offset', type=str, default="", help='offset from robot leader, string')

        args = parser.parse_args(rospy.myargv()[1:])
        x = args.x
        y = args.y
        theta = args.theta
        topic = args.topic
        action = args.action
        name = args.name
        name_leader = args.name_leader
        offset = eval(args.offset)
        print(x, y, theta, topic)
        print(action, name, name_leader, offset)

        if action == "Leader":
            R = Leader(name)
            R.set_current_pose(np.array([x, y, theta]))
            R.set_gains(0.3, 0.05, 0.05)
            R.main()
        elif action == "Follower":
            R = Follower(name, name_leader)
            R.set_current_pose(np.array([x, y, theta]))
            R.set_gains(0.3, 0.05, 0.05)
            R.set_offset(offset[0], offset[1], offset[2])
            R.main()

    except rospy.ROSInterruptException:
        raise
