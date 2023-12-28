#!/usr/bin/python3
import cubicpath
import numpy as np
import rospy
from cubicpath import *
from robotics_2_project.msg import path_message

path_msg = path_message()

pub_traj = rospy.Publisher('/path_leader', path_message, queue_size=10, )


def set_message(x_d, y_d, theta_d, v_d, w_d):
    path_msg.x_d = x_d
    path_msg.y_d = y_d
    path_msg.theta_d = theta_d
    path_msg.v_d = v_d
    path_msg.w_d = w_d


def main():
    try:
        rospy.init_node("Path_Generator")
        r = rospy.Rate(50)
        t_steps = np.linspace(0, 30, 1500)
        # Initial e Final Pose
        waypoints = np.array([[0., 0., 0.], [3., 3., 0.], [2., 6., 0.]])
        # Initial and Final Speed
        k = 1
        t = 0
        i = 0
        step = 1

        path = compute_path_from_waypoints(waypoints, k, t_steps)
        if pub_traj.get_num_connections() >= 1:
            while not rospy.is_shutdown():
                leng = np.array(path[i]['x']).shape[0]
                if t < leng:
                    set_message(path[i]['x'][t], path[i]['y'][t], path[i]['theta'][t], path[i]['v'][t], path[i]['w'][t])
                    t = t + step
                elif t >= leng and i <= len(path):
                    set_message(path[i]['x'][t-1], path[i]['y'][t-1], path[i]['theta'][t-1], path[i]['v'][t-1],
                                path[i]['w'][t-1])
                    i = i + 1
                    t = 0
                elif i > len(path):
                    i = len(path)
                    t = leng + step
                    set_message(path[i]['x'][-1], path[i]['y'][-1], path[i]['theta'][-1], path[i]['v'][-1],
                                path[i]['w'][-1])

                pub_traj.publish(path_msg)
                r.sleep()

    except Exception as e:
        print(" An Exception occurred during the control loop. Terminating gracefully.")
        print(" Exception: ", e)


if __name__ == '__main__':
    main()
