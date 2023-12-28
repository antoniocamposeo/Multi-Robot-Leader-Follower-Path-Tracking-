#!/usr/bin/python3
import rospy
import numpy as np

# Import some key libraries
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from robotics_2_project.msg import pose_message
from robotics_2_project.msg import path_message

position_robot = dict(R1=[[0, 0, 0], '/tb3_0/cmd_vel'],
                      R2=[[2, -2, 0], '/tb3_1/cmd_vel'],
                      R3=[[-2, -2, 0], '/tb3_2/cmd_vel'],
                      R4=[[0, -4, 0], '/tb3_3/cmd_vel'])


def from_polar_to_cartesian(p):
    rho = p[0]
    delta = p[1]
    gamma = p[2]
    x = rho * np.cos(delta + np.pi)
    y = rho * np.sin(delta + np.pi)
    theta = delta - gamma
    return x, y, theta


def from_cartesian_to_polar(q):
    x = q[0]
    y = q[1]
    theta = q[2]
    rho = np.sqrt(x ** 2 + y ** 2)
    gamma = np.arctan2(y, x) - theta + np.pi
    delta = gamma + theta
    return rho, delta, gamma


class Robot:

    def __init__(self, name):

        self.name = name
        self.current_pose = [None, None, None]  # current pose of robot
        self.goal_pose = [None, None, None]
        self.error = [None, None, None]
        # CONTROL
        self.control = None  # Class referred to all possible control actions
        self.k1 = None
        self.k2 = None
        self.k3 = None
        self.v = None
        self.w = None
        self.max_linear_speed = None
        self.min_linear_speed = None
        self.max_angular_speed = None
        self.min_angular_speed = None
        self.flag = 0

        # ROS STRUCTURE
        self.sub_gazebo_pose = None  # both
        self.pub_control = None  # both
        self.rate = 50
        self.topic_gazebo_states = '/gazebo/model_states'
        self.position_msg = None

        self.THRESHOLD = 0.07
        self.move_cmd = Twist()  # message for control actions

    def init_node(self):
        rospy.init_node("Robot_" + self.name)

    def get_position_msg(self, msg):
        global position_robot
        if self.flag == 0:
            for i in range(1, len(position_robot) + 1):  # check the robots order
                arr_pose = np.round(np.array(
                    [msg.pose[i].position.x, msg.pose[i].position.y, 0]))  # check just at the simulation beginning
                if np.array_equal(arr_pose, np.array(position_robot[self.name][0])):
                    self.position_msg = i

            print(self.position_msg)
            self.flag = 1

    def get_gains(self):
        return self.k1, self.k2

    def get_current_pose(self):
        return self.current_pose

    def pub_control_actions(self):
        if self.move_cmd is not None:
            self.pub_control.publish(self.move_cmd)
        else:
            print('Error: control actions not defined')
            raise ValueErrorS

    def set_control_action(self):
        self.move_cmd.linear.x = self.v
        self.move_cmd.angular.z = self.w

    def set_goal_pose(self, goal_pose):
        self.goal_pose = goal_pose

    def set_current_pose(self, current_pose):
        self.current_pose = current_pose

    def set_control(self, control):
        self.control = control  # TODO: CREATE A CLASS FOR CONTROL LAWS

    def set_gains(self, *args):
        if len(args) == 2:
            self.k1, self.k2 = args
        elif len(args) == 3:
            self.k1, self.k2, self.k3 = args

    def pose_callback(self, msg):
        self.get_position_msg(msg)
        pose = np.array([msg.pose[self.position_msg].position.x,
                         msg.pose[self.position_msg].position.y,
                         msg.pose[self.position_msg].position.z])

        orientation = np.array(euler_from_quaternion([
            msg.pose[self.position_msg].orientation.x,
            msg.pose[self.position_msg].orientation.y,
            msg.pose[self.position_msg].orientation.z,
            msg.pose[self.position_msg].orientation.w,
        ]))

        self.current_pose = np.array([pose[0], pose[1], orientation[2]])
        self.msg.x = self.current_pose[0]
        self.msg.y = self.current_pose[1]
        self.msg.theta = self.current_pose[2]

    def cartesian_controller(self, *args):
        x, y, theta = args[0]
        if np.abs(x) <= self.THRESHOLD and np.abs(y) <= self.THRESHOLD:
            self.v = 0
            self.w = 0
        else:
            self.v = - self.k1 * (x * np.cos(theta) + y * np.sin(theta))
            self.w = self.k2 * (np.arctan2(y, x) + np.pi - theta)

    def polar_controller(self, *args):
        x, y, theta = args[0]
        rho, delta, gamma = from_cartesian_to_polar([x, y, theta])
        if np.abs(x) <= self.THRESHOLD and np.abs(y) <= self.THRESHOLD:
            self.v = 0
            self.w = 0
        else:
            self.v = self.k1 * rho * np.cos(gamma)
            self.w = self.k2 * gamma + self.k1 * ((np.sin(gamma) * np.cos(gamma)) / gamma) * (
                    gamma + self.k3 * delta)


class Leader(Robot):

    def __init__(self, name):
        self.pub_leader = None  # publisher leader pose
        self.sub_path_pose = None  # subscribe path
        self.path_leader = None  # path to follow get from callback
        self.path_msg = path_message() # message for leader path
        self.msg = pose_message()  # message for leader pose
        super().__init__(name)

    def set_publisher_subscriber(self):
        self.sub_gazebo_pose = rospy.Subscriber(self.topic_gazebo_states, ModelStates, self.pose_callback)
        self.sub_path_pose = rospy.Subscriber('/path_leader', path_message, self.path_callback)
        self.pub_leader = rospy.Publisher('/' + self.name + '/pose_leader', pose_message, queue_size=1)
        self.pub_control = rospy.Publisher(position_robot[self.name][1], Twist, queue_size=1)

    def pub_pose_leader(self):
        if self.msg is not None:
            self.pub_leader.publish(self.msg)
        else:
            print('Error: pose_message not defined')
            raise ValueError

    def path_callback(self, msg_path):
        self.path_leader = np.array([msg_path.x_d, msg_path.y_d, msg_path.theta_d, msg_path.v_d, msg_path.w_d])

    def set_error(self):
        if self.path_leader is not None:
            self.error = np.array(self.current_pose) - np.array(self.path_leader[0:3])
            self.error[2] = self.current_pose[2]
        else:
            self.error = np.array([0, 0, 0])

    def main(self):
        self.init_node()
        r = rospy.Rate(self.rate)
        self.set_publisher_subscriber()

        while not rospy.is_shutdown():
            self.pub_pose_leader()
            self.set_error()
            self.polar_controller(self.error)
            self.set_control_action()
            self.pub_control_actions()
            print(self.v, self.w)
            r.sleep()


class Follower(Robot):

    def __init__(self, name, leader_name):
        self.leader_name = leader_name
        self.pub_follower_pose = None
        self.pose_leader = None
        self.sub_leader_pose = None
        self.msg = pose_message()  
        self.offset = np.array([0, 0, 0])

        super().__init__(name)

    def set_publisher_subscriber(self):
        self.sub_gazebo_pose = rospy.Subscriber(self.topic_gazebo_states, ModelStates, self.pose_callback)
        self.sub_leader_pose = rospy.Subscriber('/' + self.leader_name + '/pose_leader', pose_message,
                                                self.sub_pose_callback)
        self.pub_follower_pose = rospy.Publisher('/' + self.name + '/pose_leader', pose_message, queue_size=1)
        self.pub_control = rospy.Publisher(position_robot[self.name][1], Twist, queue_size=1)

    def sub_pose_callback(self, pose_message):
        self.pose_leader = np.array([pose_message.x, pose_message.y, pose_message.theta])

    def set_offset(self, *args):
        self.offset[0], self.offset[1], self.offset[2] = args

    def set_error(self):
        if self.pose_leader is not None:
            self.error = np.array(self.current_pose) - np.array([self.pose_leader[0], self.pose_leader[1], 0]) \
                         + np.array(self.offset)

        elif self.pose_leader is None:
            self.error = np.array([0, 0, 0])

    def pub_pose_follower(self):
        if self.msg is not None:
            self.pub_follower_pose.publish(self.msg)
        else:
            print('Error: pose_message not defined')
            raise ValueError

    def main(self):
        self.init_node()
        r = rospy.Rate(self.rate)
        self.set_publisher_subscriber()
        while not rospy.is_shutdown():
            self.pub_pose_follower()
            self.set_error()
            self.polar_controller(self.error)
            self.set_control_action()
            self.pub_control_actions()
            # print(self.error)
            r.sleep()
