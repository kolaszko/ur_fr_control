#!/usr/bin/env python
import numpy as np
import sys

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import message_filters

import robot_controller


class Ur3JoyControllerRos():
    def __init__(self, robot_ip, joy_topic="/joy", gripper_topic="/gripper/command", scale=0.001):
        print('Connecting to {}'.format(robot_ip))
        self.robot = robot_controller.Ur3(robot_ip, 30003, 30002)
        self.sub = message_filters.Subscriber(joy_topic, Joy)
        self.cache = message_filters.Cache(self.sub, cache_size=1, allow_headerless=False)
        self.pub = rospy.Publisher(gripper_topic, String, queue_size=2)

        self.scale = scale
        self.orient_scale = 0.01
        self.last_pose = self.robot.get_pose()
        self.last_data_h_seq = None
        self.dead_zone = 0.2
        self.update_count = 20
        self.update_counter = 0

    def callback(self):
        data = self.cache.getLast()
        if data is not None and data.header.seq != self.last_data_h_seq:
            self.last_data_h_seq = data.header.seq
            # if not(abs(data.axes[0]) < 0.1 and abs(data.axes[1]) < 0.1 and abs(data.axes[3]) < 0.1):
            dx, dy, dz = self.remove_dead_zone((data.axes[1], data.axes[0], data.axes[3]))

            if (dx or dy or dz) != 0:
                try:
                    if data.buttons[5] == 0:
                        dx = -dx * self.scale
                        dy = -dy * self.scale
                        dz = -dz * self.scale
                        self.last_pose = self.last_pose + np.array([dx, dy, dz, 0, 0, 0])
                    elif data.buttons[5] == 1:
                        dx = -dx * self.orient_scale
                        dy = -dy * self.orient_scale
                        dz = -dz * self.orient_scale
                        self.last_pose = self.last_pose + np.array([0, 0, 0, dx, dy, dz])

                    self.robot.move([self.last_pose], False, True, a=1, v=0.05)
                    self.update_counter += 1
                    print(self.update_counter)

                except:
                    print("Oops!")

            if self.update_counter == self.update_count:
                self.update_counter = 0
                print('===================================================')
                print(self.last_pose)
                print('===================================================')

            if data.buttons[0] == 1:
                self.pub.publish('close')
            elif data.buttons[3] == 1:
                self.pub.publish('open')
            elif data.buttons[1] == 1:
                self.pub.publish('semi_close')
            elif data.buttons[2] == 1:
                self.pub.publish('semi_open')

    def remove_dead_zone(self, values):
        sticks = []
        for v in values:
            if abs(v) > self.dead_zone:
                sticks.append(v)
            else:
                sticks.append(0)
        return sticks


def parse_params(namespace='ur_joy'):
    namespace = rospy.get_param('~namespace', namespace)
    return {
        'namespace': namespace,
        'robot_ip': rospy.get_param('~robot_ip')
    }


if __name__ == '__main__':
    namespace = 'ur3_joy_controller'
    rospy.init_node(namespace)
    params = parse_params()

    controller = Ur3JoyControllerRos(robot_ip=params['robot_ip'])
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        controller.callback()
        r.sleep()