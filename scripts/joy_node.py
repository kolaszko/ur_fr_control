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
    def __init__(self, ip='192.168.0.245', joy_topic="/joy", gripper_topic="/gripper/command", scale=0.001):
        self.robot = robot_controller.Ur3(ip, 30003, 30002)
        self.sub = message_filters.Subscriber(joy_topic, Joy)
        self.cache = message_filters.Cache(self.sub, cache_size=1, allow_headerless=False)
        self.pub = rospy.Publisher(gripper_topic, String, queue_size=1)

        self.scale = scale
        self.orient_scale = 0.01
        self.last_pose = self.robot.get_pose()

    def callback(self):
        data = self.cache.getLast()
        if data is not None:
            if abs(data.axes[0]) < 0.1 and abs(data.axes[1]) < 0.1 and abs(data.axes[4]) < 0.1:
                print("zeros")
                return
            try:
                if data.buttons[5] == 0:
                    dx = -data.axes[1] * self.scale
                    dy = -data.axes[0] * self.scale
                    dz = -data.axes[3] * self.scale
                    self.last_pose = self.last_pose + np.array([dx, dy, dz, 0, 0, 0])
                elif data.buttons[5] == 1:
                    dx = -data.axes[1] * self.orient_scale
                    dy = -data.axes[0] * self.orient_scale
                    dz = -data.axes[3] * self.orient_scale
                    self.last_pose = self.last_pose + np.array([0, 0, 0, dx, dy, dz])

                self.robot.move([self.last_pose], False, True, a=1, v=0.05)

            except:
                print("Oops!")

            if data.buttons[0] == 1:
                self.pub.publish('close')
            elif data.buttons[3] == 1:
                self.pub.publish('open')
            elif data.buttons[1] == 1:
                self.pub.publish('semi_close')
            elif data.buttons[2] == 1:
                self.pub.publish('semi_open')


if __name__ == '__main__':
    rospy.init_node("ur3_joy_controller")
    controller = Ur3JoyControllerRos()
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        controller.callback()
        r.sleep()
