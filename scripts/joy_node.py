#!/usr/bin/env python
import numpy as np
import sys

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import message_filters

import robot_controller

class Ur3JoyControllerRos():
    def __init__(self, ip='150.254.47.146', joy_topic="/joy", scale = 0.005):
        self.robot = robot_controller.Ur3(ip, 30003, 30002)
        self.sub = message_filters.Subscriber(joy_topic, Joy)
        self.cache = message_filters.Cache(self.sub, cache_size=1, allow_headerless=False)
        self.scale = scale
        self.counter = 0
        self.last_pose = self.robot.get_pose()

    def callback(self):
        data = self.cache.getLast()
        #print(data)
        if data is not None:
            if data.axes[0] == 0 and data.axes[1] == 0 and data.axes[4] == 0:
                print("zeros")
                return
            try:
                dx = data.axes[1] * self.scale
                dy = data.axes[0] * self.scale
                dz = data.axes[4] * self.scale
                self.last_pose =  self.last_pose + np.array([dx, dy, dz, 0, 0, 0])
                self.counter += 1
                if self.counter == 10:
                    self.robot.move([self.last_pose], False, True, a=1, v=0.05)
                    self.counter = 0

            except:
                print("Oops!")



if __name__ == '__main__':
    rospy.init_node("ur3_joy_controller")
    controller = Ur3JoyControllerRos()
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        controller.callback()
        r.sleep()