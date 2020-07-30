#!/usr/bin/env python
import numpy as np
import sys

import rospy
from geometry_msgs.msg import Twist
import message_filters

import robot_controller


class Ur3CmdVelControllerRos():
    def __init__(self, ip='150.254.47.146', joy_topic="/cmd_vel", scale = 0.005):
        self.robot = robot_controller.Ur3(ip, 30003, 30002)
        self.sub = message_filters.Subscriber(joy_topic, Twist)
        self.cache = message_filters.Cache(self.sub, cache_size=1, allow_headerless=True)
        self.scale = scale
        self.last_pose = self.robot.get_pose()
        self.bounds_x = (0.225, -0.225)
        self.bounds_y = (0.315, -0.315)

    def callback(self):
        data = self.cache.getLast()
        if data is not None:
            if data.linear.x == 0 and data.angular.z ==0:
                return
            try:

                x = 2 * (data.linear.x - self.bounds_x[1])/(self.bounds_x[0] - self.bounds_x[1]) - 1
                y = 2 * (data.angular.z - self.bounds_y[1]) / (self.bounds_y[0] - self.bounds_y[1]) - 1

                dx = x * self.scale
                dy = y * self.scale
                self.last_pose += np.array([dx, dy, 0, 0, 0, 0])
                self.robot.move([self.last_pose], False, True, a=1, v=0.05)

            except:
                print("Oops!")



if __name__ == '__main__':
    rospy.init_node("ur3_fr_joy_controller")
    controller = Ur3CmdVelControllerRos()
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        controller.callback()
        r.sleep()