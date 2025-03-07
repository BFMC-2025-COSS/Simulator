#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from utils.msg import localisation

from pure_pursuit import *

import math
import json
import os

class Control:
    def __init__(self):
        # ROS Node
        rospy.init_node('control_node', anonymous=True)

        # ROS Parameters
        self.look_ahead_dist = rospy.get_param('~look_ahead_dist', 0.068)
        self.wheel_base = rospy.get_param('~wheel_base', 0.034)
        self.desired_speed = rospy.get_param('~desired_speed', 20.0)

        # ROS Subscribers
        self.path_sub = rospy.Subscriber('/global_path', Path, self.path_callback)
        self.gps_sub = rospy.Subscriber('/automobile/localisation', localisation, self.gps_callback)

        # ROS Publishers
        self.command_pub = rospy.Publisher('/automobile/command', String, queue_size=1)
        self.current_pos_pub = rospy.Publisher('/visualization/current_pos', Marker, queue_size=1)
        self.look_ahead_pub = rospy.Publisher('/visualization/look_ahead', Marker, queue_size=1)
        self.path_marker_pub = rospy.Publisher('/visualization/look_ahead_line', Marker, queue_size=1)

        # Internal variables
        self.path = []  # global path
        self.current_pos = (0.0, 0.0)
        self.current_yaw = 0.0  # radian
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        self.pp = PurePursuit(self.look_ahead_dist, self.wheel_base)

    def normalize_angle(self, angle):
        while angle >= math.pi:
            angle -= 2.0 * math.pi
        while angle <= math.pi:
            angle += 2.0 * math.pi
        return angle

    def path_callback(self, msg):
        self.path = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.path.append((x, y))

    def gps_callback(self, msg):
        self.current_pos = (msg.posA, msg.posB)
        self.current_yaw = self.normalize_angle(msg.rotA)

    def control_loop(self, event):
        if not self.path:
            return

        steering_angle = self.pp.compute_steering_angle(
            self.path,
            self.current_pos,
            self.current_yaw
        )

        command = {}
        command['action'] =  '1'
        command['speed'] = float(self.desired_speed / 100.0)
        command = json.dumps(command)
        self.command_pub.publish(command)

        command = {}
        command['action'] = '2'
        command['steerAngle'] = float(-1.0 * math.degrees(steering_angle))
        command = json.dumps(command)
        self.command_pub.publish(command)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        c = Control()
        c.run()
    except rospy.ROSInterruptException:
        pass
