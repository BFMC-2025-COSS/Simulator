#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from utils.msg import localisation, IMU

from pure_pursuit import *
from mpc import *

import math
import json
import os

class Control:
    def __init__(self):
        # ROS Node
        rospy.init_node('control_node', anonymous=True)

        # ROS Parameters
        self.path_topic = rospy.get_param('~path_topic', '/global_path')
        self.gps_topic = rospy.get_param('~gps_topic', '/automobile/localisation')
        self.imu_topic = rospy.get_param('~imu_topic', '/automobile/IMU')
        self.command_topic = rospy.get_param('~command_topic', '/automobile/command')

        self.controller_type = rospy.get_param('~controller_type', 'mpc')

        self.look_ahead_dist = rospy.get_param('~look_ahead_dist', 0.38)
        self.wheel_base = rospy.get_param('~wheel_base', 0.26)
        self.desired_speed = rospy.get_param('~desired_speed', 0.3)

        # ROS Subscribers
        self.path_sub = rospy.Subscriber(self.path_topic, Path, self.path_callback)
        self.gps_sub = rospy.Subscriber(self.gps_topic, localisation, self.gps_callback)
        self.imu_sub = rospy.Subscriber(self.imu_topic, IMU, self.imu_callback, queue_size=1)

        # ROS Publishers
        self.command_pub = rospy.Publisher(self.command_topic, String, queue_size=10)
        self.current_pos_pub = rospy.Publisher('/visualization/current_pos', Marker, queue_size=1)
        self.look_ahead_pub = rospy.Publisher('/visualization/look_ahead', Marker, queue_size=1)
        self.path_marker_pub = rospy.Publisher('/visualization/look_ahead_line', Marker, queue_size=1)

        # Internal variables
        self.path = []  # global path
        self.current_pos = (0.0, 0.0)
        self.current_yaw = 0.0  # radian
        self.path_recevied = False
        self.gps_recevied = False
        self.imu_recevied = False
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

        self.pp = PurePursuit(self.look_ahead_dist, self.wheel_base)
        self.mpc = NonlinearMPCController(dt=0.25, horizon=10, wheelbase=self.wheel_base)

        rospy.loginfo("[Control] ROS node started.")
        rospy.loginfo("[Control] Waiting for messages...\n  Path topic: %s\n  GPS topic: %s\n  IMU topic: %s", self.path_topic, self.gps_topic, self.imu_topic)

    def normalize_angle(self, angle):
        while angle >= math.pi:
            angle -= 2.0 * math.pi
        while angle <= math.pi:
            angle += 2.0 * math.pi
        return angle

    def path_callback(self, msg):
        if self.path_recevied:
            return
        
        self.path = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.path.append((x, y))

        self.path_recevied = True

    def gps_callback(self, msg):
        if not self.imu_callback:
            return
        
        x_center, y_center = msg.posA, msg.posB

        x_rear = x_center - (self.wheel_base / 2) * math.cos(self.current_yaw)
        y_rear = y_center - (self.wheel_base / 2) * math.sin(self.current_yaw)

        self.current_pos = (x_rear, y_rear)

        self.gps_recevied = True
        
    def imu_callback(self, msg):
        self.current_yaw = self.normalize_angle(msg.yaw)
        self.imu_recevied = True

    def control_loop(self, event):
        if not self.path_recevied or not self.gps_recevied or not self.imu_recevied:
            return

        if self.controller_type.lower() == 'purepursuit':
            speed = self.desired_speed
            steering_angle = self.pp.compute_steering_angle(
                self.path, 
                self.current_pos, 
                self.current_yaw
                )
            rospy.loginfo("[Control] Using PurePursuit -> speed=%.3f, steering_angle=%.3f deg", speed, math.degrees(steering_angle))

        elif self.controller_type.lower() == 'mpc':
            speed, steering_angle = self.mpc.compute_control_command(
                self.path, 
                self.current_pos, 
                self.current_yaw,
                self.desired_speed
                )
            rospy.loginfo("[Control] Using MPC -> speed=%.3f, steering_angle=%.3f deg", speed, math.degrees(steering_angle))

        command = {}
        command['action'] =  '1'
        command['speed'] = float(speed)
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
