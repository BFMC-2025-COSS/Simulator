#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Path
from std_msgs.msg import String

from utils.msg import localisation

import math
import json
import os

class PurePursuit:
    def __init__(self):
        # ROS Node
        rospy.init_node('pure_pursuit_node', anonymous=True)

        # ROS Parameters
        self.look_ahead_dist = rospy.get_param('~look_ahead_dist', 0.068)
        self.wheel_base = rospy.get_param('~wheel_base', 0.034)
        self.desired_speed = rospy.get_param('~desired_speed', 20.0)

        # ROS Subscribers
        self.path_sub = rospy.Subscriber('/global_path', Path, self.path_callback)
        self.gps_sub = rospy.Subscriber('/automobile/localisation', localisation, self.gps_callback)

        # ROS Publishers
        self.command_pub = rospy.Publisher('/automobile/command', String, queue_size=1)

        # Internal variables
        self.path = []  # global path
        self.current_pos = (0.0, 0.0)
        self.current_yaw = 0.0  # radian
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def path_callback(self, msg):
        # Convert nav_msgs/Path to a list of (x, y)
        self.path = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.path.append((x, y))

    def gps_callback(self, msg):
        self.current_pos = (msg.posA, msg.posB)
        self.current_yaw = self.normalize_angle(msg.rotA)

    def normalize_angle(self, angle):
        while angle >= math.pi:
            angle -= 2.0 * math.pi
        while angle <= math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_nearest_index(self, x, y):
        # Find the nearest path index to (x, y)
        if not self.path:
            return None
        dists = [(x - px)**2 + (y - py)**2 for (px, py) in self.path]
        min_dist = min(dists)
        return dists.index(min_dist)

    def get_look_ahead_point(self, x, y, nearest_index):
        # Look ahead from nearest index until distance is >= look_ahead_dist
        dist_sum = 0.0
        for i in range(nearest_index + 1, len(self.path)):
            px, py = self.path[i]
            dist_sum = math.sqrt((px - x)**2 + (py - y)**2)
            if dist_sum >= self.look_ahead_dist:
                return (px, py)
        return None

    def compute_steering_angle(self, look_ahead_point):
        # Use the vehicle yaw and the position of the look-ahead point
        x, y = self.current_pos
        lx, ly = look_ahead_point
        angle_to_target = math.atan2(ly - y, lx - x)
        heading_error = angle_to_target - self.current_yaw
        # Normalize angle
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        # Pure pursuit formula
        return math.atan2(2.0 * self.wheel_base * math.sin(heading_error), self.look_ahead_dist)

    def control_loop(self, event):
        if not self.path:
            return

        # 1. find nearest point
        x, y = self.current_pos
        nearest_index = self.get_nearest_index(x, y)
        if nearest_index is None:
            return

        # 2. find lookahead point
        look_ahead_point = self.get_look_ahead_point(x, y, nearest_index)
        if not look_ahead_point:
            return

        # 3. compute steering
        steering_angle = self.compute_steering_angle(look_ahead_point)

        # 4. ROS logging
        lx, ly = look_ahead_point
        diff_x = lx - x
        diff_y = ly - y
        distance_to_look_ahead = math.sqrt(diff_x**2 + diff_y**2)
        
        os.system('cls' if os.name=='nt' else 'clear')
        rospy.loginfo(f"Current Position: x={x:.3f}, y={y:.3f}")
        rospy.loginfo(f"Look-Ahead Point: x={lx:.3f}, y={ly:.3f}")
        rospy.loginfo(f"Position Difference: Δx={diff_x:.3f}, Δy={diff_y:.3f}, Distance={distance_to_look_ahead:.3f}")
        rospy.loginfo(f"Speed: {self.desired_speed:.2f}")
        rospy.loginfo(f"Steering Angle: {-1.0 * math.degrees(steering_angle):.2f}°")

        # 4. publish cmd_vel
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
        pp = PurePursuit()
        pp.run()
    except rospy.ROSInterruptException:
        pass
