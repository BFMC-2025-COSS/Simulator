#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from utils.msg import localisation

import math
import json
import os

class PurePursuit:
    def __init__(self, look_ahead_dist, wheel_base):
        self.look_ahead_dist = look_ahead_dist
        self.wheel_base = wheel_base

        self.path = []  # global path
        self.current_pos = (0.0, 0.0)
        self.current_yaw = 0.0  # radian
        self.look_ahead_point = None
    
    def ros_init(self):
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
        self.current_pos_pub = rospy.Publisher('/visualization/current_pos', Marker, queue_size=1)
        self.look_ahead_pub = rospy.Publisher('/visualization/look_ahead', Marker, queue_size=1)
        self.path_marker_pub = rospy.Publisher('/visualization/look_ahead_line', Marker, queue_size=1)

        # Internal variables
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

    def visualize_current_position(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Coordinate frame used in RViz
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pure_pursuit"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.current_pos[0]
        marker.pose.position.y = self.current_pos[1]
        marker.pose.position.z = 0.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Opaque

        return marker

    def visualize_look_ahead_point(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pure_pursuit"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.look_ahead_point[0]
        marker.pose.position.y = self.look_ahead_point[1]
        marker.pose.position.z = 0.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # Blue
        marker.color.a = 1.0  # Opaque

        return marker

    def visualize_look_ahead_line(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pure_pursuit"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05  # Line thickness

        marker.color.r = 0.0
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0
        marker.color.a = 1.0  # Opaque

        start_point = Point()
        start_point.x = self.current_pos[0]
        start_point.y = self.current_pos[1]
        start_point.z = 0.0

        end_point = Point()
        end_point.x = self.look_ahead_point[0]
        end_point.y = self.look_ahead_point[1]
        end_point.z = 0.0

        marker.points.append(start_point)
        marker.points.append(end_point)

        return marker

    def normalize_angle(self, angle):
        while angle >= math.pi:
            angle -= 2.0 * math.pi
        while angle <= math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def get_nearest_index(self, path, x, y):
        # Find the nearest path index to (x, y)
        if not path:
            return None
        dists = [(x - px)**2 + (y - py)**2 for (px, py) in path]
        min_dist = min(dists)
        return dists.index(min_dist)

    def get_look_ahead_point(self, path, x, y, nearest_index, look_ahead_dist):
        # Look ahead from nearest index until distance is >= look_ahead_dist
        if not path:
            return None
        for i in range(nearest_index + 1, len(path)):
            px, py = path[i]
            dist_val = math.sqrt((px - x)**2 + (py - y)**2)
            if dist_val >= look_ahead_dist:
                return (px, py)
        return None

    def compute_steering_angle(self, path, current_pos, current_yaw, look_ahead_dist=None, wheel_base=None):
        # Use the vehicle yaw and the position of the look-ahead point
        if not path:
            self.look_ahead_point = None
            return 0.0
        
        if look_ahead_dist is None:
            look_ahead_dist = self.look_ahead_dist
        if wheel_base is None:
            wheel_base = self.wheel_base

        x, y = current_pos
        nearest_index = self.get_nearest_index(path, x, y)
        if nearest_index is None:
            self.look_ahead_point = None
            return 0.0

        look_ahead_point = self.get_look_ahead_point(path, x, y, nearest_index, look_ahead_dist)
        self.look_ahead_point = look_ahead_point
        if not look_ahead_point:
            return 0.0

        lx, ly = look_ahead_point
        angle_to_target = math.atan2(ly - y, lx - x)
        heading_error = angle_to_target - current_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        steering_angle = math.atan2(
            2.0 * wheel_base * math.sin(heading_error),
            look_ahead_dist
        )

        return steering_angle

    def control_loop(self, event):
        if not self.path:
            return

        # 1. compute steering angle
        steering_angle = self.compute_steering_angle(
            self.path,
            self.current_pos,
            self.current_yaw
        )

        # 2. Check look ahead point is valid
        if not self.look_ahead_point:
            return

        # 3. ROS logging
        lx, ly = self.look_ahead_point
        x, y = self.current_pos
        diff_x = lx - x
        diff_y = ly - y
        distance_to_look_ahead = math.sqrt(diff_x**2 + diff_y**2)
        
        os.system('cls' if os.name=='nt' else 'clear')
        rospy.loginfo(f"Current Position: x={x:.3f}, y={y:.3f}")
        rospy.loginfo(f"Look-Ahead Point: x={lx:.3f}, y={ly:.3f}")
        rospy.loginfo(f"Position Difference: Δx={diff_x:.3f}, Δy={diff_y:.3f}, Distance={distance_to_look_ahead:.3f}")
        rospy.loginfo(f"Speed: {self.desired_speed:.2f}")
        rospy.loginfo(f"Steering Angle: {-1.0 * math.degrees(steering_angle):.2f}°")

        # 4. Visualization
        current_position_marker = self.visualize_current_position()
        look_ahead_point_marker = self.visualize_look_ahead_point()
        look_ahead_line_marker = self.visualize_look_ahead_line()

        self.current_pos_pub.publish(current_position_marker)
        self.look_ahead_pub.publish(look_ahead_point_marker)
        self.path_marker_pub.publish(look_ahead_line_marker)

        # 5. publish command
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
        pp = PurePursuit(0.068, 0.034)
        pp.ros_init()
        pp.run()
    except rospy.ROSInterruptException:
        pass
