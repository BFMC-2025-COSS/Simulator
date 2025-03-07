#!/usr/bin/env python3

import rospy
import rospkg
import networkx as nx
import os
import yaml
import numpy as np
from scipy.interpolate import interp1d, CubicSpline
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from node_to_path.msg import PathWithDotted, DottedPose
import math  # <- 추가 필요!

class NodeToPath:
    def __init__(self):
        rospy.init_node('node_to_path', anonymous=True)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('node_to_path')

        default_graphml_path = os.path.join(package_path, 'config', 'Competition_track_graph.graphml')
        default_key_nodes_path = os.path.join(package_path, 'config', 'global_path_key_nodes.yaml')

        self.graphml_file = rospy.get_param('~graphml_file', default_graphml_path)
        self.key_nodes_file = rospy.get_param('~key_nodes_file', default_key_nodes_path)
        self.target_spacing = rospy.get_param('~target_spacing', 0.1)  # 목표 간격 (m)

        rospy.loginfo(f"Reading GraphML file from: {self.graphml_file}")
        rospy.loginfo(f"Reading key_nodes from: {self.key_nodes_file}")

        # Publisher for nav_msgs/Path
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
        self.dotted_path_pub = rospy.Publisher('/global_path_dotted', PathWithDotted, queue_size=1, latch=True)

        # Load GraphML
        self.graph = self.load_graphml_file(self.graphml_file)
        self.key_nodes = self.load_key_nodes_file(self.key_nodes_file)

        # Create path from checkpoints
        full_node_list = self.build_path_via_checkpoints(self.graph, self.key_nodes)

        # Resample the path for smoother trajectory
        resampled_path = self.resample_path(self.graph, full_node_list, self.target_spacing)

        # Generate messages
        self.global_path_msg = self.build_path_msg_from_nodes(resampled_path)
        self.dotted_path_msg = self.build_dotted_path_msg(resampled_path)

        publish_rate = rospy.get_param('~publish_rate', 1.0)
        self.rate = rospy.Rate(publish_rate)

    def load_graphml_file(self, file_path):
        try:
            g = nx.read_graphml(file_path)
            rospy.loginfo("Successfully loaded GraphML.")
            return g
        except Exception as e:
            rospy.logerr(f"Failed to load GraphML file: {e}")
            return None

    def load_key_nodes_file(self, file_path):
        if not os.path.exists(file_path):
            rospy.logwarn(f"Key nodes file {file_path} not found. Using an empty list.")
            return []
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
            key_nodes = data.get('key_nodes', [])
            rospy.loginfo(f"Loaded {len(key_nodes)} key nodes from {file_path}.")
            return key_nodes
        except Exception as e:
            rospy.logerr(f"Failed to load key nodes file: {e}")
            return []

    def build_path_via_checkpoints(self, graph, key_nodes):
        if graph is None:
            rospy.logerr("Graph is None. Cannot build path.")
            return []

        if len(key_nodes) < 2:
            return key_nodes

        full_list = []
        for i in range(len(key_nodes) - 1):
            start_n = str(key_nodes[i])
            end_n   = str(key_nodes[i+1])

            if start_n not in graph.nodes():
                rospy.logwarn(f"Key node {start_n} not in graph. Skipping.")
                continue
            if end_n not in graph.nodes():
                rospy.logwarn(f"Key node {end_n} not in graph. Skipping.")
                continue

            try:
                sub_path = nx.shortest_path(graph, source=start_n, target=end_n)
            except nx.NetworkXNoPath:
                rospy.logwarn(f"No path found between {start_n} and {end_n}. Skipping.")
                continue

            if i == 0:
                full_list.extend(sub_path)
            else:
                full_list.extend(sub_path[1:])

        return full_list

    def resample_path(self, graph, node_list, spacing):
        """
        Resample the path with equal spacing using interpolation.
        """
        if graph is None or len(node_list) < 2:
            return []

        path_array = np.array([(float(graph.nodes[n]['x']), float(graph.nodes[n]['y'])) for n in node_list])
        distances = np.cumsum(np.sqrt(np.sum(np.diff(path_array, axis=0) ** 2, axis=1)))
        distances = np.insert(distances, 0, 0)  # 첫 번째 점까지 거리 추가

        interpolators = [interp1d(distances, path_array[:, i], kind='linear') for i in range(2)]
        new_distances = np.arange(0, distances[-1], spacing)

        resampled_path = np.vstack([interpolators[i](new_distances) for i in range(2)]).T
        return resampled_path.tolist()

    def build_path_msg_from_nodes(self, path_array):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for i, (x, y) in enumerate(path_array):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0

            if i > 0:
                dx = x - path_array[i-1][0]
                dy = y - path_array[i-1][1]
                yaw = math.atan2(dy, dx)
                ps.pose.orientation.z = math.sin(yaw / 2)
                ps.pose.orientation.w = math.cos(yaw / 2)

            path_msg.poses.append(ps)

        return path_msg

    def build_dotted_path_msg(self, path_array):
        dotted_path = PathWithDotted()
        dotted_path.header.frame_id = "map"
        dotted_path.header.stamp = rospy.Time.now()

        for x, y in path_array:
            dp = DottedPose()
            dp.pose.header.frame_id = "map"
            dp.pose.header.stamp = rospy.Time.now()
            dp.pose.pose.position.x = x
            dp.pose.pose.position.y = y
            dp.pose.pose.orientation.w = 1.0
            dp.dotted = False  # 수정 가능

            dotted_path.poses.append(dp)

        return dotted_path

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if self.global_path_msg:
                self.global_path_msg.header.stamp = now
                self.path_pub.publish(self.global_path_msg)

            if self.dotted_path_msg:
                self.dotted_path_msg.header.stamp = now
                self.dotted_path_pub.publish(self.dotted_path_msg)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        ntp = NodeToPath()
        ntp.run()
    except rospy.ROSInterruptException:
        pass
