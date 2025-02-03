#!/usr/bin/env python3

import rospy
import rospkg

from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from node_to_path.msg import PathWithDotted, DottedPose

import networkx as nx
import os
import yaml

class GraphMLToPathNode:
    def __init__(self):
        rospy.init_node('graphml_to_path_node', anonymous=True)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('node_to_path')

        default_graphml_path = os.path.join(package_path, 'config', 'Competition_track_graph.graphml')
        default_key_nodes_path = os.path.join(package_path, 'config', 'global_path_key_nodes.yaml')

        # Parameter or default
        self.graphml_file = rospy.get_param('~graphml_file', default_graphml_path)
        self.key_nodes_file = rospy.get_param('~key_nodes_file', default_key_nodes_path)

        rospy.loginfo(f"Reading GraphML file from: {self.graphml_file}")
        rospy.loginfo(f"Reading key_nodes from: {self.key_nodes_file}")

        # Publisher for nav_msgs/Path
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        # Publisher for the custom PathWithDotted
        self.dotted_path_pub = rospy.Publisher('/global_path_dotted', PathWithDotted, queue_size=1)

        # Load the GraphML file
        self.graph = self.load_graphml_file(self.graphml_file)
        # Load the list of checkpoints
        self.key_nodes = self.load_key_nodes_file(self.key_nodes_file)

        # Concatenate the paths between (start -> checkpoint -> ... -> end)
        full_node_list = self.build_path_via_checkpoints(self.graph, self.key_nodes)

        # Convert the node list into a nav_msgs/Path
        self.global_path_msg = self.build_path_msg_from_nodes(self.graph, full_node_list)
        # Convert the node list into a custom PathWithDotted
        self.dotted_path_msg = self.build_dotted_path_msg(self.graph, full_node_list)

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
        """
        Example:
        key_nodes = [263, 59, 242, 313]
        1) Shortest path 263 -> 59
        2) Shortest path 59 -> 242
        3) Shortest path 242 -> 313
        Concatenate them to form a complete node list.
        """
        if graph is None:
            rospy.logerr("Graph is None. Cannot build path.")
            return []

        if len(key_nodes) < 2:
            return key_nodes  # If only one or zero key nodes exist, return as is

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
                # Remove the duplicated boundary node
                full_list.extend(sub_path[1:])

        return full_list

    def build_path_msg_from_nodes(self, graph, node_list):
        """
        Ex: node_list = ['263','264','265','59','242', ... ,'313']
        Convert this into a nav_msgs/Path message.
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        if graph is None:
            rospy.logerr("Graph is None. Unable to build path message.")
            return path_msg

        for node_id in node_list:
            if node_id not in graph.nodes:
                rospy.logwarn(f"Node {node_id} not found in graph. Skipping.")
                continue

            node_data = graph.nodes[node_id]
            x = float(node_data.get('x', 0.0))
            y = float(node_data.get('y', 0.0))

            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0

            path_msg.poses.append(ps)

        return path_msg

    def build_dotted_path_msg(self, graph, node_list):
        """
        Convert node_list -> PathWithDotted.
        """
        dotted_path = PathWithDotted()
        dotted_path.header.frame_id = "map"
        dotted_path.header.stamp = rospy.Time.now()

        if graph is None:
            rospy.logerr("Graph is None. Unable to build dotted path message.")
            return dotted_path

        for i, node_id in enumerate(node_list):
            node_str = str(node_id)
            if node_str not in graph.nodes:
                rospy.logwarn(f"Node {node_id} not found in graph. Skipping.")
                continue

            node_data = graph.nodes[node_str]
            x = float(node_data.get('x', 0.0))
            y = float(node_data.get('y', 0.0))

            dp = DottedPose()
            dp.pose.header.frame_id = "map"
            dp.pose.header.stamp = rospy.Time.now()
            dp.pose.pose.position.x = x
            dp.pose.pose.position.y = y
            dp.pose.pose.orientation.w = 1.0

            if i > 0:
                prev_node = node_list[i-1]
                if graph.has_edge(str(prev_node), node_str):
                    edge_data = graph.get_edge_data(str(prev_node), node_str)
                    if 'dotted' in edge_data and str(edge_data['dotted']).lower() == 'true':
                        dp.dotted = True
                    else:
                        dp.dotted = False
                else:
                    dp.dotted = False
            else:
                # First node, no incoming edge
                dp.dotted = False

            dotted_path.poses.append(dp)

        return dotted_path

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # Update standard Path header
            if self.global_path_msg:
                self.global_path_msg.header.stamp = now
                for pose in self.global_path_msg.poses:
                    pose.header.stamp = now

                self.path_pub.publish(self.global_path_msg)

            # Update PathWithDotted header
            if self.dotted_path_msg:
                self.dotted_path_msg.header.stamp = now
                for dp in self.dotted_path_msg.poses:
                    dp.pose.header.stamp = now

                self.dotted_path_pub.publish(self.dotted_path_msg)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = GraphMLToPathNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
