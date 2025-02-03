#!/usr/bin/env python3

import rospy
import rospkg

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

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

        # Load the GraphML file
        self.graph = self.load_graphml_file(self.graphml_file)
        # Load the list of checkpoints
        self.key_nodes = self.load_key_nodes_file(self.key_nodes_file)

        # Concatenate the paths between (start -> checkpoint -> ... -> end)
        full_node_list = self.build_path_via_checkpoints(self.graph, self.key_nodes)

        # Convert the node list into a nav_msgs/Path
        self.global_path_msg = self.build_path_msg_from_nodes(self.graph, full_node_list)

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

            # If start_n or end_n is not in the graph
            if start_n not in graph.nodes():
                rospy.logwarn(f"Key node {start_n} not in graph. Skipping.")
                continue
            if end_n not in graph.nodes():
                rospy.logwarn(f"Key node {end_n} not in graph. Skipping.")
                continue

            try:
                # networkx.shortest_path -> BFS-based shortest path in an unweighted graph
                sub_path = nx.shortest_path(graph, source=start_n, target=end_n)
            except nx.NetworkXNoPath:
                rospy.logwarn(f"No path found between {start_n} and {end_n}. Skipping.")
                continue

            # Example sub_path: ['263', '264', '265', '59']
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
            # Extract x,y from GraphML (change keys if necessary)
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

    def run(self):
        while not rospy.is_shutdown():
            if self.global_path_msg:
                current_time = rospy.Time.now()
                self.global_path_msg.header.stamp = current_time
                for pose in self.global_path_msg.poses:
                    pose.header.stamp = current_time
                self.path_pub.publish(self.global_path_msg)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = GraphMLToPathNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
