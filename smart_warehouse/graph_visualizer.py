#!/usr/bin/env python3
"""
Graph Visualizer Node for Nav2 Route Server.

Reads a GeoJSON graph file and publishes MarkerArray to RViz2 for visualization.
- Blue spheres: Nodes (waypoints)
- Green lines: Edges (connections)
"""

import json
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class GraphVisualizer(Node):
    def __init__(self):
        super().__init__('graph_visualizer')
        
        # Parameters
        self.declare_parameter('graph_file', '')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_rate', 1.0)
        
        graph_file = self.get_parameter('graph_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # If no graph file specified, use default
        if not graph_file:
            pkg_dir = get_package_share_directory('smart_warehouse')
            graph_file = os.path.join(pkg_dir, 'graphs', 'warehouse_graph.geojson')
        
        self.get_logger().info(f'Loading graph from: {graph_file}')
        
        # Publisher with QoS profile for latching (Transient Local)
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/route_graph_markers', qos_profile)
        
        # Load graph
        self.nodes = {}
        self.edges = []
        self.load_graph(graph_file)
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_markers)
        
        self.get_logger().info(f'Graph loaded: {len(self.nodes)} nodes, {len(self.edges)} edges')

    def load_graph(self, filepath: str):
        """Load GeoJSON graph file."""
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            for feature in data.get('features', []):
                feature_id = feature.get('id', '')
                geom_type = feature['geometry']['type']
                coords = feature['geometry']['coordinates']
                props = feature.get('properties', {})
                
                if geom_type == 'Point':
                    # This is a node
                    self.nodes[feature_id] = {
                        'x': coords[0],
                        'y': coords[1],
                        'z': 0.0,
                        'name': props.get('name', feature_id),
                        'description': props.get('description', '')
                    }
                elif geom_type == 'LineString':
                    # This is an edge
                    self.edges.append({
                        'id': feature_id,
                        'from': props.get('from', ''),
                        'to': props.get('to', ''),
                        'points': coords,
                        'weight': props.get('weight', 1.0),
                        'direction': props.get('direction', 'bidirectional')
                    })
                    
        except Exception as e:
            self.get_logger().error(f'Failed to load graph: {e}')

    def publish_markers(self):
        """Publish MarkerArray with nodes and edges."""
        marker_array = MarkerArray()
        marker_id = 0
        
        # === NODE MARKERS (Blue Spheres) ===
        for node_id, node_data in self.nodes.items():
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'nodes'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = float(node_data['x'])
            marker.pose.position.y = float(node_data['y'])
            marker.pose.position.z = float(node_data['z'])
            marker.pose.orientation.w = 1.0
            
            # Scale (sphere diameter)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            # Color (Blue)
            marker.color = ColorRGBA(r=0.0, g=0.3, b=1.0, a=1.0)
            
            # Lifetime (0 = forever)
            marker.lifetime.sec = 0
            
            marker_array.markers.append(marker)
            marker_id += 1
            
            # === NODE LABEL (Text) ===
            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'node_labels'
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(node_data['x'])
            text_marker.pose.position.y = float(node_data['y'])
            text_marker.pose.position.z = float(node_data['z']) + 0.7
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.4  # Text height
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = node_data['name']
            
            marker_array.markers.append(text_marker)
            marker_id += 1
        
        # === EDGE MARKERS (Green Lines) ===
        for edge in self.edges:
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'edges'
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Points
            for coord in edge['points']:
                p = Point()
                p.x = float(coord[0])
                p.y = float(coord[1])
                p.z = 0.0
                marker.points.append(p)
            
            # Scale (line width)
            marker.scale.x = 0.1
            
            # Color (Green for bidirectional, Yellow for unidirectional)
            if edge['direction'] == 'bidirectional':
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.3, a=0.8)
            else:
                marker.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8)
            
            marker.lifetime.sec = 0
            
            marker_array.markers.append(marker)
            marker_id += 1
            
            # === EDGE WEIGHT LABEL ===
            if len(edge['points']) >= 2:
                # Place label at midpoint
                mid_x = (edge['points'][0][0] + edge['points'][-1][0]) / 2
                mid_y = (edge['points'][0][1] + edge['points'][-1][1]) / 2
                
                weight_marker = Marker()
                weight_marker.header.frame_id = self.frame_id
                weight_marker.header.stamp = self.get_clock().now().to_msg()
                weight_marker.ns = 'edge_weights'
                weight_marker.id = marker_id
                weight_marker.type = Marker.TEXT_VIEW_FACING
                weight_marker.action = Marker.ADD
                
                weight_marker.pose.position.x = float(mid_x)
                weight_marker.pose.position.y = float(mid_y)
                weight_marker.pose.position.z = 0.3
                weight_marker.pose.orientation.w = 1.0
                
                weight_marker.scale.z = 0.25
                weight_marker.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=0.9)
                weight_marker.text = f"w={edge['weight']}"
                
                marker_array.markers.append(weight_marker)
                marker_id += 1
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = GraphVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
