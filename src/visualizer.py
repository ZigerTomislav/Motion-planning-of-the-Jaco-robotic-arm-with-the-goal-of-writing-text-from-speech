#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import visualization_msgs.msg


class Visualizer:
    def __init__(self):
        self.marker_publisher = None
        self.point_marker_id = 0
        self.line_marker_id = 1000
        self.create_marker_publisher()
    
    def create_marker_publisher(self):
        self.marker_publisher = rospy.Publisher(
            '/visualization_marker', 
            visualization_msgs.msg.Marker, 
            queue_size=100
        )
        rospy.sleep(0.5)
    
    def create_point_marker(self, x, y, z, frame_id, marker_id=None, color=(0, 1, 0)):
        if marker_id is None:
            marker_id = self.point_marker_id
            self.point_marker_id += 1

        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_trajectory"
        marker.id = marker_id
        marker.type = visualization_msgs.msg.Marker.SPHERE
        marker.action = visualization_msgs.msg.Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.005
        marker.scale.y = 0.005
        marker.scale.z = 0.005

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(0)

        return marker
    
    def create_line_marker(self, points, frame_id, marker_id=None, color=(0, 1, 0)):
        if marker_id is None:
            marker_id = self.line_marker_id
            self.line_marker_id += 1

        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_trajectory"
        marker.id = marker_id
        marker.type = visualization_msgs.msg.Marker.LINE_STRIP
        marker.action = visualization_msgs.msg.Marker.ADD

        marker.scale.x = 0.001

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        marker.points = points
        marker.lifetime = rospy.Duration(0)

        return marker
    
    def publish_point_marker(self, x, y, z, frame_id, marker_id=None, color=(0, 1, 0)):
        marker = self.create_point_marker(x, y, z, frame_id, marker_id, color)
        self.marker_publisher.publish(marker)
    
    def publish_line_marker(self, points, frame_id, marker_id=None, color=(0, 1, 0)):
        marker = self.create_line_marker(points, frame_id, marker_id, color)
        self.marker_publisher.publish(marker)
    
    def clear_markers(self):
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_trajectory"
        marker.action = visualization_msgs.msg.Marker.DELETEALL
        self.marker_publisher.publish(marker) 