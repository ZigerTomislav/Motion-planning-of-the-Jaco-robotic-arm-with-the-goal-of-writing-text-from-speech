#!/usr/bin/env python3

import rospy
import numpy as np
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import time
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


class RobotDrawer:
    Z_OFFSET = -0.05
    
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_arm = moveit_commander.MoveGroupCommander("arm")
        self.visualizer = None  
    
    def set_visualizer(self, visualizer):
        self.visualizer = visualizer
    
    def update_visualization(self, line_points, planning_frame):
        if self.visualizer:
            current_position = self.group_arm.get_current_pose().pose.position
            point = geometry_msgs.msg.Point()
            point.x = current_position.x
            point.y = current_position.y
            point.z = current_position.z + self.Z_OFFSET
            line_points.append(point)

            self.visualizer.publish_point_marker(
                current_position.x,
                current_position.y,
                current_position.z + self.Z_OFFSET,
                planning_frame
            )

            self.visualizer.publish_line_marker(line_points, planning_frame)
    
    def visualize_complete_path(self, x_scaled, y_scaled):
        if self.visualizer:
            complete_path_points = []
            for x, y in zip(x_scaled, y_scaled):
                point = geometry_msgs.msg.Point()
                point.x = x
                point.y = y
                point.z = 0.1
                complete_path_points.append(point)

            self.visualizer.publish_line_marker(
                complete_path_points,
                self.group_arm.get_planning_frame(),
                color=(1, 0, 0)
            )
    
    def attach_cylinder_to_gripper(self, cylinder_id="attached_cylinder", 
                                  height=0.08, radius=0.02, pose_offset=(0, 0, 0.00)):
        attached_object = moveit_msgs.msg.AttachedCollisionObject()
        attached_object.link_name = self.group_arm.get_end_effector_link()
        attached_object.object.header.frame_id = self.group_arm.get_end_effector_link()
        attached_object.object.id = cylinder_id
        
        cylinder = SolidPrimitive()
        cylinder.type = cylinder.CYLINDER
        cylinder.dimensions = [height, radius]
        
        cylinder_pose = geometry_msgs.msg.Pose()
        cylinder_pose.position.x = pose_offset[0]
        cylinder_pose.position.y = pose_offset[1]
        cylinder_pose.position.z = pose_offset[2]
        cylinder_pose.orientation.x = 1.0
        cylinder_pose.orientation.w = 1.0
        
        attached_object.object.primitives = [cylinder]
        attached_object.object.primitive_poses = [cylinder_pose]
        attached_object.object.operation = attached_object.object.ADD
        
        attached_object.touch_links = [self.group_arm.get_end_effector_link()]
        
        self.scene.attach_object(attached_object)
        
        rospy.sleep(0.5)
        
        print(f"Pen atached to gripper ")
        return attached_object
    
    def detach_cylinder_from_gripper(self, cylinder_id="attached_cylinder"):
        self.scene.remove_attached_object(name=cylinder_id)
        rospy.sleep(0.5)
        print(f"detached from gripper")
    
    def clear_all_attached_objects(self):
        attached_objects = self.scene.get_attached_objects()
        
        for object_name in attached_objects.keys():
            self.scene.remove_attached_object(name=object_name)
        
        known_objects_names = self.scene.get_known_object_names()
        for object_name in known_objects_names:
            self.scene.remove_world_object(name=object_name)
        
        rospy.sleep(1.0)
        print("All objects cleared")
    
    def go_to_home(self):
        self.group_arm.set_named_target("Home")
        success = self.group_arm.go(wait=True)
        print(f"Moving to Home, Success: {success}")
        self.group_arm.clear_pose_targets()
        return success
    
    def move_up(self, distance=0.1):
        current_pose = self.group_arm.get_current_pose().pose
        current_pose.position.z = current_pose.position.z + distance
        self.group_arm.set_pose_target(current_pose)
        success = self.group_arm.go(wait=True)
        print(f"Moving Up by {distance}, Success: {success}")
        self.group_arm.clear_pose_targets()
        return success
    
    def move_to_pose(self, pose):
        self.group_arm.set_pose_target(pose)
        success = self.group_arm.go(wait=True)
        print(f"Moving to pose, Success: {success}")
        self.group_arm.clear_pose_targets()
        return success
    
    def move_along_path(self, path):
        current_pose = self.group_arm.get_current_pose().pose

        x_scaled = path[:, 0]
        y_scaled = path[:, 1]

        start_pose = Pose()
        start_pose.position.x = x_scaled[0]
        start_pose.position.y = y_scaled[0]
        start_pose.position.z = 0.2
        start_pose.orientation = current_pose.orientation
        self.move_to_pose(start_pose)

        print("Starting the path")

        self.visualize_complete_path(x_scaled, y_scaled)

        planning_frame = self.group_arm.get_planning_frame()
        line_points = []
        
        point = geometry_msgs.msg.Point()
        point.x = start_pose.position.x
        point.y = start_pose.position.y
        point.z = start_pose.position.z + self.Z_OFFSET
        line_points.append(point)

        waypoints = []
        for x, y in zip(x_scaled, y_scaled):
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = start_pose.position.z
            p.orientation = start_pose.orientation
            waypoints.append(p)

        self.update_visualization(line_points, planning_frame)

        for i in range(len(waypoints)):
            wp = waypoints[i: i + 1]
            self.group_arm.allow_replanning(True)
            self.group_arm.set_planning_time(5.0)
            self.group_arm.set_num_planning_attempts(5)
            
            plan, fraction = self.group_arm.compute_cartesian_path(wp, 0.01)
            
            self.group_arm.execute(plan, wait=True)

            self.update_visualization(line_points, planning_frame)

            if fraction != 1.0:
                print(f"Path planning: {fraction:.2%} computed.")
                print("Problem with path planning")
    
    def get_current_pose(self):
        return self.group_arm.get_current_pose().pose
    
    def shutdown(self):
        self.group_arm.stop()
        self.group_arm.clear_pose_targets()
        moveit_commander.roscpp_shutdown() 