#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander

from audio_processor import AudioProcessor
from visualizer import Visualizer
from text_to_waypoints import TextToWaypoints
from robot_drawer import RobotDrawer


class RobotDrawing:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_drawing_app', anonymous=True)
        
        self.audio_processor = AudioProcessor()
        self.visualizer = Visualizer()
        self.text_to_waypoints = TextToWaypoints()
        self.robot_drawer = RobotDrawer()
        self.robot_drawer.set_visualizer(self.visualizer)
            
    def draw(self, text=None): 
        try:
            self.visualizer.clear_markers()
            self.robot_drawer.clear_all_attached_objects()
            
            self.robot_drawer.attach_cylinder_to_gripper(
                cylinder_id="drawing_pen", 
                height=0.15, 
                radius=0.005, 
                pose_offset=(0, 0, 0.0)
            )
            self.robot_drawer.go_to_home()
            
            if text is None:
                text = self.audio_processor.record_and_transcribe()
            
            current_pose = self.robot_drawer.get_current_pose()
            scaled_points = self.text_to_waypoints.text_to_robot_waypoints(text, current_pose)
            
            if not scaled_points:
                return False
            
            for path in scaled_points:
                self.robot_drawer.move_along_path(path)
                self.robot_drawer.move_up()
            
            #self.robot_drawer.detach_cylinder_from_gripper("drawing_pen")
            self.robot_drawer.go_to_home()
            return True
            
        except Exception as e:
            print(f"Error {e}")
            return False


def main():
    try:
        app = RobotDrawing()
        start_time = time.time()
        #if testing setup text to be desired word
        text = None
        success = app.draw(text)
        end_time = time.time()
        total_time = end_time - start_time
        print(f"Program completed: {success}")
        print(f"Total time: {total_time:.2f} seconds")
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        sys.exit(0)


if __name__ == "__main__":
    main() 