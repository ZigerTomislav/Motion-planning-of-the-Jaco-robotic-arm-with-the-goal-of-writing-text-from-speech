#!/usr/bin/env python3

import numpy as np
from HersheyFonts import HersheyFonts


class TextToWaypoints:
    def __init__(self):
        self.font = HersheyFonts()
        self.font.load_default_font()
        self.font.normalize_rendering(1)
    
    def remove_duplicates(self, waypoints):
        result = []
        for lista in waypoints:
            curr_list = []
            last = (0, 0)

            for i in lista:
                if last == i:
                    continue

                curr_list.append(i)
                last = i

            result.append(curr_list)

        return result
    
    def get_waypoints_hershey(self, text):
        lines = self.font.lines_for_text(text)

        mem_x = 0
        mem_y = 0

        pairs = []
        waypoints = []
        
        for (x1, y1), (x2, y2) in lines:
            if mem_x != x1 or mem_y != y1:
                waypoints.append(pairs.copy())
                pairs = []

            pairs.append((x1, y1))
            pairs.append((x2, y2))
            mem_x = x2
            mem_y = y2

        waypoints.append(pairs)

        return waypoints
    
    def transform_points_lists(self, points_list, current_pose, span_x=0.2, span_y=0.1):
        all_points = np.concatenate(points_list, axis=0)

        min_x, max_x = np.min(all_points[:, 0]), np.max(all_points[:, 0])
        min_y, max_y = np.min(all_points[:, 1]), np.max(all_points[:, 1])

        x0, y0 = current_pose.position.x - 0.2, current_pose.position.y - 0.5
        x0 = 0

        result_list = []
        for pts in points_list:
            scaled_x = (pts[:, 0] - min_x) / (max_x - min_x) * span_x + x0 + 0.2
            scaled_y = (pts[:, 1] - min_y) / (max_y - min_y) * span_y + y0 + 0.2
            result_list.append(np.vstack([scaled_x, scaled_y]).T)

        return result_list
    
    def text_to_robot_waypoints(self, text, current_pose, span_x=0.2, span_y=0.1):
        waypoints = self.get_waypoints_hershey(text.strip())
        waypoints = self.remove_duplicates(waypoints)

        points_np = []
        for waypoint_list in waypoints:
            if waypoint_list:
                points_np.append(np.array(waypoint_list))
                
        scaled_points = self.transform_points_lists(points_np, current_pose, span_x, span_y)
        
        return scaled_points 