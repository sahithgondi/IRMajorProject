#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import heapq
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool

# --- 1. TOPOLOGICAL MAP ---
# Replace with your Lab Coordinates
NODES = {
    "A": {"x": 0.295, "y": 0.200}, 
    "B": {"x": 4.620, "y": 0.200},
    "C": {"x": 4.620, "y": 1.234},
    "D": {"x": 0.295, "y": 1.234}
}

# Full connectivity (Square Loop)
# Format: Start: [(Neighbor, Distance)]
GRAPH = {
    "A": [("B", 4.8), ("D", 1.5)],
    "B": [("A", 4.8), ("C", 1.5)],
    "C": [("B", 1.5), ("D", 4.8)],
    "D": [("A", 1.5), ("C", 4.8)]
}

class DeliberativeLayer:
    def __init__(self):
        rospy.init_node('deliberative_layer')
        
        self.lcs_pub = rospy.Publisher('/current_lcs', String, queue_size=1)
        self.goal_pub = rospy.Publisher('/current_goal', Point, queue_size=1)
        
        rospy.Subscriber('/robot_stuck', Bool, self.stuck_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_cb)

        self.current_pose = None
        self.path_queue = []        # The current list of nodes to visit
        self.blocked_edges = []     # List of blocked tuples [('A','B'), ('B','A')]
        
        # State Tracking
        self.last_visited_node = "A" # Assume start at A
        self.current_target_node = None
        self.global_goal_node = None # Where we ultimately want to end up
        self.mission_state = "IDLE" 

        rospy.sleep(1.0) 

    def pose_cb(self, msg):
        self.current_pose = msg.pose.pose.position

    # --- 2. DYNAMIC RE-PLANNING ---
    def stuck_callback(self, msg):
        """
        Triggered when Reactive Layer says 'I can't move forward'.
        Handles blocking the edge and BACKTRACKING to the last safe node.
        """
        if msg.data and self.mission_state == "EXECUTING":
            rospy.logwarn("OBSTACLE DETECTED! Marking path blocked and Re-planning...")

            # 1. Stop the robot immediately
            self.lcs_pub.publish("STOP")

            # 2. Identify the Blocked Link
            # We are stuck between 'last_visited' (e.g., A) and 'current_target' (e.g., B)
            if self.last_visited_node and self.current_target_node:
                u, v = self.last_visited_node, self.current_target_node

                # Mark both directions as blocked
                if (u, v) not in self.blocked_edges:
                    self.blocked_edges.append((u, v))
                if (v, u) not in self.blocked_edges:
                    self.blocked_edges.append((v, u))

                # FIXED: Python 2 compatible formatting
                rospy.loginfo("Graph Update: Edge {}<->{} is now BLOCKED.".format(u, v))

                # 3. Re-Calculate Path from Last Known Good Node (A) to Global Goal
                # We do NOT use plan_and_execute here because we need strict control over the queue
                rospy.sleep(1.0) 
                
                # Calculate path: e.g., returns ['A', 'D', 'C']
                new_path = self.get_astar_path(self.last_visited_node, self.global_goal_node)

                if new_path:
                    # Check if the start node is missing. If so, put it back.
                    if new_path[0] != self.last_visited_node:
                         new_path.insert(0, self.last_visited_node)

                    # 4. EXPLICIT BACKTRACKING LOGIC
                    # We set the queue to the new path. 
                    # The first item is 'A' (self.last_visited_node).
                    self.path_queue = new_path
                    
                    # CRITICAL: We set current_target to None. 
                    # This forces the run() loop to pop the first item ('A') off the queue immediately.
                    # This makes the robot physically drive BACK to 'A' before going to 'D'.
                    self.current_target_node = None
                    
                    rospy.loginfo("Backtracking: returning to safe node {} before proceeding.".format(new_path[0]))
                    rospy.loginfo("New Path: {}".format(new_path))
                    
                    # Resume execution
                    self.lcs_pub.publish("GO_TO_POINT")
                    
                else:
                    rospy.logerr("CRITICAL: No path found after blocking edge! Robot is stranded.")
                    self.mission_state = "COMPLETED" # Or some error state

    # --- 3. A* ALGORITHM (With Blockage Check) ---
    def get_astar_path(self, start, goal):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {start: 0}
        
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal: break
            
            for next_node, dist in GRAPH[current]:
                # DYNAMIC CHECK: Is this specific edge blocked?
                if (current, next_node) in self.blocked_edges:
                    dist = float('inf') # Infinite cost
                
                new_cost = cost_so_far[current] + dist
                
                if new_cost < float('inf'): # Only process if traversable
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + self.heuristic(next_node, goal)
                        heapq.heappush(frontier, (priority, next_node))
                        came_from[next_node] = current
                    
        return self.reconstruct_path(came_from, start, goal)

    def heuristic(self, a, b):
        # Euclidean distance heuristic
        return math.sqrt((NODES[a]["x"] - NODES[b]["x"])**2 + (NODES[a]["y"] - NODES[b]["y"])**2)

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        if goal not in came_from: 
            return [] # No path possible
        while current != start:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def plan_and_execute(self, start, goal):
        self.global_goal_node = goal
        # FIXED: Python 2 formatting
        rospy.loginfo("Calculating path from {} to {}...".format(start, goal))
        
        new_path = self.get_astar_path(start, goal)
        
        if not new_path:
            rospy.logerr("CRITICAL: No valid path found! All routes blocked.")
            self.lcs_pub.publish("STOP")
            self.mission_state = "FAILED"
            return

        # FIXED: Python 2 formatting
        rospy.loginfo("New Plan: {}".format(new_path))
        self.path_queue = new_path
        self.mission_state = "EXECUTING"

    # --- 4. EXECUTION LOOP ---
    def run(self):
        # WAIT for system ready
        rospy.sleep(2)
        
        # --- MISSION START: Go from A to C ---
        self.last_visited_node = "A"
        self.plan_and_execute("A", "B")
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.mission_state == "EXECUTING" and self.path_queue:
                # Peek at next target
                target_id = self.path_queue[0]
                self.current_target_node = target_id
                
                # 1. Command Reactive Layer
                self.lcs_pub.publish("GO_TO_POINT")
                
                # 2. Send Target Coordinates
                p = Point()
                p.x = NODES[target_id]["x"]
                p.y = NODES[target_id]["y"]
                self.goal_pub.publish(p)
                
                # 3. Check for Arrival
                if self.current_pose:
                    dist = math.sqrt((self.current_pose.x - p.x)**2 + (self.current_pose.y - p.y)**2)
                    if dist < 0.35: # 0.2m tolerance
                        # FIXED: Python 2 formatting
                        rospy.loginfo("Reached Node: {}".format(target_id))
                        
                        # Update state
                        self.last_visited_node = target_id 
                        self.path_queue.pop(0)
                        
                        if not self.path_queue:
                            rospy.loginfo("Mission Complete! Global Goal Reached.")
                            self.lcs_pub.publish("STOP")
                            self.mission_state = "COMPLETED"
            
            elif self.mission_state == "EXECUTING" and not self.path_queue:
                 self.lcs_pub.publish("STOP") # Just in case

            rate.sleep()

if __name__ == '__main__':
    DeliberativeLayer().run()
