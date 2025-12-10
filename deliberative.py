#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import heapq
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool

# the points for our topological map
NODES = {
    "A": {"x": 0.295, "y": 0.200}, 
    "B": {"x": 4.620, "y": 0.200},
    "C": {"x": 4.620, "y": 1.234},
    "D": {"x": 0.295, "y": 1.234}
}

# points in the format (Neighbor, Distance)
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
        # list of nodes to visit and blocked edges
        self.path_queue = []       
        self.blocked_edges = []     
        
        # track our current state and goal assume we start with node A
        self.last_visited_node = "A" 
        self.current_target_node = None
        self.global_goal_node = None 
        self.mission_state = "IDLE" 

        rospy.sleep(1.0) 

    def pose_cb(self, msg):
        self.current_pose = msg.pose.pose.position

    # replanning and backtracks to last safe spot
    def stuck_callback(self, msg):
        if msg.data and self.mission_state == "EXECUTING":
            rospy.logwarn("OBSTACLE DETECTED Re-planning...")

            # stop robot
            self.lcs_pub.publish("STOP")

            # identify where we are blocked
            if self.last_visited_node and self.current_target_node:
                u, v = self.last_visited_node, self.current_target_node

                # set both as blocked
                if (u, v) not in self.blocked_edges:
                    self.blocked_edges.append((u, v))
                if (v, u) not in self.blocked_edges:
                    self.blocked_edges.append((v, u))

                # log blocked
                rospy.loginfo("Update: Edge {}<->{} is now Blocked!!".format(u, v))
                rospy.sleep(1.0) 
                
                # recalculate path
                new_path = self.get_astar_path(self.last_visited_node, self.global_goal_node)

                if new_path:
                    # if start node not added add back
                    if new_path[0] != self.last_visited_node:
                         new_path.insert(0, self.last_visited_node)

                    # set queue to the new path
                    self.path_queue = new_path
                    
                    # remove a target so we go back to safe node
                    self.current_target_node = None
                    
                    rospy.loginfo("Backtracking and returning to safe node {} before continuing.".format(new_path[0]))
                    rospy.loginfo("New Path: {}".format(new_path))
                    
                    # resume 
                    self.lcs_pub.publish("GO_TO_POINT")
                    
                else:
                    rospy.logerr("!!! no path found after blocked edge")
                    self.mission_state = "COMPLETED" # Or some error state

    # A* implementation
    def get_astar_path(self, start, goal):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {start: 0}
        
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal: break
            
            for next_node, dist in GRAPH[current]:
                # check if edge is blocked
                if (current, next_node) in self.blocked_edges:
                    # set distance to infinity
                    dist = float('inf')
                
                new_cost = cost_so_far[current] + dist
                # only continue if finite distance
                if new_cost < float('inf'): 
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + self.heuristic(next_node, goal)
                        heapq.heappush(frontier, (priority, next_node))
                        came_from[next_node] = current
                    
        return self.reconstruct_path(came_from, start, goal)

    def heuristic(self, a, b):
        # use eucledian distance
        return math.sqrt((NODES[a]["x"] - NODES[b]["x"])**2 + (NODES[a]["y"] - NODES[b]["y"])**2)

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        # no possible path
        if goal not in came_from: 
            return [] 
        # otherwise add to path
        while current != start:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    # plan path and execute
    def plan_and_execute(self, start, goal):
        self.global_goal_node = goal
        rospy.loginfo("Calculating path from {} to {}...".format(start, goal))
        
        new_path = self.get_astar_path(start, goal)
        
        if not new_path:
            rospy.logerr("!!No valid path found, all routes blocked.")
            self.lcs_pub.publish("STOP")
            self.mission_state = "FAILED"
            return

        rospy.loginfo("New Plan: {}".format(new_path))
        self.path_queue = new_path
        self.mission_state = "EXECUTING"

    # run function
    def run(self):
        rospy.sleep(2)
        
        # start at node A
        self.last_visited_node = "A"
        self.plan_and_execute("A", "B")
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.mission_state == "EXECUTING" and self.path_queue:
                # check next target
                target_id = self.path_queue[0]
                self.current_target_node = target_id
                
                # tell reactive layer to go
                self.lcs_pub.publish("GO_TO_POINT")
                
                # send coordinates
                p = Point()
                p.x = NODES[target_id]["x"]
                p.y = NODES[target_id]["y"]
                self.goal_pub.publish(p)
                
                # check when arrived
                if self.current_pose:
                    dist = math.sqrt((self.current_pose.x - p.x)**2 + (self.current_pose.y - p.y)**2)
                    if dist < 0.35:
                        rospy.loginfo("Reached Node: {}".format(target_id))
                        
                        # update the current state
                        self.last_visited_node = target_id 
                        self.path_queue.pop(0)
                        
                        # see if goal reached
                        if not self.path_queue:
                            rospy.loginfo("Global Goal Reached.")
                            self.lcs_pub.publish("STOP")
                            self.mission_state = "COMPLETED"
            
            elif self.mission_state == "EXECUTING" and not self.path_queue:
                 self.lcs_pub.publish("STOP") 

            rate.sleep()

if __name__ == '__main__':
    DeliberativeLayer().run()
