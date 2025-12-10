#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Since we are using python 2 we need to make sure we include utf-8 encoding

import rospy
import random
import math
import time

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String, Bool
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped

# conversion from feet to meters
FT_TO_M = 0.3048

class ReactiveController(object):

    def __init__(self):
        rospy.init_node('reactive_controller', anonymous=False)

        # parameters for robot movement either get from defined paramter or use default value here
        # these are in meters per second or in radians per second
        self.forward_speed = rospy.get_param('~forward_speed', 0.2)
        self.turn_speed = rospy.get_param('~turn_speed', 0.6)
        self.avoid_range_m = rospy.get_param('~avoid_range_m', 0.4)
        self.symmetric_eps_m = rospy.get_param('~symmetric_eps_m', 0.05)
        self.escape_spread_rad = rospy.get_param('~escape_spread_rad', math.radians(30.0))
        self.teleop_timeout = rospy.get_param('~teleop_timeout', 0.4)
        self.log_every = rospy.get_param('~log_every', 10)

        self.stuck_start_time = None
        self.last_obstacle_time = 0
        self.stuck_triggered = False
    # time we last saw an obstacle
	self.persistence_timer = 0.0  
        # where was it
        self.last_obstacle_side = 0    

        # creates a publisher that sends commands to the turtlebot
        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.stuck_pub = rospy.Publisher('/robot_stuck', Bool, queue_size=1)

        # subscribers
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bump_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        rospy.Subscriber('/teleop_cmd', Twist, self.teleop_cb)
	rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)

        # planning subscribers
        rospy.Subscriber('/current_lcs', String, self.lcs_cb)
        rospy.Subscriber('/current_goal', Point, self.goal_cb)

        # variables for the current state
        self.halt = False
        self.mode = 'IDLE'
        self.lcs_strategy = "IDLE"
        self.current_goal_xy = None

        # stuck variables 
        self.stuck_start_time = None
        self.stuck_triggered = False

        # current and teleop variables
        self.last_teleop = None
        self.last_teleop_time = 0.0
        self.angle = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.turn_target = None

        self.has_scan = False
        self.d_front = float('inf')
        self.d_left = float('inf')
        self.d_right = float('inf')

        self._dbg_i = 0
        self.rate = rospy.Rate(20)

    # callbacks for different robot states 
    def lcs_cb(self, msg):
        self.lcs_strategy = msg.data

    def goal_cb(self, msg):
        self.current_goal_xy = (msg.x, msg.y)

    # if the bumper is pressed stop the turtlebot
    def bump_cb(self, msg):
        if msg.state == BumperEvent.PRESSED:
            self.halt = True
            self.mode = 'HALT'
    # update the teleop message and time
    def teleop_cb(self, msg):
        self.last_teleop = msg
        self.last_teleop_time = time.time()

    # track the angle and distance traveled
    def odom_cb(self, msg):
        # extract the angle from the turtlebot
        q = msg.pose.pose.orientation
        (_, _, angle) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angle = angle

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    # scanning callback
    def scan_cb(self, scan):
        n = len(scan.ranges)
        if n == 0:
            self.has_scan = False
            return

        mid = n // 2
        # degrees calculation
        def bins_for_deg(deg):
            return max(1, int(abs(math.radians(deg)) / scan.angle_increment))

        w_front = bins_for_deg(10.0)
        w_side = bins_for_deg(15.0)
        off30 = bins_for_deg(30.0)

        def sector_min_by_index(center_idx, half_width):
        # calculate low and high ends of sectors
            lo = max(0, center_idx - half_width)
            hi = min(n - 1, center_idx + half_width)
            # add distances in the range
            vals = [r for r in scan.ranges[lo:hi + 1] if not (math.isinf(r) or math.isnan(r))]
            # return the closest obstacle in sector
            return min(vals) if vals else float('inf')

        # find closest obstacle in each sector
        self.d_front = sector_min_by_index(mid, w_front)
        self.d_left = sector_min_by_index(min(n - 1, mid + off30), w_side)
        self.d_right = sector_min_by_index(max(0, mid - off30), w_side)
        self.has_scan = True

    def amcl_cb(self, msg):
        # update position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # update orientation
        q = msg.pose.pose.orientation
        (_, _, angle) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angle = angle

    def teleop_active(self):
        # if teleop velocity is nonzero consider it active
        if self.last_teleop is None:
            return False
        if abs(self.last_teleop.linear.x) > 0.01 or abs(self.last_teleop.angular.z) > 0.01:
            return True
        # otherwise use timeout to check 
        return (time.time() - self.last_teleop_time) <= self.teleop_timeout

    # get small angle difference
    def ang_diff(self, a, b):
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi


    def check_stuck_condition(self):

        if self.lcs_strategy != "GO_TO_POINT":
            return

        now = rospy.get_time()

	# 1. We are blocked if in AVOID, OR if in BLENDED mode and close to front wall
	is_blocked = (self.mode == 'AVOID') or (self.mode == 'BLENDED' and self.d_front < 0.65)

        if is_blocked:
            self.last_obstacle_time = now

        if self.stuck_start_time is None:
            self.stuck_start_time = now

        if self.stuck_start_time is not None:
            total_struggle_time = now - self.stuck_start_time
            time_since_last_block = now - self.last_obstacle_time

            if total_struggle_time > 5.0:
                if not self.stuck_triggered:
                    rospy.logwarn(
                        "STUCK TRIGGERED. Robot has been struggling for {:.1f}s".format(total_struggle_time)
                    )
                    self.stuck_pub.publish(True)
                    self.stuck_triggered = True

            if time_since_last_block > 2.0:
                self.stuck_start_time = None
                self.stuck_triggered = False


    def choose_behavior(self):
        
        # check if turtlebot bumper is pressed and in halt mode
        self.check_stuck_condition()

        if self.halt:
            self.mode = 'HALT'
            return Twist()

        # check if teleop is active
        if self.teleop_active():
            self.mode = 'TELEOP'
            return self.last_teleop

        # calculate distance and goal data
        heading_error = 0.0
        dist_to_goal = 0.0
        # set go to point for strategy and calculate data
        if self.lcs_strategy == "GO_TO_POINT" and self.current_goal_xy:
            dx = self.current_goal_xy[0] - self.current_x
            dy = self.current_goal_xy[1] - self.current_y
            target_angle = math.atan2(dy, dx)
            heading_error = self.ang_diff(target_angle, self.angle)
            dist_to_goal = math.sqrt(dx**2 + dy**2)

        # analyze sensors
        min_dist = float('inf')
        if self.has_scan:
            min_dist = min(self.d_left, self.d_front, self.d_right)

        cmd = Twist()

        # check if too close 
        if min_dist < self.avoid_range_m: 
            self.mode = 'AVOID'
            cmd.linear.x = 0.0
            
	    # if stuck turn right until free
            if self.stuck_triggered == True:
		cmd.angular.z = -self.turn_speed 
        # turn away from current facing
            elif self.d_left < self.d_right: 
                cmd.angular.z = -self.turn_speed 
            else:
                cmd.angular.z = self.turn_speed  
            return cmd


        now = rospy.get_time()
        is_in_memory = (now - self.persistence_timer) < 4.0
        
        # if too close 
        if self.lcs_strategy == "GO_TO_POINT" and (min_dist < (self.avoid_range_m * 2.5) or is_in_memory):
            
            # actively avoid
            if min_dist < (self.avoid_range_m * 2.5):
                self.mode = 'BLENDED'
                
                # update timer
                if min_dist < (self.avoid_range_m * 2.2):
                    self.persistence_timer = now 

                # scale down the repulsion to avoid
                repulsion_scale = 1.0
                if dist_to_goal < 1.0:
                    # scales down from 1 to 0.2
                    repulsion_scale = max(0.2, dist_to_goal) 

                # go towards goal
                f_goal = 0.6 * heading_error

                # push from wall left and right
                push_left = 0.0
                push_right = 0.0
                
                if self.d_left < 1.0: 
                    # get repulsion scale for right
                    push_right = (-0.6 / (self.d_left * self.d_left)) * repulsion_scale
                
                if self.d_right < 1.0:
                    # get repulsion scale for left
                    push_left = (0.6 / (self.d_right * self.d_right)) * repulsion_scale
                
                # get obstacle side
                if abs(push_right) > abs(push_left):
                    self.last_obstacle_side = 1 
                else:
                    self.last_obstacle_side = -1 
                
                f_obs = push_left + push_right

                # combine 
                cmd.angular.z = f_goal + f_obs
                cmd.linear.x = self.forward_speed * 0.4

            # blind spot navigation
            elif is_in_memory:
                self.mode = 'BLENDED_MEM'
                
                # go straight
                cmd.angular.z = 0.0 
                cmd.linear.x = self.forward_speed * 0.6

            # limit rotation speed
            cmd.angular.z = max(min(cmd.angular.z, self.turn_speed), -self.turn_speed)
            
            return cmd

        # navigate open space
        elif self.lcs_strategy == "GO_TO_POINT":
            self.mode = 'GO_TO_POINT'
            
            # if facing wrong way turn in place
            if abs(heading_error) > 0.3: 
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed if heading_error > 0 else -self.turn_speed
            else:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.8 * heading_error
            return cmd

        # idle mode
        elif self.lcs_strategy == "STOP":
            self.mode = 'STOPPED'
            return Twist()

        self.mode = 'IDLE'
        return Twist()

    # spin movement
    def spin(self):
        while not rospy.is_shutdown():
            cmd = self.choose_behavior()
            self.cmd_pub.publish(cmd)
            self._dbg_i += 1

            if self.log_every > 0 and (self._dbg_i % self.log_every == 0):
                rospy.loginfo("mode=%s Strategy=%s", self.mode, self.lcs_strategy)

            self.rate.sleep()


if __name__ == '__main__':
    ReactiveController().spin()
