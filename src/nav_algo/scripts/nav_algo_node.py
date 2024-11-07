#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import std_msgs.msg
from std_msgs.msg import UInt8, Header
import builtin_interfaces.msg
from builtin_interfaces.msg import Time
import imu_process.msg
from imu_process.msg import Position
import uart_slave.msg
from uart_slave.msg import FocAngle

import time

from nav_algo import *
import multiprocessing
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# global constants
STOP = 0
FORWARD = 1
BACKWARD = 2
ANTICLOCKWISE = 3
CLOCKWISE = 4
TIMER_PERIOD_IN_SECOND = 0.001 # seconds

DEG_TO_RAD = 0.01745329 

def calculate_distance(self, dist, v_x, reached, dt):
    """Calculate the distance travelled by the robot"""
    self.get_logger().info("Calculating distance.....")

    dist += abs(v_x * dt) # seconds, dummy value
    self.get_logger().info("dist_x: %f" % dist)
    dt = 0.0 # reset the time counter
    self.get_logger().info("self.dt is reset: %f" % dt)
    if dist >= 0.03:
        # reached 30cm, reset the distance counter
        reached = True
        dist = 0.0
        self.get_logger().info("dist_x is reset to: %f" % dist)
        # print("dist is reset to: ", dist)
    self.get_logger().info("reached: %s \n" % reached)
    return dist, reached, dt

class NavAlgo(Node):

    def __init__(self):
        super().__init__('nav_algo') # initialize the node

        self.reentrant_callback_group = ReentrantCallbackGroup()
        
        # self.timer_ = self.create_timer(TIMER_PERIOD_IN_SECOND, self.timer_callback)
        # self.nav_sub_pos_local_ = self.create_subscription(imu_process.msg.Position, 'Imu_local', self.position_callback, 50)
        # self.nav_sub_focangle_ = self.create_subscription(uart_slave.msg.FocAngle, 'FOC_angle', self.focangle_callback, 10)
        # self.nav_pub_action_ = self.create_publisher(std_msgs.msg.UInt8, '/Robot_action', 10)

        self.timer_ = self.create_timer(
            TIMER_PERIOD_IN_SECOND, self.timer_callback, callback_group=self.reentrant_callback_group)
        self.nav_sub_pos_local_ = self.create_subscription(
            imu_process.msg.Position, 'Imu_local', 
            self.position_callback, 50, callback_group=self.reentrant_callback_group)
        self.nav_sub_focangle_ = self.create_subscription(
            uart_slave.msg.FocAngle, 'FOC_angle', 
            self.focangle_callback, 10, callback_group=self.reentrant_callback_group)
        self.nav_pub_action_ = self.create_publisher(
            std_msgs.msg.UInt8, '/Robot_action', 10, callback_group=self.reentrant_callback_group)
        
        # initialize the instance variables
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.is_moving = False
        self.distance_counter = 0.0
        self.reach_distance = False
        self.foc_left = 3.0 # dummy values
        self.foc_right = 3.0 # dummy values
        self.dt = 0.0 # for keeping time for integration
        
    def position_callback(self, msg):
        self.get_logger().info("current time: %d" % rclpy.clock.Clock().now().nanoseconds)
        self.get_logger().info("position msg time: %d" % msg.header.stamp.sec)

        # get data from message
        self.vel_x = round(msg.vel_x, 10)
        self.vel_y = round(msg.vel_y, 10)
        self.get_logger().info("self.vel_x: %f, self.vel_y: %f" % (self.vel_x, self.vel_y))
        
        # calculate the distance
        self.distance_counter, self.reach_distance, self.dt = calculate_distance(self, self.distance_counter, self.vel_x, self.reach_distance, self.dt)
        
        # threshold for determining if the robot is moving
        threshold = 0.0001
        
        if self.vel_x > threshold:
            self.is_moving = True
        self.get_logger().info("self.is_moving: %s" % self.is_moving)
        
        if self.reach_distance == True:
            # run the navigation algorithm
            self.get_logger().info("Running the navigation algorithm.....")
            self.get_logger().info("current time: %d" % rclpy.clock.Clock().now().nanoseconds)
            action = 10 # dummy value
            action = run_nav_algo(self.foc_left, self.foc_right, self.is_moving)
            self.get_logger().info("current time: %d" % rclpy.clock.Clock().now().nanoseconds)
            self.get_logger().info("action: %d" % action)

            # publish the action
            action_msg = std_msgs.msg.UInt8()
            action_msg.data = action
            self.get_logger().info("action_msg.data: %d" % action_msg.data)
            self.nav_pub_action_.publish(action_msg)
        
        # reset the boolean
        self.get_logger().info("Resetting the boolean variables.....")
        self.is_moving = False
        self.reach_distance = False
        
    def focangle_callback(self, msg):
        self.foc_left = round(msg.left, 10) / DEG_TO_RAD # convert to degree
        self.foc_right = round(msg.right, 10) / DEG_TO_RAD # convert to degree
        # self.get_logger().info("FOC left: %f, FOC right: %f" % (self.foc_left, self.foc_right))
    
    def timer_callback(self):
        self.dt += TIMER_PERIOD_IN_SECOND

def main(arg=None):
    rclpy.init(args=arg)
    nav_algo = NavAlgo() # create the node

    # use multithread
    executor = MultiThreadedExecutor()
    executor.add_node(nav_algo)

    print("Running the navigation node.....")

    try:
        executor.spin()
        print("this is working")
    except KeyboardInterrupt:
        print("Exiting program.")
        nav_algo.destroy_node()
        rclpy.shutdown()
        
    # rclpy.spin(nav_algo) # invoke the singlethreaded executor
    # rclpy.shutdown() 

if __name__ == '__main__':
    main()