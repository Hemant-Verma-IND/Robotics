#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import time

class StarDrawer(Node):

    def __init__(self):
        super().__init__('star_drawer')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.client_teleport = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        self.client_pen = self.create_client(SetPen, 'turtle1/set_pen')
        self.reset_turtle()
        time.sleep(1)  # Wait for the services to be available

    def reset_turtle(self):
        while not self.client_teleport.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting again...')
        req = TeleportAbsolute.Request()
        req.x = 5.5
        req.y = 5.5
        req.theta = 0.0
        self.client_teleport.call_async(req)

    def set_pen(self, r, g, b, width, off):
        while not self.client_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Pen service not available, waiting again...')
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.client_pen.call_async(req)

    def draw_star(self):
        self.set_pen(0, 0, 255, 2, 0)
        twist = Twist()
        
        # Length of one side of the star
        length = 5.0
        
        for _ in range(5):
            twist.linear.x = length
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(1.5)  # Move forward
            
            twist.linear.x = 0.0
            twist.angular.z = 4 * 3.14159 / 5  # Turn 144 degrees
            self.publisher_.publish(twist)
            time.sleep(1.0)  # Turning time

        # Stop the turtle at the end
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    star_drawer = StarDrawer()
    
    try:
        star_drawer.draw_star()
    except KeyboardInterrupt:
        pass

    star_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
