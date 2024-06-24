#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import time

class SquareDrawer(Node):

    def __init__(self):
        super().__init__('square_drawer')
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

    def draw_square(self):
        self.set_pen(255, 0, 0, 2, 0)
        square_movements = [
            (2.0, 0.0, 2.0),
            (0.0, 1.57, 1.0),
            (2.0, 0.0, 2.0),
            (0.0, 1.57, 1.0),
            (2.0, 0.0, 2.0),
            (0.0, 1.57, 1.0),
            (2.0, 0.0, 2.0),
            (0.0, 1.57, 1.0)
        ]
        self.draw_shape(square_movements)

    def draw_shape(self, movements):
        twist = Twist()
        for movement in movements:
            twist.linear.x = movement[0]
            twist.angular.z = movement[1]
            self.publisher_.publish(twist)
            time.sleep(movement[2])

def main(args=None):
    rclpy.init(args=args)
    square_drawer = SquareDrawer()
    
    try:
        square_drawer.draw_square()
    except KeyboardInterrupt:
        pass

    square_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
