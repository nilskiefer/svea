#! /usr/bin/env python3

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_control')
        self.velocity = 0.0
        self.publisher = self.create_publisher(Float32, '/lli/ctrl/throttle', 10)
        self.twist_sub = self.create_subscription(
            Twist, 'cmd_vel', self.set_velocity, 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        msg = Float32()
        msg.data = self.velocity  # Example linear velocity
        self.publisher.publish(msg)
    
    def set_velocity(self, msg):
        self.velocity = msg.linear.x
        self.get_logger().info(f'Set velocity to: {self.velocity}')



def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopControl()
    rclpy.spin(teleop)
    

    try:
        with rclpy.init(args=args):
            teleop = TeleopControl()
            rclpy.spin(teleop)
    except (KeyboardInterrupt, ExternalShutdownException):
        teleop.destroy_node()
        rclpy.shutdown()
        pass

if __name__ == '__main__':
    main()
