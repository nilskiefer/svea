#! /usr/bin/env python3

from sensor_msgs.msg import Joy, JoyFeedback, JoyFeedbackArray
from std_msgs.msg import Int8
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

class SteeringControl(Node):

    def __init__(self):
        super().__init__('steering_control')
        self.velocity = 0
        self.steering = 0
        # Publishers - match the C interface topics
        self.vel_pub = self.create_publisher(Int8, '/lli/ctrl/throttle', 10)
        self.steering_pub = self.create_publisher(Int8, '/lli/ctrl/steering', 10)
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.01, self.publish_commands)
        from std_msgs.msg import Bool
        self.high_gear_pub = self.create_publisher(Bool, '/lli/ctrl/high_gear', 10)
        self.high_gear = False

    def publish_commands(self):
        # Create Int8 messages for steering and throttle
        steering_msg = Int8()
        steering_msg.data = self.steering
        self.steering_pub.publish(steering_msg)

        throttle_msg = Int8()
        throttle_msg.data = self.velocity
        self.vel_pub.publish(throttle_msg)


    def joy_callback(self, msg):
        # axes[0]: steering (left/right)
        # axes[2]: throttle (forward)
        # axes[3]: brake (reverse)
        # buttons[4]: button 5 (index 4), buttons[5]: button 6 (index 5)

        # Gear selection and publish to /lli/ctrl/high_gear only on change
        if len(msg.buttons) > 5:
            from std_msgs.msg import Bool
            if msg.buttons[5] and not self.high_gear:  # Button 6 pressed, switch to high
                self.high_gear = True
                msg_out = Bool()
                msg_out.data = True
                self.high_gear_pub.publish(msg_out)
            if msg.buttons[4] and self.high_gear:  # Button 5 pressed, switch to low
                self.high_gear = False
                msg_out = Bool()
                msg_out.data = False
                self.high_gear_pub.publish(msg_out)

        # Double the sensitivity: only need to turn half as much
        self.steering = int(msg.axes[0] * -254)  # Negative for correct direction, double range

        # Throttle: cubic scaling for even smoother low-speed control
        # axes[2]: [-1, 1] -> [0, 1], then cube for very gradual response
        forward = (msg.axes[2] + 1.0) / 2.0
        forward_scaled = forward ** 3
        reverse = (msg.axes[3] + 1.0) / 2.0
        reverse_scaled = reverse ** 3

        # Net velocity: forward minus reverse
        forward_val = int(forward_scaled * 127)
        reverse_val = int(reverse_scaled * 127)
        self.velocity = forward_val - reverse_val

        #slow down for demo

        #self.velocity = int(self.velocity * (1/3))


        # Clamp values to Int8 range
        self.steering = max(-127, min(127, self.steering))
        self.velocity = max(-127, min(127, self.velocity))

        self.get_logger().info(f'Steering: {self.steering}, Throttle: {self.velocity}, High Gear: {self.high_gear}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop = SteeringControl()
        rclpy.spin(teleop)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if 'teleop' in locals():
            teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
