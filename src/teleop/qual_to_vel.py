import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mocap4r2_msgs.msg import RigidBodies
from scipy.spatial.transform import Rotation

class VelocityNode(Node):
    def __init__(self):
        super().__init__('qualisys_velocity_node')
        self.sub = self.create_subscription(RigidBodies, '/rigid_bodies', self.callback, 10)
        self.pub = self.create_publisher(Twist, '/qualisys/velocity', 10)
        self.last_pos = None
        self.last_time = None

    def callback(self, msg):
        if not msg.rigidbodies:
            return
        rb = msg.rigidbodies[3]  # Use correct index for your rigid body
        pos = rb.pose.position
        quat = rb.pose.orientation
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Check for zero-norm quaternion
        norm = (quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2) ** 0.5
        if norm < 1e-6:
            self.get_logger().warn("Received zero-norm quaternion, skipping frame.")
            self.last_pos = pos
            self.last_time = now
            return

        if self.last_pos is not None and self.last_time is not None:
            dt = now - self.last_time
            if dt > 0:
                vx = (pos.x - self.last_pos.x) / dt
                vy = (pos.y - self.last_pos.y) / dt
                vz = (pos.z - self.last_pos.z) / dt

                r = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
                vel_body = r.inv().apply([vx, vy, vz])

                twist = Twist()
                twist.linear.x = vel_body[0]
                twist.linear.y = vel_body[1]
                twist.linear.z = vel_body[2]
                self.pub.publish(twist)
        self.last_pos = pos
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = VelocityNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()