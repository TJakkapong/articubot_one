import rclpy
import math
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped


class PosRobotSubscriber(Node):

    def __init__(self):
        super().__init__('pos_sub')
        self.subscription_leftwheel = self.create_subscription(
            Vector3Stamped,
            '/leftwheel',
            self.leftwheel_callback,
            10)
        self.subscription_rightwheel = self.create_subscription(
            Vector3Stamped,
            '/rightwheel',
            self.rightwheel_callback,
            10)
        self.wheel_msg = JointState()
        self.wheel_msg.name.append('left_wheel_joint')
        self.wheel_msg.name.append('right_wheel_joint')

        self.wheel_msg.position.append(0.0)
        self.wheel_msg.position.append(0.0)
        self.wheel_msg.velocity.append(0.0)
        self.wheel_msg.velocity.append(0.0)

        self.wheel_msg.header.stamp = self.get_clock().now().to_msg()
        self.time_track = self.get_clock().now().nanoseconds
        self.state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def leftwheel_callback(self, msg):
        self.wheel_msg.velocity[0] = float(msg.vector.y)

    def rightwheel_callback(self, msg):
        self.wheel_msg.velocity[1] = float(msg.vector.y)

    def timer_callback(self):
        now = self.get_clock().now()

        time_diff = (now.nanoseconds - self.time_track)/ 1e9
        # Calculate the position based on the velocity
        self.wheel_msg.position[0] += self.wheel_msg.velocity[0]/0.15 * time_diff
        self.wheel_msg.position[1] += self.wheel_msg.velocity[1]/0.15 * time_diff
        self.time_track = now.nanoseconds
        # Publish the updated joint state
        self.wheel_msg.header.stamp = now.to_msg()
        self.wheel_msg.header.stamp.nanosec= now.to_msg().nanosec
        self.state_publisher.publish(self.wheel_msg)


def main(args=None):
    rclpy.init(args=args)

    node = PosRobotSubscriber()
    print("Node Start")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
