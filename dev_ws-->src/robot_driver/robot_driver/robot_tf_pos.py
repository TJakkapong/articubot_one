import rclpy
import math
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import numpy as np
def quaternion_from_euler(ai, aj, ak):
    
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(math.radians(ai))
    si = math.sin(math.radians(ai))
    cj = math.cos(math.radians(aj))
    sj = math.sin(math.radians(aj))
    ck = math.cos(math.radians(ak))
    sk = math.sin(math.radians(ak))
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class posRobot_subscriber(Node):

    def __init__(self):
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        super().__init__('pos_sub')
        self.subscription_baselink = self.create_subscription(
            Vector3Stamped,
            '/posvector',
            self.baselink_callback,
            10)
        self.timer = self.create_timer(0.0001,self.timer_callback)
        self.odom_pub = self.create_publisher(Odometry,'odom',10)
        self.t= TransformStamped()
        self.odom = Odometry()
        self.pos_broadcaster = TransformBroadcaster(self)
        self.subscription_baselink  # prevent unused variable warning

            
    def timer_callback(self):
        self.odom.header.stamp = self.t.header.stamp
        self.odom.header.stamp.nanosec = self.t.header.stamp.nanosec
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        self.odom.pose.pose.position.x = self._x 
        self.odom.pose.pose.position.y = self._y
        self.odom.pose.pose.position.z   = 0.0

        q = quaternion_from_euler(0, 0, self._th)
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(self.odom)
        
        self.pos_broadcaster.sendTransform(self.t)
    def baselink_callback(self, msg):
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        self.t.header.frame_id = 'odom'
        self.t.child_frame_id = 'base_link'
        self.t.transform.translation.x = msg.vector.x
        self.t.transform.translation.y = msg.vector.y
        self.t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, msg.vector.z)

        self._x = msg.vector.x
        self._y = msg.vector.y
        self._th = msg.vector.z
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
        #self.get_logger().info("2")

def main(args=None):
    rclpy.init(args=args)

    node = posRobot_subscriber()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
