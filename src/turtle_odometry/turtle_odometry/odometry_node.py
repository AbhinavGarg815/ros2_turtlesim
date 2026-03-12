import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose

import tf2_ros

def yaw_to_quaternion(yaw: float):
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))

class TurtleOdometryNode(Node):

    WORLD_CENTRE_X = 5.5
    WORLD_CENTRE_Y = 5.5

    def __init__(self):
        super().__init__('turtle_odometry')

        self._odom_pub = self.create_publisher(Odometry, '/turtle1/odom', 10)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self._publish_static_map_odom_tf()

        self._pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self._pose_callback,
            10,
        )

        self.get_logger().info(
            'TurtleOdometryNode started.\n'
            f'(offset x={self.WORLD_CENTRE_X}, y={self.WORLD_CENTRE_Y})\n'
        )

    def _publish_static_map_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = self.WORLD_CENTRE_X
        t.transform.translation.y = self.WORLD_CENTRE_Y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._static_tf_broadcaster.sendTransform(t)
        self.get_logger().info('Static TF map → odom published.')

    def _pose_callback(self, pose: Pose):
        now = self.get_clock().now().to_msg()

        x_odom = pose.x - self.WORLD_CENTRE_X
        y_odom = pose.y - self.WORLD_CENTRE_Y

        qx, qy, qz, qw = yaw_to_quaternion(pose.theta)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x_odom
        odom.pose.pose.position.y = y_odom
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = pose.linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = pose.angular_velocity

        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'

        tf.transform.translation.x = x_odom
        tf.transform.translation.y = y_odom
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
