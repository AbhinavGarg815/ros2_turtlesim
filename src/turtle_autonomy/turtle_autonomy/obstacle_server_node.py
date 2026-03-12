import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtle_interfaces.srv import GetRobotPoses


class ObstacleServerNode(Node):
    def __init__(self):
        super().__init__('obstacle_server')

        # -- Parameters ------------------------------------------------------
        self.declare_parameter('robot_names', ['turtle1', 'turtle2'])
        robot_names = list(
            self.get_parameter('robot_names').get_parameter_value().string_array_value
        )

        self._poses: dict = {name: None for name in robot_names}

        for name in robot_names:
            topic = '/%s/pose' % name
            self.create_subscription(
                Pose,
                topic,
                self._make_pose_cb(name),
                10,
            )
            self.get_logger().info('Subscribed to %s' % topic)

        self._srv = self.create_service(
            GetRobotPoses,
            '/get_robot_poses',
            self._handle_request,
        )

        self.get_logger().info(
            'ObstacleServerNode ready.\n'
            '  Tracking robots : %s\n'
            '  Service         : /get_robot_poses' % robot_names
        )

    def _make_pose_cb(self, robot_name: str):
        def cb(msg: Pose):
            self._poses[robot_name] = (msg.x, msg.y)
        return cb

    def _handle_request(self, request: GetRobotPoses.Request, response: GetRobotPoses.Response):
        requester = request.requester_name

        names, xs, ys = [], [], []
        for name, pose in self._poses.items():
            if name == requester:
                continue          
            if pose is None:
                continue          
            names.append(name)
            xs.append(float(pose[0]))
            ys.append(float(pose[1]))

        response.names = names
        response.xs    = xs
        response.ys    = ys

        self.get_logger().debug(
            'Request from "%s" -> returning %d obstacle(s): %s'
            % (requester, len(names), list(zip(names, xs, ys)))
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
