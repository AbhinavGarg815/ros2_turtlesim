import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class SpawnTurtlesNode(Node):
    def __init__(self):
        super().__init__('spawn_turtles')

        self.declare_parameter('spawn_configs', ['turtle2,2.0,2.0,0.0'])
        raw = list(
            self.get_parameter('spawn_configs')
            .get_parameter_value().string_array_value
        )

        self._configs = []
        for entry in raw:
            parts = entry.split(',')
            if len(parts) != 4:
                self.get_logger().warn('Skipping bad spawn_config: "%s"' % entry)
                continue
            name  = parts[0].strip()
            x     = float(parts[1])
            y     = float(parts[2])
            theta = float(parts[3])
            self._configs.append((name, x, y, theta))

        self._client = self.create_client(Spawn, '/spawn')
        self._idx    = 0
        self._timer  = self.create_timer(0.5, self._try_spawn)

    def _try_spawn(self):
        if not self._client.service_is_ready():
            self.get_logger().info('Waiting for /spawn service...')
            return

        if self._idx >= len(self._configs):
            self.get_logger().info('All turtles spawned. Shutting down spawn node.')
            self._timer.cancel()
            raise SystemExit

        name, x, y, theta = self._configs[self._idx]
        req = Spawn.Request()
        req.name  = name
        req.x     = x
        req.y     = y
        req.theta = theta

        self.get_logger().info(
            'Spawning "%s" at (%.2f, %.2f, %.2f)...' % (name, x, y, theta)
        )
        future = self._client.call_async(req)
        future.add_done_callback(lambda f: self._spawn_done(f, name))
        self._idx += 1
        self._timer.cancel()  

    def _spawn_done(self, future, name: str):
        try:
            result = future.result()
            self.get_logger().info('Spawned "%s" -> got name "%s"' % (name, result.name))
        except Exception as e:
            self.get_logger().error('Spawn failed for "%s": %s' % (name, str(e)))

        self._timer = self.create_timer(0.1, self._try_spawn)

def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtlesNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
