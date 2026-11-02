"""Minimal publisher: sends a std_msgs/String on /chatter once per period."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.declare_parameter('message', 'Hello robotics')
        self.declare_parameter('period', 0.5)
        period = self.get_parameter('period').value
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.count = 0
        self.timer = self.create_timer(period, self.on_timer)

    def on_timer(self):
        msg = String()
        msg.data = f"{self.get_parameter('message').value} #{self.count}"
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
