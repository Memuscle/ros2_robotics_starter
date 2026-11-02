"""Service client: calls /add_two_ints once and prints the result.

Usage:
    ros2 run robotics_starter add_client 3 4
    ros2 run robotics_starter add_client --ros-args -p a:=3 -p b:=4
"""

import sys

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


class AddClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.declare_parameter('a', 2)
        self.declare_parameter('b', 3)
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

    def call(self, a: int, b: int):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /add_two_ints ...')
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = AddClient()
    # Positional args (after removing ROS args) override the parameters.
    cli = remove_ros_args(sys.argv)[1:]
    try:
        if len(cli) >= 2:
            a, b = int(cli[0]), int(cli[1])
        else:
            a = node.get_parameter('a').value
            b = node.get_parameter('b').value
        result = node.call(a, b)
        node.get_logger().info(f'Result: {a} + {b} = {result.sum}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
