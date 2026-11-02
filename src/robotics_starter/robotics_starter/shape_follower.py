"""Shape follower: drives a square or a circle by publishing geometry_msgs/Twist.

    ros2 run robotics_starter shape_follower --ros-args -p shape:=circle
    ros2 run robotics_starter shape_follower --ros-args \
        -p shape:=square -p size:=2.0 -r cmd_vel:=/turtle1/cmd_vel
"""

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from robotics_starter.motion import ShapeFollower


class ShapeFollowerNode(Node):

    def __init__(self):
        super().__init__('shape_follower')
        # dynamic_typing lets users pass `size:=2` as well as `size:=2.0`.
        number = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('shape', 'square')                # 'square' or 'circle'
        self.declare_parameter('size', 2.0, number)              # side / radius (m)
        self.declare_parameter('linear_speed', 0.5, number)      # m/s
        self.declare_parameter('angular_speed', 0.8, number)     # rad/s (square turns)
        self.declare_parameter('loops', 1, number)
        self.declare_parameter('rate', 20.0, number)             # Hz
        p = self.get_parameter

        self.follower = ShapeFollower(
            shape=p('shape').value,
            size=float(p('size').value),
            linear_speed=float(p('linear_speed').value),
            angular_speed=float(p('angular_speed').value),
            loops=int(p('loops').value),
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = self.get_clock().now()
        self.done = False
        self.timer = self.create_timer(1.0 / float(p('rate').value), self.on_timer)
        self.get_logger().info(
            f'Driving a {self.follower.shape} '
            f'({self.follower.total_duration:.1f} s, {len(self.follower.segments)} segments)')

    def on_timer(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        v, w, done = self.follower.command(elapsed)
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.publisher.publish(msg)
        if done and not self.done:
            self.done = True
            self.get_logger().info('Shape complete - robot stopped. Ctrl-C to exit.')
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = ShapeFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())  # make sure the robot stops
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
