"""Keyboard teleop: reads keys from the terminal and publishes geometry_msgs/Twist.

Run in its own terminal (it needs keyboard focus):
    ros2 run robotics_starter teleop_keyboard
    ros2 run robotics_starter teleop_keyboard --ros-args -r cmd_vel:=/turtle1/cmd_vel
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from robotics_starter.motion import TELEOP_HELP, TeleopController

# ANSI escape sequences sent by the arrow keys.
ARROWS = {'\x1b[A': 'up', '\x1b[B': 'down', '\x1b[C': 'right', '\x1b[D': 'left'}


def read_key(timeout: float = 0.1) -> str:
    """Read one key (or arrow-key sequence) from stdin; '' on timeout."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return ''
    key = sys.stdin.read(1)
    if key == '\x1b':
        key += sys.stdin.read(2)
        key = ARROWS.get(key, '')
    return key


class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('angular_step', 0.2)
        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_angular', 2.0)
        p = self.get_parameter
        self.controller = TeleopController(
            linear_step=p('linear_step').value,
            angular_step=p('angular_step').value,
            max_linear=p('max_linear').value,
            max_angular=p('max_angular').value,
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def publish(self):
        msg = Twist()
        msg.linear.x = float(self.controller.v)
        msg.angular.z = float(self.controller.w)
        self.publisher.publish(msg)


def main(args=None):
    if not sys.stdin.isatty():
        print('teleop_keyboard needs an interactive terminal (use `ros2 run`, not a launch file).')
        return
    rclpy.init(args=args)
    node = TeleopKeyboard()
    settings = termios.tcgetattr(sys.stdin)
    print(TELEOP_HELP + '  CTRL-C     : quit\n')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key()
            if key == '\x03':  # CTRL-C in cbreak mode
                break
            if key and node.controller.handle_key(key) is not None:
                print(f'\rv = {node.controller.v:+.2f} m/s   '
                      f'w = {node.controller.w:+.2f} rad/s   ', end='', flush=True)
            # Publish continuously (~10 Hz) so the robot keeps moving.
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.controller.stop()
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
