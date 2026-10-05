"""Keyboard or continuous forward control with Isaac Sim's bundled ROS 2 Humble."""

import argparse
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty

# Use the simulator's Python 3.11 bindings, including its message definitions.
bridge = (Path(sys.prefix) / "lib/python3.11/site-packages/isaacsim/exts"
          / "isaacsim.ros2.bridge/humble/rclpy")
sys.path.insert(0, str(bridge))

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--speed", type=float, default=0.2)
    parser.add_argument("--turn", type=float, default=0.5)
    parser.add_argument("--forward", action="store_true",
                        help="Publish forward velocity continuously until Ctrl+C.")
    args = parser.parse_args()
    if not args.forward and not sys.stdin.isatty():
        parser.error("Run keyboard control in an interactive terminal.")

    keys = {"i": (1, 0), ",": (-1, 0), "j": (0, 1), "l": (0, -1),
            "u": (1, 1), "o": (1, -1), "k": (0, 0), " ": (0, 0)}
    # Keep ROS alive while the finally block sends a stop on Ctrl+C.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("omnilrs_keyboard")
    publisher = node.create_publisher(Twist, args.topic, 10)
    settings = None if args.forward else termios.tcgetattr(sys.stdin)
    command = Twist()
    last_key = 0.0
    print(f"ROS 2 Humble, domain {os.environ.get('ROS_DOMAIN_ID', '0')}, {args.topic}")
    if args.forward:
        command.linear.x = args.speed
        print(f"Continuous forward at {args.speed} m/s, Ctrl+C: stop and exit.", flush=True)
    else:
        print("Hold i: forward, comma: reverse, j/l: turn, k/space: stop, Ctrl+C: exit.")
        print("Commands stop after 0.5 seconds without a keypress.", flush=True)
    try:
        if not args.forward:
            tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if not args.forward:
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1)
                    if not key or key == "\x03":
                        break
                    linear, angular = keys.get(key, (0, 0))
                    command.linear.x = linear * args.speed
                    command.angular.z = angular * args.turn
                    last_key = time.monotonic()
                if time.monotonic() - last_key > 0.5:
                    command = Twist()
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.0)
            if args.forward:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            for _ in range(3):
                publisher.publish(Twist())
                time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
