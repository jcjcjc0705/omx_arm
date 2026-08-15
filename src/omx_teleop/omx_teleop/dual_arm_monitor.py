#!/usr/bin/env python3
"""Live side-by-side state monitor for the leader and follower arms.

Subscribes only -- never publishes, never claims an interface -- so it is safe
to run alongside a live teleop session.

    ros2 run omx_teleop dual_arm_monitor
    ros2 run omx_teleop dual_arm_monitor --check
    ros2 run omx_teleop dual_arm_monitor --joints gripper_joint_1
"""

import argparse
import curses
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from control_msgs.msg import DynamicJointState

SHOWN = ['position', 'velocity', 'temperature', 'voltage', 'error', 'moving']


class ArmState:

    def __init__(self, node, topic):
        self.topic = topic
        self.state = {}
        node.create_subscription(DynamicJointState, topic, self._on_state, 10)

    def _on_state(self, msg):
        for name, iface in zip(msg.joint_names, msg.interface_values):
            self.state[name] = dict(zip(iface.interface_names, iface.values))


class DualArmMonitor(Node):

    def __init__(self, args):
        super().__init__('dual_arm_monitor')
        self._filter = args.joints
        self.arms = [
            ('LEADER', ArmState(self, args.leader_topic)),
            ('FOLLOWER', ArmState(self, args.follower_topic)),
        ]

    def joints(self):
        """Union of both arms, so a joint missing on one side stays visible."""
        names = set()
        for _, arm in self.arms:
            names |= set(arm.state)
        if self._filter:
            names &= set(self._filter)
        return sorted(names)

    def ready(self):
        return all(arm.state for _, arm in self.arms)


def header():
    return f'{"":2}{"joint":<18}' + ''.join(f'{s[:10]:>11}' for s in SHOWN)


def row(joint, state):
    line = f'  {joint:<18}'
    for field in SHOWN:
        value = state.get(field)
        line += f'{value:>11.2f}' if isinstance(value, float) else f'{"--":>11}'
    return line


def snapshot(monitor):
    joints = monitor.joints()
    for label, arm in monitor.arms:
        print(f'=== {label} ({arm.topic}) ===')
        print(header())
        print('-' * len(header()))
        for joint in joints:
            print(row(joint, arm.state.get(joint, {})))
        print()


def addnstr(win, y, x, text, n, attr=0):
    try:
        win.addnstr(y, x, text, n, attr)
    except curses.error:
        pass


def run_tui(stdscr, monitor):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(200)

    while True:
        stdscr.erase()
        _, width = stdscr.getmaxyx()
        addnstr(stdscr, 0, 0, ' dual-arm monitor   q: quit '.ljust(width),
                width, curses.A_REVERSE)

        joints = monitor.joints()
        line = 2
        for label, arm in monitor.arms:
            addnstr(stdscr, line, 0, f'=== {label} ==='.ljust(width), width, curses.A_BOLD)
            line += 1
            addnstr(stdscr, line, 0, header(), width, curses.A_BOLD)
            line += 1
            for joint in joints:
                addnstr(stdscr, line, 0, row(joint, arm.state.get(joint, {})), width)
                line += 1
            line += 1
        stdscr.refresh()

        if stdscr.getch() in (ord('q'), 27):
            return


def build_parser():
    p = argparse.ArgumentParser(prog='dual_arm_monitor', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--leader-topic', default='/omx_leader/dynamic_joint_states')
    p.add_argument('--follower-topic', default='/omx_follower/dynamic_joint_states')
    p.add_argument('--joints', nargs='+', default=None,
                   help='only show these joints (default: every joint published)')
    p.add_argument('--check', action='store_true',
                   help='print one snapshot and exit')
    return p


def main():
    args = build_parser().parse_args()
    rclpy.init()
    monitor = DualArmMonitor(args)

    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    try:
        deadline = time.time() + 10.0
        while not monitor.ready() and time.time() < deadline:
            time.sleep(0.2)
        if not monitor.ready():
            print('no dynamic_joint_states from both arms, showing what arrived',
                  file=sys.stderr)

        if args.check:
            snapshot(monitor)
        else:
            curses.wrapper(run_tui, monitor)
        return 0
    finally:
        executor.shutdown()
        monitor.destroy_node()
        spin.join(timeout=2.0)
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
