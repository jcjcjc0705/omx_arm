#!/usr/bin/env python3
"""Direct position mirroring: wherever the leader is dragged, the follower goes.

    /omx_leader/joint_states  ->  /omx_follower/position_controller/commands

Leader and follower are physically different arms, so each joint can need a
sign flip or an offset (mounting direction, zero pose). Those live in the
signs/offsets parameters, not in the code:

    follower_cmd[i] = signs[i] * leader_pos[i] + offsets[i]

Joints are matched by name, never by index -- joint_states is alphabetical
while the command array follows the controller's joints parameter.

Mirroring stays disarmed until every joint is within align_tolerance, so the
follower cannot snap across a large pose gap the moment teleop starts.
"""

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class LeaderFollowerMirror(Node):

    def __init__(self):
        super().__init__('leader_follower_mirror')

        leader_topic = self.declare_parameter(
            'leader_states_topic', '/omx_leader/joint_states').value
        follower_topic = self.declare_parameter(
            'follower_states_topic', '/omx_follower/joint_states').value
        controller = self.declare_parameter(
            'follower_controller', '/omx_follower/position_controller').value
        joint_names = list(self.declare_parameter('joint_names', ['']).value)
        if joint_names == ['']:
            joint_names = []

        self._order = joint_names or self._controller_joints(controller)
        n = len(self._order)

        self._signs = list(self.declare_parameter('signs', [1.0] * n).value)
        self._offsets = list(self.declare_parameter('offsets', [0.0] * n).value)
        self._tolerance = float(self.declare_parameter('align_tolerance', 0.15).value)
        self._gripper_joint = self.declare_parameter('gripper_joint', '').value
        self._gripper_open = float(self.declare_parameter('gripper_open', 0.0).value)

        if len(self._signs) != n or len(self._offsets) != n:
            raise ValueError(
                f'signs ({len(self._signs)}) and offsets ({len(self._offsets)}) '
                f'must each have {n} entries to match {self._order}')

        self._armed = False
        self._follower = None
        self._opened = False
        self._warned = False

        self._pub = self.create_publisher(Float64MultiArray, f'{controller}/commands', 10)
        self.create_subscription(JointState, follower_topic, self._on_follower, 10)
        self.create_subscription(JointState, leader_topic, self._on_leader, 10)

        self.get_logger().info(f'mirroring {leader_topic} -> {controller}/commands')
        self.get_logger().info(f'joints={self._order} signs={self._signs}')
        self.get_logger().warn(
            f'disarmed: move the leader to within {self._tolerance:.2f} rad of the follower')

    def _controller_joints(self, controller):
        client = self.create_client(GetParameters, f'{controller}/get_parameters')
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'cannot reach {controller}/get_parameters')
        future = client.call_async(GetParameters.Request(names=['joints']))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return list(future.result().values[0].string_array_value)

    def _mapped(self, leader):
        return [self._signs[i] * leader[name] + self._offsets[i]
                for i, name in enumerate(self._order)]

    def _open_gripper(self):
        """Hold every arm joint where it is and drive only the gripper open."""
        if self._gripper_joint not in self._order:
            self._opened = True
            return
        cmd = [self._follower.get(name, 0.0) for name in self._order]
        cmd[self._order.index(self._gripper_joint)] = self._gripper_open
        self._pub.publish(Float64MultiArray(data=cmd))
        self.get_logger().info(
            f'opening {self._gripper_joint} to {self._gripper_open:.3f} rad')
        self._opened = True

    def _on_follower(self, msg):
        self._follower = dict(zip(msg.name, msg.position))
        if not self._opened:
            self._open_gripper()

    def _on_leader(self, msg):
        if self._follower is None:
            return
        leader = dict(zip(msg.name, msg.position))

        missing = [j for j in self._order if j not in leader]
        if missing:
            if not self._warned:
                self.get_logger().error(f'leader has no {missing}')
                self._warned = True
            return

        cmd = self._mapped(leader)

        if not self._armed:
            gaps = {name: abs(cmd[i] - self._follower.get(name, 0.0))
                    for i, name in enumerate(self._order)}
            worst = max(gaps, key=gaps.get)
            if gaps[worst] > self._tolerance:
                self.get_logger().warn(
                    f'disarmed: {worst} off by {gaps[worst]:.3f} rad',
                    throttle_duration_sec=2.0)
                return
            self._armed = True
            self.get_logger().info('armed: mirroring live')

        self._pub.publish(Float64MultiArray(data=cmd))


def main():
    rclpy.init()
    node = LeaderFollowerMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
