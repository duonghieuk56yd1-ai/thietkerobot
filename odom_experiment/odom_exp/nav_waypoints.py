#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav_waypoints.py — Gửi tuần tự 5 goal NavigateToPose (Nav2) để TurtleBot3 chạy
quỹ đạo kín hình vuông quanh trụ giữa của turtlebot3_world (frame map).

    python3 nav_waypoints.py              # 1 vòng
    python3 nav_waypoints.py --loops 2    # 2 vòng liên tiếp

Ctrl+C giữa chừng: hủy goal đang chạy rồi thoát an toàn.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# (tên, x [m], y [m], yaw [độ]) — frame map, trùng tọa độ thế giới Gazebo
WAYPOINTS = [
    ('WP1', 0.55, -0.55, 90.0),
    ('WP2', 0.55, 0.55, 180.0),
    ('WP3', -0.55, 0.55, -90.0),
    ('WP4', -0.55, -0.55, 0.0),
    ('Start', -2.0, -0.5, 0.0),
]

STATUS_NAME = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def fmt_num(v):
    """0.55 -> '0.55', -2.0 -> '-2.0', -0.5 -> '-0.5'"""
    s = f'{v:.2f}'.rstrip('0')
    return s + '0' if s.endswith('.') else s


def fmt_wp(x, y, yaw_deg):
    return f'({fmt_num(x)}, {fmt_num(y)}, {yaw_deg:g}°)'


def init_rclpy():
    """Ctrl+C chỉ sinh KeyboardInterrupt, context ROS vẫn sống để gửi lệnh hủy goal."""
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()


class WaypointRunner(Node):
    def __init__(self):
        super().__init__('odom_exp_nav_waypoints')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self)
        self.goal_handle = None

    def spin_until(self, future, timeout_s=None):
        end = None if timeout_s is None else time.monotonic() + timeout_s
        while not future.done():
            if end is not None and time.monotonic() > end:
                return False
            self.ex.spin_once(timeout_sec=0.1)
        return True

    def wait_server(self):
        while not self.client.wait_for_server(timeout_sec=2.0):
            print('Chờ Nav2 action server /navigate_to_pose ... (kiểm tra T2 đã chạy và đặt initialpose)',
                  flush=True)

    @staticmethod
    def make_pose(x, y, yaw_deg):
        p = PoseStamped()
        p.header.frame_id = 'map'
        # stamp = 0: Nav2 dùng TF mới nhất (tránh lệch giữa wall time và sim time)
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = 0.0
        yaw = math.radians(yaw_deg)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def go_to(self, x, y, yaw_deg, timeout_s):
        """Trả về (status, thời gian chạy [s])."""
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw_deg)

        t0 = time.monotonic()
        send_future = self.client.send_goal_async(goal)
        if not self.spin_until(send_future, 10.0):
            return GoalStatus.STATUS_UNKNOWN, time.monotonic() - t0
        self.goal_handle = send_future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.goal_handle = None
            return None, time.monotonic() - t0   # bị từ chối

        result_future = self.goal_handle.get_result_async()
        if not self.spin_until(result_future, timeout_s):
            print(f'   quá {timeout_s:.0f} s -> hủy goal', flush=True)
            self.cancel_current()
            return GoalStatus.STATUS_CANCELED, time.monotonic() - t0
        self.goal_handle = None
        return result_future.result().status, time.monotonic() - t0

    def cancel_current(self):
        gh = self.goal_handle
        self.goal_handle = None
        if gh is None:
            return
        print('Đang hủy goal hiện tại ...', flush=True)
        cancel_future = gh.cancel_goal_async()
        if self.spin_until(cancel_future, 5.0):
            print('Đã hủy goal.', flush=True)
        else:
            print('Không nhận được xác nhận hủy goal (Nav2 có thể đã dừng).', flush=True)


def parse_args():
    ap = argparse.ArgumentParser(description='Chạy chuỗi waypoint Nav2 quanh trụ giữa turtlebot3_world.')
    ap.add_argument('--loops', type=int, default=1, help='số vòng (mặc định 1)')
    ap.add_argument('--timeout', type=float, default=180.0,
                    help='thời gian tối đa cho mỗi waypoint, giây (mặc định 180)')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])
    if args.loops < 1:
        ap.error('--loops phải >= 1')
    return args


def main():
    args = parse_args()
    init_rclpy()
    node = WaypointRunner()
    n = len(WAYPOINTS)
    ok_count, total, failed = 0, args.loops * n, []
    interrupted = False
    try:
        node.wait_server()
        for lap in range(1, args.loops + 1):
            for i, (name, x, y, yaw) in enumerate(WAYPOINTS, start=1):
                label = f'WP{i}/{n}' + ('' if name.startswith('WP') else f' ({name})')
                print(f'[vòng {lap}/{args.loops}] {label} -> {fmt_wp(x, y, yaw)} ...', flush=True)
                status, dt = node.go_to(x, y, yaw, args.timeout)
                if status is None:
                    print(f'   {name} bị Nav2 TỪ CHỐI goal', flush=True)
                    failed.append(f'vòng {lap} {name}: REJECTED')
                elif status == GoalStatus.STATUS_SUCCEEDED:
                    ok_count += 1
                    print(f'   -> tới {name} ({dt:.1f} s)', flush=True)
                else:
                    st = STATUS_NAME.get(status, str(status))
                    print(f'   {name} kết thúc với trạng thái {st} ({dt:.1f} s)', flush=True)
                    failed.append(f'vòng {lap} {name}: {st}')
    except KeyboardInterrupt:
        interrupted = True
        print('\nCtrl+C: dừng chuỗi waypoint.', flush=True)
        node.cancel_current()
    finally:
        if not interrupted:
            if failed:
                print(f'XONG tất cả waypoint ({ok_count}/{total} thành công). Lỗi: ' + '; '.join(failed))
            else:
                print(f'XONG tất cả waypoint ({ok_count}/{total} thành công)')
        node.ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
