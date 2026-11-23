#!/usr/bin/env python3
"""Lệnh nhanh cho cơ cấu nâng/băng tải của AMR và khay lật của robot chia chọn.

    ros2 run amr_gazebo amr_cmd.py lift up          # nâng khay lên 0.25 m (mặt băng 0.67 m)
    ros2 run amr_gazebo amr_cmd.py lift down        # hạ khay về 0.0 m (mặt băng 0.42 m)
    ros2 run amr_gazebo amr_cmd.py lift 0.12        # nâng tới vị trí bất kỳ trong [0, 0.25] m
    ros2 run amr_gazebo amr_cmd.py conveyor 0.2 3   # chạy băng tải 0.2 m/s trong 3 s rồi dừng
    ros2 run amr_gazebo amr_cmd.py conveyor 0       # dừng băng tải

  Robot chia chọn (sorter_agv.launch.py):
    ros2 run amr_gazebo amr_cmd.py tray left        # lật khay 30° thả hàng sang trái
    ros2 run amr_gazebo amr_cmd.py tray right       # lật khay 30° thả hàng sang phải
    ros2 run amr_gazebo amr_cmd.py tray flat        # khay về nằm ngang
    ros2 run amr_gazebo amr_cmd.py tray -0.3        # góc bất kỳ trong [-0.52, 0.52] rad
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

LIFT_MIN = 0.0
LIFT_MAX = 0.25
CONVEYOR_MAX = 0.3
TILT_MAX = 0.5236  # 30°; góc dương = nâng cạnh trái → hàng đổ sang phải


def usage():
    print(__doc__)
    sys.exit(1)


class AmrCmd(Node):

    def __init__(self):
        super().__init__('amr_cmd')
        self.lift_pub = self.create_publisher(Float64, '/lift_cmd', 10)
        self.conveyor_pub = self.create_publisher(Float64, '/conveyor_cmd', 10)
        self.tray_pub = self.create_publisher(Float64, '/tray_tilt_cmd', 10)

    def wait_for_subscriber(self, pub, timeout=3.0):
        """Chờ ros_gz_bridge kết nối để lệnh không bị mất."""
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while pub.get_subscription_count() == 0 and \
                self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            self.get_logger().warn(
                f'Chưa có subscriber trên {pub.topic_name} (bridge đã chạy chưa?)')

    def publish(self, pub, value, repeat=3):
        msg = Float64(data=float(value))
        for _ in range(repeat):
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def lift(self, arg):
        if arg == 'up':
            target = LIFT_MAX
        elif arg == 'down':
            target = LIFT_MIN
        else:
            target = float(arg)
        clamped = min(max(target, LIFT_MIN), LIFT_MAX)
        if clamped != target:
            self.get_logger().warn(
                f'Giới hạn hành trình [{LIFT_MIN}, {LIFT_MAX}] m: {target} -> {clamped}')
        self.wait_for_subscriber(self.lift_pub)
        self.publish(self.lift_pub, clamped)
        self.get_logger().info(
            f'/lift_cmd = {clamped:.3f} m (mặt băng ≈ {0.42 + clamped:.2f} m, '
            f'thời gian ≈ {abs(clamped) / 0.1:.1f} s nếu đi từ 0)')

    def conveyor(self, speed, duration=None):
        speed = min(max(float(speed), -CONVEYOR_MAX), CONVEYOR_MAX)
        self.wait_for_subscriber(self.conveyor_pub)
        self.publish(self.conveyor_pub, speed)
        self.get_logger().info(f'/conveyor_cmd = {speed:.2f} m/s')
        if duration is not None and speed != 0.0:
            end = self.get_clock().now().nanoseconds + int(float(duration) * 1e9)
            while self.get_clock().now().nanoseconds < end:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.publish(self.conveyor_pub, 0.0)
            self.get_logger().info('/conveyor_cmd = 0.00 m/s (dừng)')

    def tray(self, arg):
        presets = {'left': -TILT_MAX, 'right': TILT_MAX, 'flat': 0.0}
        target = presets[arg] if arg in presets else float(arg)
        clamped = min(max(target, -TILT_MAX), TILT_MAX)
        if clamped != target:
            self.get_logger().warn(
                f'Giới hạn góc lật [{-TILT_MAX}, {TILT_MAX}] rad: {target} -> {clamped}')
        self.wait_for_subscriber(self.tray_pub)
        self.publish(self.tray_pub, clamped)
        side = 'phải' if clamped > 0 else 'trái' if clamped < 0 else 'nằm ngang'
        self.get_logger().info(f'/tray_tilt_cmd = {clamped:.3f} rad ({side})')


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(argv) < 2:
        usage()
    rclpy.init()
    node = AmrCmd()
    try:
        if argv[0] == 'lift':
            node.lift(argv[1])
        elif argv[0] == 'conveyor':
            node.conveyor(argv[1], argv[2] if len(argv) > 2 else None)
        elif argv[0] == 'tray':
            node.tray(argv[1])
        else:
            usage()
    except ValueError:
        usage()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
