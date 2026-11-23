#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
monitor_error.py — Ghi sai số Odometry ESP32 (odom1) so với Gazebo (odom2).

    python3 monitor_error.py              # lưu odom_error.csv + odom_error.png
    python3 monitor_error.py -o run2      # lưu run2.csv + run2.png

1. Gửi /esp32/reset (std_msgs/Empty) 2 lần -> ESP32 gán odom1 = odom2 (sai số = 0).
2. Ghi mỗi mẫu /esp32/odom_error (bỏ mẫu trùng timestamp) kèm odom1 (/esp32/odom1)
   và odom2 (/odom); cứ 5 s in tiến độ.
3. Ctrl+C: dừng ghi, in bảng KẾT QUẢ, lưu CSV và PNG.
"""
import argparse
import csv
import math
import os
import sys
import time
from collections import OrderedDict

import numpy as np

import matplotlib
matplotlib.use('Agg')  # lưu PNG không cần màn hình
import matplotlib.pyplot as plt  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402
from rclpy.utilities import remove_ros_args  # noqa: E402
from geometry_msgs.msg import Vector3  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import Empty  # noqa: E402

PROGRESS_PERIOD_S = 5.0
RESET_WAIT_TIMEOUT_S = 30.0
NO_DATA_WARN_S = 3.0
ODOM2_CACHE = 300                # số mẫu /odom giữ lại để ghép đúng stamp

CSV_HEADER = ['t_s', 'ex_m', 'ey_m', 'e_pos_m', 'e_yaw_deg',
              'x2', 'y2', 'yaw2', 'x1', 'y1', 'yaw1']


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def init_rclpy():
    """Tắt signal handler của rclpy để Ctrl+C chỉ sinh KeyboardInterrupt
    (context vẫn sống -> dọn dẹp và lưu file bình thường)."""
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()


class ErrorMonitor(Node):
    def __init__(self):
        super().__init__('odom_error_monitor')
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        self.reset_pub = self.create_publisher(Empty, '/esp32/reset', 10)
        self.create_subscription(Vector3, '/esp32/odom_error', self.on_error, qos)
        self.create_subscription(Odometry, '/esp32/odom1', self.on_odom1, qos)
        self.create_subscription(Odometry, '/odom', self.on_odom2, qos)

        self.recording = False
        self.rows = []                 # [t_s, ex, ey, e_pos, e_yaw_deg, x2, y2, yaw2, x1, y1, yaw1]
        self.t0_ns = None
        self.last_key_ns = None
        self.err_rx = 0                # tổng số /esp32/odom_error nhận (kể cả trùng)
        self.dup = 0
        self.last_err_wall = None

        self.odom1 = None              # (stamp_ns, x, y, yaw)
        self.odom2_latest = None       # (stamp_ns, x, y, yaw)
        self.odom2_cache = OrderedDict()

        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self)

    # ------------------------------------------------------------------ callbacks
    def on_odom1(self, msg):
        p = msg.pose.pose
        self.odom1 = (stamp_ns(msg.header.stamp), p.position.x, p.position.y,
                      yaw_from_quat(p.orientation))

    def on_odom2(self, msg):
        p = msg.pose.pose
        s = stamp_ns(msg.header.stamp)
        rec = (s, p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.odom2_latest = rec
        self.odom2_cache[s] = rec
        while len(self.odom2_cache) > ODOM2_CACHE:
            self.odom2_cache.popitem(last=False)

    def on_error(self, msg):
        self.err_rx += 1
        self.last_err_wall = time.monotonic()
        if not self.recording or self.odom1 is None:
            return

        key = self.odom1[0]            # stamp sim của mẫu odom2 mà ESP32 đã dùng
        if key == 0 or key == self.last_key_ns:
            self.dup += 1              # trùng timestamp -> bỏ
            return
        self.last_key_ns = key
        if self.t0_ns is None:
            self.t0_ns = key

        # odom2 đúng stamp nếu có, không thì lấy mẫu mới nhất
        o2 = self.odom2_cache.get(key, self.odom2_latest)
        if o2 is None:                 # chưa có /odom: suy ra từ odom1 - e
            o2 = (key, self.odom1[1] - msg.x, self.odom1[2] - msg.y,
                  math.atan2(math.sin(self.odom1[3] - msg.z), math.cos(self.odom1[3] - msg.z)))

        t_s = (key - self.t0_ns) * 1e-9
        e_pos = math.hypot(msg.x, msg.y)
        self.rows.append([t_s, msg.x, msg.y, e_pos, math.degrees(msg.z),
                          o2[1], o2[2], o2[3],
                          self.odom1[1], self.odom1[2], self.odom1[3]])

    # ------------------------------------------------------------------ helpers
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.ex.spin_once(timeout_sec=0.05)

    def reset_esp32(self):
        t_start = time.monotonic()
        last_msg = 0.0
        while self.reset_pub.get_subscription_count() == 0:
            now = time.monotonic()
            if now - last_msg >= 2.0:
                print('Chờ ESP32 subscribe /esp32/reset ... (kiểm tra T3 và đèn LED)', flush=True)
                last_msg = now
            if now - t_start > RESET_WAIT_TIMEOUT_S:
                print('CẢNH BÁO: không thấy ESP32 subscribe /esp32/reset, vẫn tiếp tục ghi.', flush=True)
                break
            self.ex.spin_once(timeout_sec=0.2)

        for _ in range(2):
            self.reset_pub.publish(Empty())
            self.spin_for(0.2)
        # bỏ các mẫu đang trên đường truyền trước khi ESP32 xử lý reset
        self.spin_for(0.3)

        self.rows.clear()
        self.t0_ns = None
        self.last_key_ns = None
        self.dup = 0
        self.recording = True
        print('Đã reset odom1 = odom2. ĐANG GHI ... (Ctrl+C để dừng và lưu kết quả)', flush=True)

    def print_progress(self):
        if not self.rows:
            if self.last_err_wall is None or time.monotonic() - self.last_err_wall > NO_DATA_WARN_S:
                print('Chưa nhận /esp32/odom_error (kiểm tra Bước 8: agent T3, LED ESP32, Gazebo T1)',
                      flush=True)
            else:
                print('0 mẫu | chờ /esp32/odom1 ...', flush=True)
            return
        r = self.rows[-1]
        print(f'{len(self.rows)} mẫu | t={r[0]:.1f}s | |e|={r[3]:.3f} m | e_yaw={r[4]:.2f}°',
              flush=True)


# ---------------------------------------------------------------------- report
def analyze_and_save(rows, out_base):
    if not rows:
        print('\nKhông có mẫu nào được ghi -> không lưu file.')
        return

    a = np.asarray(rows, dtype=float)
    t, ex, ey, e_pos, e_yaw = a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4]
    x2, y2, x1, y1 = a[:, 5], a[:, 6], a[:, 8], a[:, 9]
    n = len(a)

    rmse_pos = float(np.sqrt(np.mean(ex ** 2 + ey ** 2)))
    i_max = int(np.argmax(e_pos))
    e_final = float(e_pos[-1])
    dist2 = float(np.sum(np.hypot(np.diff(x2), np.diff(y2)))) if n > 1 else 0.0
    drift = (e_final / dist2 * 100.0) if dist2 > 1e-6 else float('nan')
    rmse_yaw = float(np.sqrt(np.mean(e_yaw ** 2)))
    i_ymax = int(np.argmax(np.abs(e_yaw)))

    # ---- bảng tổng hợp
    line = '=' * 58
    print('\n' + line)
    print('  KẾT QUẢ SAI SỐ ODOMETRY  (odom1 ESP32  vs  odom2 Gazebo)')
    print(line)
    print(f'  Số mẫu                  : {n}  (thời gian {t[-1]:.1f} s)')
    print(f'  Quãng đường odom2       : {dist2:.3f} m')
    print(f'  RMSE vị trí             : {rmse_pos:.4f} m')
    print(f'  Sai số vị trí max       : {e_pos[i_max]:.4f} m  (t = {t[i_max]:.1f} s)')
    print(f'  Sai số vị trí cuối      : {e_final:.4f} m')
    if math.isnan(drift):
        print('  Drift                   : N/A (robot chưa di chuyển)')
    else:
        print(f'  Drift                   : {drift:.2f} %  (sai số cuối / quãng đường)')
    print(f'  RMSE góc                : {rmse_yaw:.2f} °')
    print(f'  Max góc                 : {abs(e_yaw[i_ymax]):.2f} °  (t = {t[i_ymax]:.1f} s)')
    print(line)

    # ---- CSV
    csv_path = out_base + '.csv'
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        for r in rows:
            w.writerow([f'{r[0]:.3f}'] + [f'{v:.6f}' for v in r[1:]])

    # ---- PNG
    png_path = out_base + '.png'
    fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=(15, 6))

    ax_l.plot(x2, y2, color='tab:blue', lw=2.0, label='odom2 (Gazebo)')
    ax_l.plot(x1, y1, color='tab:orange', lw=1.6, ls='--', label='odom1 (ESP32)')
    ax_l.plot(x2[0], y2[0], 'o', color='black', ms=7, label='Bắt đầu')
    ax_l.plot(x2[-1], y2[-1], 's', color='tab:blue', ms=7, label='Kết thúc odom2')
    ax_l.plot(x1[-1], y1[-1], 's', color='tab:orange', ms=7, label='Kết thúc odom1')
    ax_l.set_title('Quỹ đạo odom2 vs odom1')
    ax_l.set_xlabel('x (m)')
    ax_l.set_ylabel('y (m)')
    ax_l.set_aspect('equal', adjustable='datalim')
    ax_l.grid(True, alpha=0.3)
    ax_l.legend(loc='best', fontsize=9)

    ax_r.plot(t, e_pos, color='tab:red', lw=1.5, label='Sai số vị trí |e| (m)')
    ax_r.axhline(rmse_pos, color='tab:red', ls='--', lw=1.0, label=f'RMSE vị trí = {rmse_pos:.4f} m')
    ax_r.plot(t[i_max], e_pos[i_max], 'o', color='tab:red', ms=8,
              label=f'Max = {e_pos[i_max]:.4f} m @ {t[i_max]:.1f} s')
    ax_r.set_xlabel('t (s, thời gian mô phỏng)')
    ax_r.set_ylabel('Sai số vị trí (m)', color='tab:red')
    ax_r.tick_params(axis='y', labelcolor='tab:red')
    ax_r.grid(True, alpha=0.3)

    ax_y = ax_r.twinx()
    ax_y.plot(t, e_yaw, color='tab:green', lw=1.2, label='Sai số góc e_yaw (°)')
    ax_y.axhline(rmse_yaw, color='tab:green', ls='--', lw=1.0, label=f'RMSE góc = {rmse_yaw:.2f}°')
    ax_y.plot(t[i_ymax], e_yaw[i_ymax], 'D', color='tab:green', ms=7,
              label=f'Max |góc| = {abs(e_yaw[i_ymax]):.2f}° @ {t[i_ymax]:.1f} s')
    ax_y.set_ylabel('Sai số góc (°)', color='tab:green')
    ax_y.tick_params(axis='y', labelcolor='tab:green')

    h1, l1 = ax_r.get_legend_handles_labels()
    h2, l2 = ax_y.get_legend_handles_labels()
    ax_r.legend(h1 + h2, l1 + l2, loc='upper left', fontsize=8)
    drift_txt = 'N/A' if math.isnan(drift) else f'{drift:.2f} %'
    ax_r.set_title(f'Sai số theo thời gian  (cuối {e_final:.4f} m, drift {drift_txt})')

    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)

    print(f'Đã lưu: {os.path.abspath(csv_path)}')
    print(f'Đã lưu: {os.path.abspath(png_path)}')


def parse_args():
    ap = argparse.ArgumentParser(description='Ghi sai số odom1 (ESP32) vs odom2 (Gazebo).')
    ap.add_argument('-o', '--output', default='odom_error',
                    help='tên file kết quả, không đuôi (mặc định: odom_error -> odom_error.csv/.png)')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])
    base, ext = os.path.splitext(args.output)
    if ext.lower() in ('.csv', '.png'):
        args.output = base
    return args


def main():
    args = parse_args()
    out_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(out_dir, exist_ok=True)

    init_rclpy()
    node = ErrorMonitor()
    try:
        node.reset_esp32()
        next_print = time.monotonic() + PROGRESS_PERIOD_S
        while rclpy.ok():
            node.ex.spin_once(timeout_sec=0.1)
            if time.monotonic() >= next_print:
                node.print_progress()
                next_print += PROGRESS_PERIOD_S
    except KeyboardInterrupt:
        pass
    finally:
        node.recording = False
        rows = list(node.rows)
        print(f'\nDừng ghi. Bỏ {node.dup} mẫu trùng timestamp.')
        try:
            analyze_and_save(rows, args.output)
        finally:
            node.ex.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
