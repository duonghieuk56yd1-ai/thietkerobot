# Thí nghiệm sai số Odometry: ESP32 (micro-ROS) vs Gazebo — ROS 2 Humble

ESP32 tự tích phân `/cmd_vel` thành **odom1** (dt theo `header.stamp` của `/odom`, công thức điểm giữa),
so với **odom2** = `/odom` của Gazebo (TurtleBot3 waffle) khi Nav2 lái robot. Kết quả gồm RMSE, sai số max và drift.

```
odom_experiment/
├── esp32_platformio/          # Firmware ESP32 (PlatformIO + micro-ROS Serial 921600)
│   ├── platformio.ini
│   └── src/main.cpp
└── odom_exp/                  # Script Python ROS 2 Humble (copy vào ~/odom_exp)
    ├── monitor_error.py       # reset ESP32, ghi sai số, xuất CSV + PNG, in RMSE
    └── nav_waypoints.py       # 5 goal Nav2: vòng vuông quanh trụ giữa rồi về Start
```

## Thông số phải khớp

| Hằng số (`main.cpp`) | Giá trị | Phải khớp với |
|---|---|---|
| `ROS_DOMAIN_ID` | 77 | `export ROS_DOMAIN_ID=77` trên laptop |
| `SERIAL_BAUD` | 921600 | `-b 921600` của micro_ros_agent |
| `LED_PIN` | 2 | LED onboard |

Topic: ESP32 nghe `/cmd_vel`, `/odom`, `/esp32/reset` (`std_msgs/Empty`);
phát 20 Hz `/esp32/odom_error` (`Vector3`: x = ex, y = ey, z = e_yaw rad) và `/esp32/odom1` (`Odometry`).

LED GPIO2: **tắt** = chưa có agent · **nháy nhanh** = có agent, chưa có `/odom` · **sáng đứng** = đang chạy đúng.

## Chuẩn bị (một lần)

```bash
sed -i '/ROS_DOMAIN_ID/d;/ROS_LOCALHOST_ONLY/d;/TURTLEBOT3_MODEL/d' ~/.bashrc
cat >> ~/.bashrc <<'EOF'
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=0
export TURTLEBOT3_MODEL=waffle
EOF
source ~/.bashrc
```

```bash
sudo apt install -y ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2 ros-humble-nav2-bringup python3-matplotlib
```

```bash
sudo usermod -aG dialout $USER     # rồi log out / log in
```

```bash
mkdir -p ~/odom_exp && cp odom_exp/*.py ~/odom_exp/
```

Nạp firmware (đóng Serial Monitor trước; lần đầu build thư viện micro-ROS mất vài phút):

```bash
cd esp32_platformio && pio run -t upload
```

Cổng Serial dành riêng cho micro-ROS — **không thêm `Serial.print`** vào firmware.

## Chạy trên 5 terminal

**T1 — Gazebo** (để chạy, không đóng)

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**T2 — Nav2 + RViz** (để chạy, không đóng)

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/maps/turtlebot3_world.yaml
```

Đặt vị trí ban đầu (chạy ở T4, hoặc dùng *2D Pose Estimate* trên RViz):

```bash
ros2 topic pub -r 1 -t 3 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0.0,0.0,0.0,0.0,0.0, 0.0,0.25,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0685]}}"
```

**T3 — micro-ROS Agent** (để chạy; chờ LED sáng đứng)

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
```

Log đúng: `establish_session` → `create_participant` → 2 × `create_datawriter`, 3 × `create_datareader`.
Không thấy gì sau 5 s: nhấn nút EN trên ESP32. Kiểm tra nhanh:

```bash
ros2 topic hz /esp32/odom_error      # ~20 Hz
```

**T4 — Ghi sai số** (bắt đầu TRƯỚC khi robot chạy)

```bash
cd ~/odom_exp && python3 monitor_error.py -o run1
```

In `Đã reset odom1 = odom2. ĐANG GHI ...`, sau đó cứ 5 s một dòng `120 mẫu | t=6.0s | |e|=0.000 m | e_yaw=0.00°`.

**T5 — Cho robot chạy quỹ đạo**

```bash
cd ~/odom_exp && python3 nav_waypoints.py --loops 2
```

In `[vòng 1/2] WP1/5 -> (0.55, -0.55, 90°) ...` … và cuối cùng `XONG tất cả waypoint`.
Ctrl+C giữa chừng sẽ hủy goal đang chạy.

**Kết thúc:** khi T5 in `XONG`, bấm **Ctrl+C ở T4** → in bảng KẾT QUẢ (RMSE vị trí, max + thời điểm,
sai số cuối, drift %, RMSE/max góc) và lưu `run1.csv`, `run1.png`.

Lần đo tiếp: lặp T4 → T5 với `-o run2`, `-o run3`, … (không cần khởi động lại Gazebo hay ESP32).

## Kết quả

- CSV: `t_s, ex_m, ey_m, e_pos_m, e_yaw_deg, x2, y2, yaw2, x1, y1, yaw1` (t theo thời gian mô phỏng; yaw tính bằng rad).
- PNG: trái = quỹ đạo odom2 (xanh) vs odom1 (cam); phải = |e| và e_yaw theo thời gian, đường RMSE và điểm max.

## Lỗi thường gặp

| Hiện tượng | Xử lý |
|---|---|
| Agent không in gì | sai cổng/baud, Serial Monitor đang mở, thiếu quyền dialout; nhấn EN |
| Agent có `create_datawriter` nhưng không thấy topic esp32 | domain trong `main.cpp` ≠ `ROS_DOMAIN_ID`, hoặc `ROS_LOCALHOST_ONLY=1` |
| LED nháy mãi | ESP32 không nhận `/odom`: chạy Gazebo (T1), agent phải dùng `-b 921600` |
| `/esp32/odom_error` < 15 Hz | cáp USB kém; hoặc giảm baud cả `SERIAL_BAUD` lẫn `-b` xuống 460800 |
| `Chờ ESP32 subscribe /esp32/reset` lặp mãi | ESP32 mất kết nối agent: xem T3 và LED |
| Goal báo `ABORTED` | đặt lại initialpose (T2) rồi chạy lại |
