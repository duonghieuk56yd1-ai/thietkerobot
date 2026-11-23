# thietkerobot — AMR vận chuyển hàng trong kho (ROS 2 Humble + Gazebo Harmonic)

Workspace `ros2_ws/` mô phỏng độc lập robot AMR (đế Differential Drive + cơ cấu nâng 4 trụ + khay băng tải)
trên **Gazebo Harmonic (`gz-sim` 8)** và **RViz2**.

```
ros2_ws/src
├── amr_description/            # Mô tả robot
│   ├── urdf/amr.urdf.xacro     # Hình học, khớp, khối lượng (file chính)
│   ├── urdf/amr.gazebo.xacro   # Plugin gz-sim + cảm biến (được include khi use_gazebo:=true)
│   ├── urdf/amr_macros.xacro   # Macro quán tính + bảng màu
│   ├── launch/display.launch.py, rsp.launch.py
│   └── rviz/display.rviz
└── amr_gazebo/                 # Mô phỏng
    ├── launch/amr_gazebo.launch.py
    ├── worlds/warehouse.sdf, empty.sdf
    ├── models/cargo_box, warehouse_rack, handover_station, charging_dock
    ├── config/ros_gz_bridge.yaml
    ├── rviz/amr_gazebo.rviz
    └── scripts/amr_cmd.py      # Lệnh nhanh: nâng/hạ khay, chạy băng tải
```

## 1. Cài đặt

ROS 2 Humble mặc định đi với Gazebo Fortress; để dùng **Harmonic** cần bản `ros_gz` build cho Harmonic
(gói `ros-humble-ros-gzharmonic` từ kho `packages.osrfoundation.org`):

```bash
sudo apt install gz-harmonic ros-humble-ros-gzharmonic \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-rviz2
```

```bash
cd ros2_ws
colcon build --symlink-install
```

```bash
source install/setup.bash
```

## 2. Chạy

Chỉ xem mô hình trên RViz2 (kéo thử bánh xe / trụ nâng bằng thanh trượt):

```bash
ros2 launch amr_description display.launch.py
```

Mô phỏng Gazebo Harmonic + RViz2 (world nhà kho, thùng hàng 20 kg đặt sẵn trên khay):

```bash
ros2 launch amr_gazebo amr_gazebo.launch.py
```

Tham số launch: `world:=warehouse.sdf|empty.sdf`, `rviz:=true|false`, `headless:=true`, `x y z yaw`,
`spawn_cargo:=true|false`, `cargo_x cargo_y cargo_z cargo_yaw` (mặc định thùng hàng nằm trên khay).
Đặt thùng hàng dưới sàn, phía trước robot 1.5 m:

```bash
ros2 launch amr_gazebo amr_gazebo.launch.py cargo_x:=1.5 cargo_z:=0.21
```

### Điều khiển

Lái robot (giới hạn 0.5 m/s, 1.0 m/s², 2.0 rad/s²):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Nâng / hạ khay (4 trụ đồng bộ, 0.1 m/s, hành trình 0 → 0.25 m):

```bash
ros2 run amr_gazebo amr_cmd.py lift up
```

```bash
ros2 run amr_gazebo amr_cmd.py lift down
```

hoặc trực tiếp:

```bash
ros2 topic pub --once /lift_cmd std_msgs/msg/Float64 "{data: 0.25}"
```

Chạy băng tải tháo dỡ hàng 0.2 m/s trong 3 s (dương = đẩy hàng ra phía trước robot):

```bash
ros2 run amr_gazebo amr_cmd.py conveyor 0.2 3
```

Kịch bản trao hàng trong `warehouse.sdf`: robot xuất phát tại gốc, hướng +x; trạm trao hàng có mặt con lăn
cao 0.67 m đặt tại x = 4.6 m. Lái thẳng tới khi tâm robot ở x ≈ 3.97 m (vạch vàng trên sàn),
`lift up` (mặt băng lên 0.67 m), rồi `conveyor 0.2 4` để đẩy thùng hàng sang trạm.

## 3. Thông số kỹ thuật đã hiện thực

| Hạng mục | Giá trị trong mô hình |
|---|---|
| Thân đế | Trụ Ø0.60 m × 0.22 m, khoảng sáng gầm 0.06 m, nắp đế ở z = 0.28 m; viền cam mép trên, mảng tiếp điểm sạc màu đồng ở mặt trước (z 0.08–0.16 m, rộng 0.40 m), mũi tên chỉ hướng trên nắp |
| Bánh chủ động | Ø0.165 m (6.5"), rộng 0.04 m, wheel separation 0.44 m, khớp `continuous` |
| Bánh tự lựa | Ø0.08 m tại x = ±0.20 m, µ = 0.01 |
| LiDAR | Trụ Ø0.10 × 0.08 m giữa nắp đế, `laser_frame` ở z = 0.32 m (nằm giữa nắp đế 0.28 m và đáy khay 0.38 m) |
| Khối lượng | Đế 23.0 kg (thân 18.4 + bánh 2×1.5 + bánh tự lựa 2×0.3 + LiDAR 0.9 + IMU 0.1); cơ cấu nâng 7.0 kg (xi lanh 4×0.3 + cần 4×0.25 + khay 4.0 + mặt băng 0.8) |
| Trụ nâng | 4 khớp `prismatic` tại (±0.18, ±0.18) m, hành trình 0–0.25 m, 0.1 m/s, lực 1500 N |
| Khay băng tải | 0.52 × 0.52 m, 2 gờ chặn cao 0.05 m so với mặt băng, mặt băng cao su rộng 0.44 m; mặt băng z = 0.42 m (hạ) → 0.67 m (nâng) |
| Thùng hàng | Lập phương 0.40 m, 20 kg, µ = 0.8 (`models/cargo_box`) |

### Plugin gz-sim và topic

| Plugin | Cấu hình | Topic ROS 2 (qua `ros_gz_bridge`) |
|---|---|---|
| `DiffDrive` | odom 50 Hz, v ≤ 0.5 m/s, a ≤ 1.0 m/s², α ≤ 2.0 rad/s² | `/cmd_vel` (vào), `/odom`, `/tf` (`odom → base_footprint`) |
| `JointPositionController` | 1 controller cho cả 4 khớp nâng, `use_velocity_commands` với `cmd_max` = 0.1 m/s | `/lift_cmd` (`std_msgs/Float64`, m) |
| `gpu_lidar` | 360 tia (1°), 0.3–12.0 m, 10 Hz, nhiễu Gauss σ = 0.01 m | `/scan` |
| `imu` + `magnetometer` | 100 Hz, có nhiễu → IMU 9 trục | `/imu`, `/imu/mag` |
| `JointStatePublisher` | 2 bánh + 4 trụ nâng | `/joint_states` |
| `TrackController` (thêm) | Mặt băng tải, ±0.3 m/s | `/conveyor_cmd` (`std_msgs/Float64`, m/s), `/conveyor_odom` |

Ngoài ra `/clock` được bridge để mọi node dùng `use_sim_time`.

## 4. Ghi chú thiết kế

- **Khay nâng trong cây URDF**: URDF không cho phép vòng kín, nên khay được gắn cứng vào trụ trước-trái
  (`lift_fl_link`). Cả 4 khớp nâng do **một** `JointPositionController` điều khiển, nên luôn nhận cùng một
  lệnh vận tốc và chuyển động đồng bộ.
- **Bánh tự lựa cao hơn sàn 1 mm**: tránh hệ siêu tĩnh 4 điểm tiếp xúc (bánh chủ động có thể bị "treo" và
  mất bám). Độ nghiêng tối đa khi tì lên bánh tự lựa chỉ ≈ 0.3°.
- **LiDAR không bị che**: các cần piston cách tâm 0.255 m, nhỏ hơn tầm đo tối thiểu 0.3 m. Với `gpu_lidar`,
  tầm đo tối thiểu là mặt phẳng cắt gần, nên vật thể trong bán kính này không được render và không chắn tia.
- **Băng tải**: `conveyor_belt_link` được giữ là link riêng (`preserveFixedJoint`) để `TrackController`
  điều khiển vận tốc bề mặt tiếp xúc. Trạm trao hàng dùng ma sát thấp (µ = 0.05) để mô phỏng con lăn tự do.
- Màu sắc lấy từ `<material>` trong URDF, hiển thị trên cả RViz2 và Gazebo.
- Khi chạy world trực tiếp bằng `gz sim` (không qua launch), cần thêm
  `amr_gazebo/models` vào `GZ_SIM_RESOURCE_PATH`.

---

## Thí nghiệm sai số Odometry ESP32 (micro-ROS) vs Gazebo

Thư mục [`odom_experiment/`](odom_experiment/README.md): firmware ESP32 PlatformIO (`esp32_platformio/`)
và script Python ROS 2 Humble (`odom_exp/monitor_error.py`, `odom_exp/nav_waypoints.py`).
Xem hướng dẫn chạy 5 terminal trong [`odom_experiment/README.md`](odom_experiment/README.md).
