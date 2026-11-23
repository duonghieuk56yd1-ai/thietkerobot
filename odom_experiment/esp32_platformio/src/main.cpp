// =============================================================================
//  Thí nghiệm sai số Odometry: ESP32 (micro-ROS) vs Gazebo  —  ROS 2 Humble
//
//  odom1 : ESP32 tự tích phân /cmd_vel (v, w) theo thời gian MÔ PHỎNG
//          (dt lấy từ header.stamp của /odom, KHÔNG dùng millis()).
//  odom2 : /odom chuẩn của Gazebo (TurtleBot3 waffle, ~30 Hz).
//
//  Subscribers : /cmd_vel        geometry_msgs/msg/Twist
//                /odom           nav_msgs/msg/Odometry
//                /esp32/reset    std_msgs/msg/Empty
//  Publishers  : /esp32/odom_error  geometry_msgs/msg/Vector3  (20 Hz)
//                    x = ex = x1 - x2, y = ey = y1 - y2,
//                    z = e_yaw = normalize(yaw1 - yaw2)  [rad]
//                /esp32/odom1       nav_msgs/msg/Odometry      (20 Hz)
//
//  LED (GPIO 2): TẮT       = chưa kết nối micro_ros_agent
//                NHÁY NHANH = đã kết nối agent, chưa nhận /odom
//                SÁNG ĐỨNG  = đang nhận /odom ổn định
//
//  LƯU Ý: cổng Serial dành riêng cho micro-ROS -> KHÔNG dùng Serial.print().
// =============================================================================
#include <Arduino.h>
#include <math.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/empty.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error "Firmware này cần board_microros_transport = serial trong platformio.ini"
#endif

// ----------------------------- Thông số phải khớp ----------------------------
static const size_t   ROS_DOMAIN_ID = 77;       // = ROS_DOMAIN_ID trên laptop
static const uint32_t SERIAL_BAUD   = 921600;   // = tham số -b của micro_ros_agent
static const int      LED_PIN       = 2;        // LED onboard ESP32 DevKit

// ----------------------------- Thông số nội bộ -------------------------------
static const char *   NODE_NAME            = "esp32_odom_node";
static const uint32_t PUBLISH_PERIOD_MS    = 50;    // 20 Hz
static const uint32_t PING_PERIOD_MS       = 1000;  // kiểm tra agent khi đã kết nối
static const uint32_t WAIT_PING_PERIOD_MS  = 500;   // dò agent khi chưa kết nối
static const uint32_t LED_BLINK_MS         = 100;   // nháy nhanh 5 Hz
static const uint32_t ODOM_STALE_MS        = 500;   // quá thời gian này coi như mất /odom
static const double   MAX_DT_S             = 1.0;   // dt lớn hơn -> bỏ qua tích phân (khoảng trống dữ liệu)

// ----------------------------- Tiện ích --------------------------------------
#define RCCHECK(fn)                                   \
  do {                                                \
    rcl_ret_t temp_rc = (fn);                         \
    if (temp_rc != RCL_RET_OK) { return false; }      \
  } while (0)

#define RCSOFTCHECK(fn)                               \
  do {                                                \
    rcl_ret_t temp_rc = (fn);                         \
    (void)temp_rc;                                    \
  } while (0)

#define EXECUTE_EVERY_N_MS(MS, X)                     \
  do {                                                \
    static volatile int64_t init_ = -1;               \
    if (init_ == -1) { init_ = uxr_millis(); }        \
    if (uxr_millis() - init_ > (MS)) {                \
      X;                                              \
      init_ = uxr_millis();                           \
    }                                                 \
  } while (0)

static inline double normalize_angle(double a) { return atan2(sin(a), cos(a)); }

static inline double yaw_from_quat(double x, double y, double z, double w) {
  return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

static inline double stamp_to_sec(const builtin_interfaces__msg__Time & t) {
  return (double)t.sec + (double)t.nanosec * 1e-9;
}

// ----------------------------- State machine kết nối -------------------------
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState state = WAITING_AGENT;

// ----------------------------- Thực thể micro-ROS ----------------------------
static rcl_allocator_t     allocator;
static rcl_init_options_t  init_options;
static rclc_support_t      support;
static rcl_node_t          node;
static rcl_subscription_t  sub_cmd_vel;
static rcl_subscription_t  sub_odom;
static rcl_subscription_t  sub_reset;
static rcl_publisher_t     pub_error;
static rcl_publisher_t     pub_odom1;
static rcl_timer_t         timer;
static rclc_executor_t     executor;

// Cờ theo dõi thực thể nào đã tạo thành công (để hủy an toàn khi lỗi giữa chừng)
static bool init_options_ok = false, support_ok = false, node_ok = false;
static bool sub_cmd_ok = false, sub_odom_ok = false, sub_reset_ok = false;
static bool pub_error_ok = false, pub_odom1_ok = false, timer_ok = false, executor_ok = false;

// ----------------------------- Message buffers -------------------------------
static geometry_msgs__msg__Twist   msg_cmd_vel;
static nav_msgs__msg__Odometry     msg_odom_in;
static std_msgs__msg__Empty        msg_reset;
static geometry_msgs__msg__Vector3 msg_error;
static nav_msgs__msg__Odometry     msg_odom1;

// Bộ nhớ tĩnh cho chuỗi frame_id / child_frame_id của /odom nhận về
// (micro-ROS không cấp phát động; thiếu bộ nhớ này message sẽ bị bỏ).
static char odom_in_frame_buf[48];
static char odom_in_child_buf[48];
static char odom1_frame_buf[]  = "odom";
static char odom1_child_buf[]  = "base_footprint_esp32";

// ----------------------------- Trạng thái odometry ---------------------------
static double cmd_v = 0.0, cmd_w = 0.0;            // lệnh mới nhất
static double x1 = 0.0, y1_ = 0.0, th1 = 0.0;      // odom1 (ESP32 tích phân)
static double x2 = 0.0, y2 = 0.0, th2 = 0.0;       // odom2 (Gazebo)
static double last_stamp_s = 0.0;                  // stamp /odom trước đó (sim time)
static builtin_interfaces__msg__Time last_stamp;   // stamp /odom mới nhất
static bool     synced        = false;             // đã gán odom1 = odom2 lần đầu chưa
static bool     reset_pending = false;             // nhận /esp32/reset -> gán lại ở mẫu kế tiếp
static uint32_t last_odom_rx_ms = 0;               // chỉ dùng cho LED, không dùng tính dt

// ----------------------------- Callbacks -------------------------------------
static void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * m = (const geometry_msgs__msg__Twist *)msgin;
  cmd_v = m->linear.x;
  cmd_w = m->angular.z;
}

static void reset_callback(const void * msgin) {
  (void)msgin;
  reset_pending = true;
}

static void sync_odom1_to_odom2(double stamp_s) {
  x1 = x2;
  y1_ = y2;
  th1 = th2;
  last_stamp_s = stamp_s;
  synced = true;
  reset_pending = false;
}

static void odom_callback(const void * msgin) {
  const nav_msgs__msg__Odometry * m = (const nav_msgs__msg__Odometry *)msgin;

  x2 = m->pose.pose.position.x;
  y2 = m->pose.pose.position.y;
  th2 = yaw_from_quat(m->pose.pose.orientation.x, m->pose.pose.orientation.y,
                      m->pose.pose.orientation.z, m->pose.pose.orientation.w);
  last_stamp = m->header.stamp;
  last_odom_rx_ms = millis();
  const double stamp_s = stamp_to_sec(m->header.stamp);

  // Mẫu đầu tiên hoặc vừa có lệnh reset: odom1 = odom2 -> sai số = 0
  if (!synced || reset_pending) {
    sync_odom1_to_odom2(stamp_s);
    return;
  }

  const double dt = stamp_s - last_stamp_s;
  if (dt <= 0.0) {
    // Thời gian mô phỏng đi lùi (Gazebo khởi động lại) hoặc mẫu trùng stamp
    if (dt < 0.0) { sync_odom1_to_odom2(stamp_s); }
    return;
  }
  last_stamp_s = stamp_s;
  if (dt > MAX_DT_S) { return; }  // khoảng trống dữ liệu quá lớn: không tích phân bừa

  // Tích phân điểm giữa (midpoint)
  const double th_m = th1 + 0.5 * cmd_w * dt;
  x1  += cmd_v * cos(th_m) * dt;
  y1_ += cmd_v * sin(th_m) * dt;
  th1  = normalize_angle(th1 + cmd_w * dt);
}

static void timer_callback(rcl_timer_t * t, int64_t last_call_time) {
  (void)last_call_time;
  if (t == NULL || !synced) { return; }

  // /esp32/odom1
  msg_odom1.header.stamp = last_stamp;  // gắn stamp sim của mẫu odom2 đã dùng
  msg_odom1.pose.pose.position.x = x1;
  msg_odom1.pose.pose.position.y = y1_;
  msg_odom1.pose.pose.position.z = 0.0;
  msg_odom1.pose.pose.orientation.x = 0.0;
  msg_odom1.pose.pose.orientation.y = 0.0;
  msg_odom1.pose.pose.orientation.z = sin(0.5 * th1);
  msg_odom1.pose.pose.orientation.w = cos(0.5 * th1);
  msg_odom1.twist.twist.linear.x = cmd_v;
  msg_odom1.twist.twist.angular.z = cmd_w;
  RCSOFTCHECK(rcl_publish(&pub_odom1, &msg_odom1, NULL));

  // /esp32/odom_error (phát sau odom1 để phía PC nhận odom1 tương ứng trước)
  msg_error.x = x1 - x2;
  msg_error.y = y1_ - y2;
  msg_error.z = normalize_angle(th1 - th2);
  RCSOFTCHECK(rcl_publish(&pub_error, &msg_error, NULL));
}

// ----------------------------- Tạo / hủy thực thể ----------------------------
static void prepare_messages() {
  // Chuỗi của /odom nhận về: trỏ vào bộ nhớ tĩnh
  msg_odom_in.header.frame_id.data = odom_in_frame_buf;
  msg_odom_in.header.frame_id.size = 0;
  msg_odom_in.header.frame_id.capacity = sizeof(odom_in_frame_buf);
  msg_odom_in.child_frame_id.data = odom_in_child_buf;
  msg_odom_in.child_frame_id.size = 0;
  msg_odom_in.child_frame_id.capacity = sizeof(odom_in_child_buf);

  // Chuỗi của /esp32/odom1 phát đi
  msg_odom1.header.frame_id.data = odom1_frame_buf;
  msg_odom1.header.frame_id.size = strlen(odom1_frame_buf);
  msg_odom1.header.frame_id.capacity = sizeof(odom1_frame_buf);
  msg_odom1.child_frame_id.data = odom1_child_buf;
  msg_odom1.child_frame_id.size = strlen(odom1_child_buf);
  msg_odom1.child_frame_id.capacity = sizeof(odom1_child_buf);
}

static void destroy_entities() {
  if (support_ok) {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  }
  if (executor_ok)  { RCSOFTCHECK(rclc_executor_fini(&executor)); }
  if (timer_ok)     { RCSOFTCHECK(rcl_timer_fini(&timer)); }
  if (pub_odom1_ok) { RCSOFTCHECK(rcl_publisher_fini(&pub_odom1, &node)); }
  if (pub_error_ok) { RCSOFTCHECK(rcl_publisher_fini(&pub_error, &node)); }
  if (sub_reset_ok) { RCSOFTCHECK(rcl_subscription_fini(&sub_reset, &node)); }
  if (sub_odom_ok)  { RCSOFTCHECK(rcl_subscription_fini(&sub_odom, &node)); }
  if (sub_cmd_ok)   { RCSOFTCHECK(rcl_subscription_fini(&sub_cmd_vel, &node)); }
  if (node_ok)      { RCSOFTCHECK(rcl_node_fini(&node)); }
  if (support_ok)   { RCSOFTCHECK(rclc_support_fini(&support)); }
  if (init_options_ok) { RCSOFTCHECK(rcl_init_options_fini(&init_options)); }

  init_options_ok = support_ok = node_ok = false;
  sub_cmd_ok = sub_odom_ok = sub_reset_ok = false;
  pub_error_ok = pub_odom1_ok = timer_ok = executor_ok = false;
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  init_options_ok = true;
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  support_ok = true;

  node = rcl_get_zero_initialized_node();
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
  node_ok = true;

  // QoS reliable (mặc định): bắt buộc cho Odometry (~720 byte > MTU 512)
  // vì chỉ stream reliable mới hỗ trợ phân mảnh.
  sub_cmd_vel = rcl_get_zero_initialized_subscription();
  RCCHECK(rclc_subscription_init_default(
      &sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
  sub_cmd_ok = true;

  sub_odom = rcl_get_zero_initialized_subscription();
  RCCHECK(rclc_subscription_init_default(
      &sub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"));
  sub_odom_ok = true;

  sub_reset = rcl_get_zero_initialized_subscription();
  RCCHECK(rclc_subscription_init_default(
      &sub_reset, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "/esp32/reset"));
  sub_reset_ok = true;

  pub_error = rcl_get_zero_initialized_publisher();
  RCCHECK(rclc_publisher_init_default(
      &pub_error, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/esp32/odom_error"));
  pub_error_ok = true;

  pub_odom1 = rcl_get_zero_initialized_publisher();
  RCCHECK(rclc_publisher_init_default(
      &pub_odom1, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/esp32/odom1"));
  pub_odom1_ok = true;

  timer = rcl_get_zero_initialized_timer();
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), timer_callback));
  timer_ok = true;

  // 3 subscription + 1 timer = 4 handle
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  executor_ok = true;
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_odom, &msg_odom_in, &odom_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_reset, &msg_reset, &reset_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Kết nối mới -> đồng bộ lại odom1 = odom2 ở mẫu /odom đầu tiên
  synced = false;
  reset_pending = false;
  cmd_v = 0.0;
  cmd_w = 0.0;
  last_odom_rx_ms = 0;
  return true;
}

// ----------------------------- LED -------------------------------------------
static void update_led() {
  static uint32_t last_toggle_ms = 0;
  static bool blink_level = false;
  const uint32_t now = millis();

  if (state != AGENT_CONNECTED) {
    digitalWrite(LED_PIN, LOW);                       // chưa kết nối agent
    return;
  }
  const bool odom_ok = synced && last_odom_rx_ms != 0 && (now - last_odom_rx_ms) < ODOM_STALE_MS;
  if (odom_ok) {
    digitalWrite(LED_PIN, HIGH);                      // đang nhận /odom ổn định
    return;
  }
  if (now - last_toggle_ms >= LED_BLINK_MS) {         // đã kết nối, chưa có /odom
    last_toggle_ms = now;
    blink_level = !blink_level;
    digitalWrite(LED_PIN, blink_level ? HIGH : LOW);
  }
}

// ----------------------------- Arduino ---------------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  set_microros_serial_transports(Serial);
  delay(500);

  prepare_messages();
  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(WAIT_PING_PERIOD_MS,
                         state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        state = AGENT_CONNECTED;
      } else {
        destroy_entities();
        state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(PING_PERIOD_MS,
                         state = (RMW_RET_OK == rmw_uros_ping_agent(100, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
  update_led();
}
