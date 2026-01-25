#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cerrno>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

using geometry_msgs::msg::Vector3;

struct TwistLite
{
  double lx{0}, ly{0}, lz{0};
  double ax{0}, ay{0}, az{0};
};

class KeyboardAuvTeleop : public rclcpp::Node
{
public:
  KeyboardAuvTeleop()
  : Node("keyboard_gncea_teleop")
  {
    // -------- Parameters ----------
    force_topic_   = declare_parameter<std::string>("force_topic",  "/auve1/force_body");
    torque_topic_  = declare_parameter<std::string>("torque_topic", "/auve1/torque_body");
    linear_speed_  = declare_parameter<double>("linear_speed",  1.0);   // m/s (equivalent)
    angular_speed_ = declare_parameter<double>("angular_speed", 1.0);   // rad/s (equivalent)
    rate_hz_       = std::max(1.0, declare_parameter<double>("rate_hz", 200.0));       // publish rate
    key_poll_hz_   = std::max(50.0, declare_parameter<double>("key_poll_hz", 400.0));  // keyboard poll rate
    k_linear_      = declare_parameter<double>("k_linear",  200.0); // N per (m/s)
    k_angular_     = declare_parameter<double>("k_angular",  100.0); // N·m per (rad/s)

    // -------- QoS (low latency) ----------
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.durability_volatile();

    pub_force_ = create_publisher<Vector3>(force_topic_, qos);
    pub_torque_ = create_publisher<Vector3>(torque_topic_, qos);

    RCLCPP_INFO(get_logger(), "force → \"%s\", torque → \"%s\"", force_topic_.c_str(), torque_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Keys: W/S ±X, A/D ±Y, E/Q ±Z, I/K ±Pitch, J/L ±Yaw, U/O ±Roll, Space zero");

    // -------- Timers ----------
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_),
      std::bind(&KeyboardAuvTeleop::OnPublishTimer, this));

    // -------- Keyboard thread (/dev/tty fallback) ----------
    start_keyboard_thread();
  }

  ~KeyboardAuvTeleop() override
  {
    stop_keyboard_thread();
  }

private:
  // ---------- Keyboard ----------
  void start_keyboard_thread()
  {
    // Prefer stdin if TTY; else open /dev/tty
    fd_ = isatty(STDIN_FILENO) ? STDIN_FILENO : open("/dev/tty", O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open TTY for keyboard input: %s", strerror(errno));
      return;
    }

    // Save and set raw mode
    if (tcgetattr(fd_, &orig_term_) == 0) {
      termios raw = orig_term_;
      cfmakeraw(&raw);
      tcsetattr(fd_, TCSANOW, &raw);
      raw_ok_ = true;
    } else {
      RCLCPP_WARN(get_logger(), "tcgetattr failed; continuing without raw mode.");
    }

    running_.store(true);
    key_thread_ = std::thread([this]{
      const double period = 1.0 / key_poll_hz_;
      timeval tv{};
      while (running_.load()) {
        fd_set rfds; FD_ZERO(&rfds); FD_SET(fd_, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = static_cast<suseconds_t>(period * 1e6);
        int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(fd_, &rfds)) {
          char c;
          ssize_t n = read(fd_, &c, 1);
          if (n == 1) {
            if (c == 0x03) { // Ctrl-C
              rclcpp::shutdown();
              break;
            }
            handle_key(c);
          }
        }
        // if ret==0 (timeout) or ret<0, just loop again
      }
    });

    RCLCPP_INFO(get_logger(), "Reading keys from: %s", (fd_ == STDIN_FILENO) ? "stdin" : "/dev/tty");
  }

  void stop_keyboard_thread()
  {
    running_.store(false);
    if (key_thread_.joinable()) key_thread_.join();

    if (raw_ok_) {
      tcsetattr(fd_, TCSANOW, &orig_term_);
    }
    if (fd_ != STDIN_FILENO && fd_ >= 0) close(fd_);
    fd_ = -1;
  }

  void handle_key(char ch)
  {
    ch = static_cast<char>(::tolower(static_cast<unsigned char>(ch)));
    bool updated = true;
    std::lock_guard<std::mutex> lk(cmd_mtx_);

    // Translation
    if      (ch == 'w') { cmd_.lx =  linear_speed_; cmd_.ly = 0.0; }
    else if (ch == 's') { cmd_.lx = -linear_speed_; cmd_.ly = 0.0; }
    else if (ch == 'a') { cmd_.ly =  linear_speed_; cmd_.lx = 0.0; }
    else if (ch == 'd') { cmd_.ly = -linear_speed_; cmd_.lx = 0.0; }
    else if (ch == 'e') { cmd_.lz =  linear_speed_; }
    else if (ch == 'q') { cmd_.lz = -linear_speed_; }
    // Rotation (roll=x, pitch=y, yaw=z)
    else if (ch == 'u') { cmd_.ax =  angular_speed_; }
    else if (ch == 'o') { cmd_.ax = -angular_speed_; }
    else if (ch == 'i') { cmd_.ay =  angular_speed_; }
    else if (ch == 'k') { cmd_.ay = -angular_speed_; }
    else if (ch == 'j') { cmd_.az =  angular_speed_; }
    else if (ch == 'l') { cmd_.az = -angular_speed_; }
    else if (ch == ' ') { zero_cmd_locked(); }
    else { updated = false; }

    if (updated) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500 /*ms*/,
        "lin=(%.2f,%.2f,%.2f) ang=(%.2f,%.2f,%.2f)",
        cmd_.lx, cmd_.ly, cmd_.lz, cmd_.ax, cmd_.ay, cmd_.az);
    }
  }

  void zero_cmd_locked()
  {
    cmd_.lx = cmd_.ly = cmd_.lz = 0.0;
    cmd_.ax = cmd_.ay = cmd_.az = 0.0;
  }

  // ---------- Publish ----------
  void OnPublishTimer()
  {
    TwistLite copy;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      copy = cmd_;
    }

    Vector3 f, tau;
    f.x   = k_linear_  * copy.lx;
    f.y   = k_linear_  * copy.ly;
    f.z   = k_linear_  * copy.lz;
    tau.x = k_angular_ * copy.ax;
    tau.y = k_angular_ * copy.ay;
    tau.z = k_angular_ * copy.az;

    pub_force_->publish(f);
    pub_torque_->publish(tau);
  }

private:
  // Parameters
  std::string force_topic_, torque_topic_;
  double linear_speed_{1.0}, angular_speed_{1.0};
  double rate_hz_{200.0}, key_poll_hz_{400.0};
  double k_linear_{100.0}, k_angular_{50.0};

  // Publishers
  rclcpp::Publisher<Vector3>::SharedPtr pub_force_;
  rclcpp::Publisher<Vector3>::SharedPtr pub_torque_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Command state
  TwistLite cmd_;
  std::mutex cmd_mtx_;

  // Keyboard thread
  std::thread key_thread_;
  std::atomic<bool> running_{false};
  int fd_{-1};
  termios orig_term_{};
  bool raw_ok_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardAuvTeleop>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  try {
    exec.spin();
  } catch (...) {
    // fallthrough to shutdown
  }
  rclcpp::shutdown();
  return 0;
}
