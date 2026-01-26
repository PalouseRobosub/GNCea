#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class CmdBridge : public rclcpp::Node
{
public:
  CmdBridge()
  : Node("cmd_bridge")
  {
    // Declare and get parameters
    auto k_lin = declare_parameter("k_linear", 100.0);
    auto k_ang = declare_parameter("k_angular", 50.0);
    std::string ns = declare_parameter("namespace", "auve1");

    // Topic names
    std::string cmd_vel_topic = "/cmd_vel";
    std::string force_topic = "/" + ns + "/force_body";
    std::string torque_topic = "/" + ns + "/torque_body";

    // Publishers
    pub_force_ = create_publisher<geometry_msgs::msg::Vector3>(force_topic, 10);
    pub_torque_ = create_publisher<geometry_msgs::msg::Vector3>(torque_topic, 10);

    // Subscriber
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      [this, k_lin, k_ang](const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::Vector3 force, torque;

        // Linear force
        force.x = k_lin * msg->linear.x;
        force.y = k_lin * msg->linear.y;
        force.z = k_lin * msg->linear.z;

        // Angular torque
        torque.x = k_ang * msg->angular.x;
        torque.y = k_ang * msg->angular.y;
        torque.z = k_ang * msg->angular.z;

        // Publish
        pub_force_->publish(force);
        pub_torque_->publish(torque);
      });

    RCLCPP_INFO(get_logger(), "Bridging /cmd_vel → %s + %s", force_topic.c_str(), torque_topic.c_str());
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_force_, pub_torque_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdBridge>());
  rclcpp::shutdown();
  return 0;
}
