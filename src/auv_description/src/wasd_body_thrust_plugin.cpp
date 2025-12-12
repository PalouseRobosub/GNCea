#include <memory>
#include <string>
#include <mutex>

#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32.hpp>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

using gz::sim::Model;
using gz::sim::Link;
using gz::sim::Entity;
using gz::sim::UpdateInfo;
using gz::sim::EntityComponentManager;

namespace auve1
{
class WasdBodyWrenchPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const Entity &entity,
                const std::shared_ptr<const sdf::Element> &sdf,
                EntityComponentManager &ecm,
                gz::sim::EventManager &) override
  {
    // --------------------------------------------------------------------------
    // 1. Initialize ROS 2 Node FIRST
    // --------------------------------------------------------------------------
    rclcpp::InitOptions opts;
    opts.shutdown_on_signal = true;
    if (!rclcpp::ok()) rclcpp::init(0, nullptr, opts);

    std::string temp_link = (sdf && sdf->HasElement("link_name")) ?
                            sdf->Get<std::string>("link_name") : "base_link";

    node_ = std::make_shared<rclcpp::Node>(("wasd_body_wrench_plugin_" + temp_link).c_str());
    clock_type_ = node_->get_clock()->get_clock_type();
    last_force_time_  = rclcpp::Time(0, 0, clock_type_);
    last_torque_time_ = rclcpp::Time(0, 0, clock_type_);

    // --------------------------------------------------------------------------
    // 2. Model + parameters
    // --------------------------------------------------------------------------
    model_ = Model(entity);
    if (!model_.Valid(ecm)) {
      RCLCPP_ERROR(node_->get_logger(), "Model invalid for entity %lu", entity);
      return;
    }

    link_name_ = temp_link;

    motor_speed_topic  = (sdf && sdf->HasElement("motor_speed_topic"))  ? sdf->Get<std::string>("motor_speed_topic")  : "/motor_speed";
    force_scale_  = (sdf && sdf->HasElement("force_scale"))  ? sdf->Get<double>("force_scale")  : 1.0;
    theta  = (sdf && sdf->HasElement("theta"))  ? sdf->Get<double>("theta")  : 0.0;
    phi = (sdf && sdf->HasElement("phi")) ? sdf->Get<double>("phi") : 0.0;

    
    if (sdf && sdf->HasElement("lever")) {
      lever_ = sdf->Get<gz::math::Vector3d>("lever");
      RCLCPP_INFO(node_->get_logger(), "Lever arm set to [%.3f, %.3f, %.3f] m", lever_.X(), lever_.Y(), lever_.Z());
    }

    // --------------------------------------------------------------------------
    // 3. Subscriptions
    // --------------------------------------------------------------------------
    sub_force_ = node_->create_subscription<std_msgs::msg::Float32>(
      motor_speed_topic, 10,
      [this](std_msgs::msg::Float32::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_);

        float speed = static_cast<float>(msg->data);

        if (abs(speed) < 0.01) speed = 0;

        double p = (90 - phi) * (M_PI / 180);
        double t = (90 + theta) * (M_PI / 180);

        double y = speed * sinf(p) * cosf(t);
        double x = speed * sinf(p) * sinf(t);
        double z = speed * cosf(p);

        RCLCPP_INFO(node_->get_logger(), "%s speed: <%.2f, %.2f, %.2f>", motor_speed_topic.c_str(), x, y, z);

        cmd_force_body_.Set(x * force_scale_, y * force_scale_, z * force_scale_);
        last_force_time_ = node_->now();
        have_force_ = true;
      });

    // --------------------------------------------------------------------------
    // 4. ROS Executor Thread
    // --------------------------------------------------------------------------
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);

    spinner_ = std::thread([this]{
      rclcpp::Rate r(300.0);
      while (rclcpp::ok()) {
        exec_->spin_some();
        r.sleep();
      }
    });

  }


  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override
  {
    if (info.paused) return;

    // Lazy link resolution
    if (linkEntity_ == gz::sim::kNullEntity) {
      auto e = model_.LinkByName(ecm, link_name_);
      if (e == gz::sim::kNullEntity) {
        if (node_) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "Waiting for link '%s' to appear...", link_name_.c_str());
        }
        return; // try again next tick
      }
      linkEntity_ = e;
      link_ = Link(linkEntity_);
      if (node_) RCLCPP_INFO(node_->get_logger(),
        "Resolved link '%s' (entity %lu).", link_name_.c_str(), linkEntity_);
    }


    gz::math::Vector3d f_body = cmd_force_body_; 

    auto pose = gz::sim::worldPose(linkEntity_, ecm);
    gz::math::Vector3d f_world = pose.Rot().RotateVector(f_body);
    gz::math::Vector3d tau_world = gz::math::Vector3d::Zero;

    gz::math::Vector3d r_world = pose.Rot().RotateVector(lever_);
    gz::math::Vector3d tau_lever = r_world.Cross(f_world);

    // Apply the combined wrench on this link
    link_.AddWorldWrench(ecm, f_world, tau_world + tau_lever);

    // link_.AddWorldForce(ecm, f_world);

    auto velComp = ecm.Component<gz::sim::components::LinearVelocity>(link_.Entity());
    auto velAComp = ecm.Component<gz::sim::components::AngularVelocity>(link_.Entity());
      RCLCPP_INFO(node_->get_logger(), "is_okay: %i, %i", velComp && 1, velAComp && 1);
    if (velComp && velAComp) {
      const gz::math::Vector3d &vel = velComp->Data();
      const gz::math::Vector3d &vela = velAComp->Data();
      gz::math::Vector3d df = vel.Abs() * vel * -100000;
      gz::math::Vector3d dft = vela.Abs() * vela * -100000;
      link_.AddWorldWrench(ecm, df, dft);

      RCLCPP_INFO(node_->get_logger(), "drag_force: <%.2f, %.2f, %.2f>\tdrag_angular: <%.2f, %.2f, %.2f>", df.X(), df.Y(), df.Z(), dft.X(), dft.Y(), dft.Z());

    }
  }

  ~WasdBodyWrenchPlugin() override
  {
    if (exec_) exec_->cancel();
    if (spinner_.joinable()) spinner_.join();
    if (rclcpp::ok()) rclcpp::shutdown();
  }

private:
  Model model_;
  std::string link_name_;
  std::string motor_speed_topic;
  double force_scale_{1.0};

  double theta{0.0};
  double phi{0.0};

  Entity linkEntity_{gz::sim::kNullEntity};
  Link link_{gz::sim::kNullEntity};

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_force_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spinner_;
  std::mutex mtx_;

  gz::math::Vector3d cmd_force_body_{0,0,0};
  gz::math::Vector3d cmd_torque_body_{0,0,0};
  gz::math::Vector3d lever_{0, 0, 0};

  rclcpp::Time last_force_time_;
  rclcpp::Time last_torque_time_;
  rcl_clock_type_t clock_type_;

  bool have_force_{false};
  bool have_torque_{false};
  bool velocity_enabled_{false};
  int dbg_counter_{0};
};
}  // namespace auve1

GZ_ADD_PLUGIN(auve1::WasdBodyWrenchPlugin,
              gz::sim::System,
              auve1::WasdBodyWrenchPlugin::ISystemConfigure,
              auve1::WasdBodyWrenchPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(auve1::WasdBodyWrenchPlugin, "auve1::WasdBodyWrenchPlugin")
GZ_ADD_PLUGIN_ALIAS(auve1::WasdBodyWrenchPlugin, "wasd_body_wrench_plugin")

// add this extra alias so your world/URDF can request it:
GZ_ADD_PLUGIN_ALIAS(auve1::WasdBodyWrenchPlugin, "wasd_body_thrust_plugin")
