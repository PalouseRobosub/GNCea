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
    opts.shutdown_on_signal = false;
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
    force_topic_  = (sdf && sdf->HasElement("force_topic"))  ? sdf->Get<std::string>("force_topic")  : "/auve1/force_body";
    motor_speed_topic  = (sdf && sdf->HasElement("motor_speed_topic"))  ? sdf->Get<std::string>("motor_speed_topic")  : "/motor_speed";
    torque_topic_ = (sdf && sdf->HasElement("torque_topic")) ? sdf->Get<std::string>("torque_topic") : "/auve1/torque_body";
    hold_ms_ = (sdf && sdf->HasElement("hold_ms")) ? sdf->Get<int>("hold_ms") : -1;
    force_scale_  = (sdf && sdf->HasElement("force_scale"))  ? sdf->Get<double>("force_scale")  : 1.0;
    torque_scale_ = (sdf && sdf->HasElement("torque_scale")) ? sdf->Get<double>("torque_scale") : 1.0;

    theta  = (sdf && sdf->HasElement("theta"))  ? sdf->Get<double>("theta")  : 0.0;
    phi = (sdf && sdf->HasElement("phi")) ? sdf->Get<double>("phi") : 0.0;

    
    if (sdf && sdf->HasElement("lever")) {
      lever_ = sdf->Get<gz::math::Vector3d>("lever");
      RCLCPP_INFO(node_->get_logger(),
                  "Lever arm set to [%.3f, %.3f, %.3f] m",
                  lever_.X(), lever_.Y(), lever_.Z());
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

        if (speed < 0.1) speed = 0;

        double p = (90 - phi) * (M_PI / 180);
        double t = (90 + theta) * (M_PI / 180);

        double x = speed * sinf(p) * cosf(t);
        double y = speed * sinf(p) * sinf(t);
        double z = speed * cosf(p);

        cmd_force_body_.Set(x * force_scale_, y * force_scale_, z * force_scale_);
        last_force_time_ = node_->now();
        have_force_ = true;
      });

    // sub_torque_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
    //   torque_topic_, 10,
    //   [this](geometry_msgs::msg::Vector3::SharedPtr msg)
    //   {
    //     std::lock_guard<std::mutex> lock(mtx_);
    //     cmd_torque_body_.Set(msg->x * torque_scale_, msg->y * torque_scale_, msg->z * torque_scale_);
    //     last_torque_time_ = node_->now();
    //     have_torque_ = true;
    //   });

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

    RCLCPP_INFO(node_->get_logger(),
      "WasdBodyWrenchPlugin loaded for link='%s' (topics: F='%s', T='%s', hold_ms=%d)",
      link_name_.c_str(), force_topic_.c_str(), torque_topic_.c_str(), hold_ms_);
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

    

    if (!velocity_enabled_)
    {
      link_.EnableVelocityChecks(ecm, true);
      velocity_enabled_ = true;
    }

    gz::math::Vector3d f_body(0,0,0), tau_body(0,0,0);
    bool use_force = false, use_torque = false;

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (have_force_) {
        if (hold_ms_ < 0) { use_force = true; f_body = cmd_force_body_; }
        else {
          auto now = rclcpp::Time(node_->now().nanoseconds(), clock_type_);
          if ((now - last_force_time_) < rclcpp::Duration::from_seconds(hold_ms_ / 1000.0))
            { use_force = true; f_body = cmd_force_body_; }
        }
      }
      if (have_torque_) {
        if (hold_ms_ < 0) { use_torque = true; tau_body = cmd_torque_body_; }
        else {
          auto now = rclcpp::Time(node_->now().nanoseconds(), clock_type_);
          if ((now - last_torque_time_) < rclcpp::Duration::from_seconds(hold_ms_ / 1000.0))
            { use_torque = true; tau_body = cmd_torque_body_; }
        }
      }
    }

    if (!use_force && !use_torque) return;

    auto pose = gz::sim::worldPose(linkEntity_, ecm);
    gz::math::Vector3d f_world   = use_force  ? pose.Rot().RotateVector(f_body)   : gz::math::Vector3d::Zero;
    gz::math::Vector3d tau_world = use_torque ? pose.Rot().RotateVector(tau_body) : gz::math::Vector3d::Zero;

    // Apply both force and torque in world frame at CoM
    // link_.AddWorldWrench(ecm, f_world, tau_world);
    

  // --- Lever-arm torque τ = r × F -------------------------------------------
  // lever_ is given in the link (base) frame
    gz::math::Vector3d r_world = pose.Rot().RotateVector(lever_);
    gz::math::Vector3d tau_lever = r_world.Cross(f_world);

    // Apply the combined wrench on this link
    link_.AddWorldWrench(ecm, f_world, tau_world + tau_lever);

    auto velComp = ecm.Component<gz::sim::components::LinearVelocity>(link_.Entity());
    auto velAComp = ecm.Component<gz::sim::components::AngularVelocity>(link_.Entity());
    if (velComp && velAComp) {
      const gz::math::Vector3d &vel = velComp->Data();
      const gz::math::Vector3d &vela = velAComp->Data();
      gz::math::Vector3d df = vel.Abs() * vel * -100;
      gz::math::Vector3d dft = vela.Abs() * vela * -100;
      link_.AddWorldWrench(ecm, df, dft);

      RCLCPP_INFO(node_->get_logger(), "drag_force: <%.2f, %.2f, %.2f>", df.X(), df.Y(), df.Z());

    }

    // gz::math::Vector3d vel = link_.WorldLinearVelocity();
    // gz::math::Vector3d d = -5 * vel;
    // RCLCPP_INFO(node_->get_logger(), "DAMPING: (%.2f,%.2f,%.2f)", d.X(), d.Y(), d.Z());
    // --------------------------------------------------------------------------

    // Debug (every 60 cycles)
    if (++dbg_counter_ % 60 == 0 && node_) {
      RCLCPP_INFO(node_->get_logger(),
        "[%s] F=(%.2f,%.2f,%.2f) τ_total=(%.2f,%.2f,%.2f) τ_lever=(%.2f,%.2f,%.2f)",
        link_name_.c_str(),
        f_world.X(), f_world.Y(), f_world.Z(),
        (tau_world + tau_lever).X(), (tau_world + tau_lever).Y(), (tau_world + tau_lever).Z(),
        tau_lever.X(), tau_lever.Y(), tau_lever.Z());
    }    

    // // (Optional) throttle debug
    // if (++dbg_counter_ % 60 == 0 && node_) {
    //   RCLCPP_INFO(node_->get_logger(),
    //     "[WRENCH] F[%.1f %.1f %.1f]  Tau[%.1f %.1f %.1f]",
    //     f_world.X(), f_world.Y(), f_world.Z(), tau_world.X(), tau_world.Y(), tau_world.Z());
    // }
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
  std::string force_topic_;
  std::string motor_speed_topic;
  std::string torque_topic_;
  int    hold_ms_{-1};
  double force_scale_{1.0};
  double torque_scale_{1.0};

  double theta{0.0};
  double phi{0.0};

  Entity linkEntity_{gz::sim::kNullEntity};
  Link link_{gz::sim::kNullEntity};

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_force_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_torque_;
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
