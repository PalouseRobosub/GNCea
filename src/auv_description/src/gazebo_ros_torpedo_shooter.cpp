#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/transport/Node.hh>

#include <gz/sim/components/LinearVelocity.hh>
#include <gz/math/Pose3.hh>
#include <fstream>
#include <sstream>
#include <thread>
#include <memory>

namespace auv_sim
{

class TorpedoShooterPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemUpdate
{
public:
  void Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &eventMgr) override
  {
    this->entity = entity;
    this->model = gz::sim::Model(entity);

    // Get world name for transport topic
    auto worldEntity = gz::sim::worldEntity(ecm);
    gz::sim::World world(worldEntity);
    this->worldName = world.Name(ecm).value_or("default");

    // Get parameters from SDF
    if (sdf->HasElement("torpedo_sdf_path"))
      this->torpedoSDFPath = sdf->Get<std::string>("torpedo_sdf_path");

    if (sdf->HasElement("launcher_link"))
      this->launcherLinkName = sdf->Get<std::string>("launcher_link");

    if (sdf->HasElement("shoot_force"))
      this->shootForce = sdf->Get<double>("shoot_force");
    else
      this->shootForce = 50.0; // Reduced default

    if (sdf->HasElement("shoot_duration"))
      this->shootDuration = sdf->Get<double>("shoot_duration");
    else
      this->shootDuration = 0.1; // Shorter duration

    if (sdf->HasElement("initial_velocity"))
      this->initialVelocity = sdf->Get<double>("initial_velocity");
    else
      this->initialVelocity = 3.0; // Reduced to 3 m/s initial velocity

    // Initialize ROS2 node
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    
    this->node = std::make_shared<rclcpp::Node>("torpedo_shooter");

    this->sub = this->node->create_subscription<std_msgs::msg::Bool>(
      "/shoot_torpedo", 10,
      std::bind(&TorpedoShooterPlugin::OnShootMsg, this, std::placeholders::_1));

    this->ros_thread = std::thread([this]() {
      rclcpp::spin(this->node);
    });

    RCLCPP_INFO(this->node->get_logger(), 
                "Torpedo shooter plugin loaded. Launcher link: %s, World: %s", 
                this->launcherLinkName.c_str(),
                this->worldName.c_str());
  }

  void Update(const gz::sim::UpdateInfo &info,
              gz::sim::EntityComponentManager &ecm) override
  {
    // Handle pending torpedo spawn
    if (this->pendingTorpedoSpawn)
    {
      RCLCPP_INFO(this->node->get_logger(), "[Update] Spawning torpedo...");
      this->pendingTorpedoSpawn = false;
      this->SpawnTorpedo(ecm);
    }

    // Look for the newly spawned torpedo after waiting
    if (this->lookingForTorpedo)
    {
      this->torpedoSearchIterations++;
      
      if (this->torpedoSearchIterations >= 10)
      {
        auto worldEntity = gz::sim::worldEntity(ecm);
        gz::sim::World world(worldEntity);
        this->torpedoEntity = world.ModelByName(ecm, this->torpedoName);
        
        if (this->torpedoEntity != gz::sim::kNullEntity)
        {
          RCLCPP_INFO(this->node->get_logger(), "[Update] Torpedo found! Entity ID: %lu", this->torpedoEntity);
          
          // Verify the torpedo has a body link
          gz::sim::Model torpedo(this->torpedoEntity);
          auto torpedoLinkEntity = torpedo.LinkByName(ecm, "body");
          if (torpedoLinkEntity != gz::sim::kNullEntity)
          {
            RCLCPP_INFO(this->node->get_logger(), "[Update] Torpedo 'body' link found! Link ID: %lu", torpedoLinkEntity);
            
            // Set initial velocity on the torpedo
            gz::sim::Link torpedoLink(torpedoLinkEntity);
            gz::math::Vector3d initialVel = this->shootDirection * this->initialVelocity;
            
            // Create or set the LinearVelocity component
            auto velComp = ecm.Component<gz::sim::components::LinearVelocity>(torpedoLinkEntity);
            if (velComp)
            {
              ecm.SetComponentData<gz::sim::components::LinearVelocity>(torpedoLinkEntity, initialVel);
            }
            else
            {
              ecm.CreateComponent(torpedoLinkEntity, gz::sim::components::LinearVelocity(initialVel));
            }
            
            RCLCPP_INFO(this->node->get_logger(), "[Update] Set initial velocity [%.2f, %.2f, %.2f]", 
                       initialVel.X(), initialVel.Y(), initialVel.Z());
          }
          else
          {
            RCLCPP_WARN(this->node->get_logger(), "[Update] Torpedo 'body' link NOT found. Check torpedo SDF.");
          }
          
          this->lookingForTorpedo = false;
          this->applyingForce = true;
          this->forceStartTime = std::chrono::steady_clock::now();
        }
        else if (this->torpedoSearchIterations > 50)
        {
          RCLCPP_ERROR(this->node->get_logger(), "[Update] Failed to find torpedo '%s' after spawning.", this->torpedoName.c_str());
          this->lookingForTorpedo = false;
          this->isShooting = false;
        }
      }
      return;
    }

    // Apply force to torpedo
    if (this->torpedoEntity != gz::sim::kNullEntity && this->applyingForce)
    {
      auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - this->forceStartTime).count();

      if (elapsed < this->shootDuration)
      {
        gz::sim::Model torpedo(this->torpedoEntity);
        auto torpedoLinkEntity = torpedo.LinkByName(ecm, "body");

        if (torpedoLinkEntity != gz::sim::kNullEntity)
        {
          gz::sim::Link torpedoLink(torpedoLinkEntity);
          
          // Get current torpedo pose for debugging
          auto torpedoPose = torpedoLink.WorldPose(ecm);
          if (torpedoPose && this->torpedoSearchIterations % 10 == 0)
          {
            RCLCPP_INFO(this->node->get_logger(), 
                       "[Update] Torpedo at [%.2f, %.2f, %.2f]",
                       torpedoPose->Pos().X(), torpedoPose->Pos().Y(), torpedoPose->Pos().Z());
          }
          
          gz::math::Vector3d force = this->shootDirection * this->shootForce;
          torpedoLink.AddWorldForce(ecm, force);
        }
      }
      else
      {
        RCLCPP_INFO(this->node->get_logger(), "[Update] Force application complete.");
        this->applyingForce = false;
        this->torpedoEntity = gz::sim::kNullEntity;
        this->isShooting = false;
      }
    }
  }

  ~TorpedoShooterPlugin()
  {
    if (this->ros_thread.joinable())
    {
      rclcpp::shutdown();
      this->ros_thread.join();
    }
  }

private:
  void OnShootMsg(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !this->isShooting)
    {
      this->isShooting = true;
      this->pendingTorpedoSpawn = true;
      RCLCPP_INFO(this->node->get_logger(), "Shoot command received.");
    }
  }

  void SpawnTorpedo(gz::sim::EntityComponentManager &ecm)
  {
    // Load torpedo SDF file
    std::ifstream ifs(this->torpedoSDFPath);
    if (!ifs)
    {
      RCLCPP_ERROR(this->node->get_logger(),
                   "Cannot open torpedo SDF file: %s", this->torpedoSDFPath.c_str());
      this->isShooting = false;
      return;
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string sdfString = buffer.str();

    // Get launcher pose
    auto launcherLinkEntity = this->model.LinkByName(ecm, this->launcherLinkName);
    if (launcherLinkEntity == gz::sim::kNullEntity)
    {
      RCLCPP_ERROR(this->node->get_logger(), 
                   "Launcher link '%s' not found in model.", 
                   this->launcherLinkName.c_str());
      this->isShooting = false;
      return;
    }

    gz::sim::Link launcherLink(launcherLinkEntity);
    auto poseOpt = launcherLink.WorldPose(ecm);
    if (!poseOpt)
    {
      RCLCPP_ERROR(this->node->get_logger(), "Failed to get launcher pose.");
      this->isShooting = false;
      return;
    }

    // Calculate shoot direction and spawn pose
    this->shootDirection = poseOpt->Rot().RotateVector(gz::math::Vector3d(1, 0, 0));
    gz::math::Pose3d spawnPose = *poseOpt;
    
    // Spawn torpedo 1 meter in front of launcher to avoid collision issues
    spawnPose.Pos() += this->shootDirection * 1.0;
    
    // Also ensure torpedo is rotated to point in shoot direction
    spawnPose.Rot() = poseOpt->Rot();

    RCLCPP_INFO(this->node->get_logger(), 
                "Spawning torpedo at [%f, %f, %f]",
                spawnPose.Pos().X(), spawnPose.Pos().Y(), spawnPose.Pos().Z());

    // Create unique torpedo name
    static int torpedoCounter = 0;
    this->torpedoName = "torpedo_" + std::to_string(torpedoCounter++);

    // Use Gazebo transport to spawn the model
    gz::msgs::EntityFactory req;
    req.set_sdf(sdfString);
    req.set_name(this->torpedoName);
    req.mutable_pose()->mutable_position()->set_x(spawnPose.Pos().X());
    req.mutable_pose()->mutable_position()->set_y(spawnPose.Pos().Y());
    req.mutable_pose()->mutable_position()->set_z(spawnPose.Pos().Z());
    req.mutable_pose()->mutable_orientation()->set_w(spawnPose.Rot().W());
    req.mutable_pose()->mutable_orientation()->set_x(spawnPose.Rot().X());
    req.mutable_pose()->mutable_orientation()->set_y(spawnPose.Rot().Y());
    req.mutable_pose()->mutable_orientation()->set_z(spawnPose.Rot().Z());

    std::string topic = "/world/" + this->worldName + "/create";
    
    gz::transport::Node gzNode;
    gz::msgs::Boolean response;
    bool result = false;
    unsigned int timeout = 5000;
    bool executed = gzNode.Request(topic, req, timeout, response, result);

    if (executed && result)
    {
      RCLCPP_INFO(this->node->get_logger(), "Torpedo spawn request sent successfully.");
      this->lookingForTorpedo = true;
      this->torpedoSearchIterations = 0;
    }
    else
    {
      RCLCPP_ERROR(this->node->get_logger(), "Failed to spawn torpedo via transport.");
      this->isShooting = false;
    }
  }

private:
  gz::sim::Entity entity;
  gz::sim::Model model;
  std::string worldName;

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub;
  std::thread ros_thread;

  std::string torpedoSDFPath;
  std::string launcherLinkName = "base_link";
  std::string torpedoName;
  double shootForce = 50.0; // Reduced default
  double shootDuration = 0.1; // Shorter duration
  double initialVelocity = 3.0; // Initial velocity in m/s
  
  bool isShooting = false;
  bool pendingTorpedoSpawn = false;
  bool applyingForce = false;
  bool lookingForTorpedo = false;
  int torpedoSearchIterations = 0;

  gz::sim::Entity torpedoEntity = gz::sim::kNullEntity;
  gz::math::Vector3d shootDirection;
  std::chrono::steady_clock::time_point forceStartTime;
};

}  // namespace auv_sim

GZ_ADD_PLUGIN(auv_sim::TorpedoShooterPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemUpdate)