#include <memory>
#include <string>
#include <array>
#include <cmath>

#include <gz/plugin/Register.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/WorldLinearVelocity.hh>
#include <gz/sim/components/WorldAngularVelocity.hh>

using namespace gz;
using namespace sim;
using namespace systems;

namespace simple_hydro
{

class SimpleHydrodynamics
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    model_ = Model(_entity);
    if (!model_.Valid(_ecm))
    {
      gzerr << "[simple_hydrodynamics] Invalid model entity.\n";
      return;
    }

    if (!_sdf || !_sdf->HasElement("link_name"))
    {
      gzerr << "[simple_hydrodynamics] <link_name> is required.\n";
      return;
    }
    linkName_ = _sdf->Get<std::string>("link_name");
    linkEntity_ = model_.LinkByName(_ecm, linkName_);
    if (!_ecm.HasEntity(linkEntity_))
    {
      gzerr << "[simple_hydrodynamics] Link '" << linkName_
            << "' not found.\n";
      return;
    }
    link_ = Link(linkEntity_);

    // Optional params
    waterDensity_ = _sdf->Get<double>("water_density", 998.0).first;

    // Default coefficients (per-DOF)
    // Linear:  Fx = -Llin.x * u,  Fy = -Llin.y * v,  Fz = -Llin.z * w
    // Angular: Tx = -Lang.x * p,  Ty = -Lang.y * q,  Tz = -Lang.z * r
    // Quadratic: Fx += -Qlin.x * u*|u|, ...
    //           Tx += -Qang.x * p*|p|, ...
    linearLin_ = _sdf->Get<math::Vector3d>("linear_drag", {0,0,0});
    linearAng_ = _sdf->Get<math::Vector3d>("angular_drag", {0,0,0});
    quadLin_   = _sdf->Get<math::Vector3d>("quadratic_drag", {0,0,0});
    quadAng_   = _sdf->Get<math::Vector3d>("quadratic_angular_drag", {0,0,0});

    // Optional constant world-frame current (m/s)
    currentWorld_ = _sdf->Get<math::Vector3d>("default_current", {0,0,0});

    // Make sure needed components exist
    if (!_ecm.Component<components::WorldPose>(linkEntity_))
      _ecm.CreateComponent(linkEntity_, components::WorldPose());
    if (!_ecm.Component<components::WorldLinearVelocity>(linkEntity_))
      _ecm.CreateComponent(linkEntity_, components::WorldLinearVelocity());
    if (!_ecm.Component<components::WorldAngularVelocity>(linkEntity_))
      _ecm.CreateComponent(linkEntity_, components::WorldAngularVelocity());

    gzdbg << "[simple_hydrodynamics] link='" << linkName_
          << "' linear_drag=" << linearLin_
          << " angular_drag=" << linearAng_
          << " quadratic_drag=" << quadLin_
          << " quadratic_angular_drag=" << quadAng_
          << " current_world=" << currentWorld_ << "\n";
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (_info.paused || linkEntity_ == kNullEntity)
      return;

    // If world was reset / reloaded, re-resolve the link
    if (!_ecm.HasEntity(linkEntity_))
    {
      linkEntity_ = model_.LinkByName(_ecm, linkName_);
      link_ = Link(linkEntity_);
      if (linkEntity_ == kNullEntity)
      {
        gzwarn << "[simple_hydrodynamics] Waiting for link '" << linkName_
               << "'...\n";
        return;
      }
    }

    // Pose and velocities in world frame
    auto poseOpt = link_.WorldPose(_ecm);
    if (!poseOpt)
      return;
    const auto &pose = *poseOpt;

    auto linComp = _ecm.Component<components::WorldLinearVelocity>(linkEntity_);
    auto angComp = _ecm.Component<components::WorldAngularVelocity>(linkEntity_);
    if (!linComp || !angComp)
      return;

    math::Vector3d vWorld = linComp->Data();
    math::Vector3d wWorld = angComp->Data();

    // Relative linear velocity of body w.r.t. water (subtract world current)
    math::Vector3d vRelWorld = vWorld - currentWorld_;

    // Convert to body frame to apply per-axis drag
    math::Quaterniond R = pose.Rot();
    math::Vector3d vRelBody = R.Inverse().RotateVector(vRelWorld);
    math::Vector3d wBody    = R.Inverse().RotateVector(wWorld);

    // Compute body-frame drag force/torque
    math::Vector3d Fbody(
      -linearLin_.X() * vRelBody.X() - quadLin_.X() * vRelBody.X() * std::abs(vRelBody.X()),
      -linearLin_.Y() * vRelBody.Y() - quadLin_.Y() * vRelBody.Y() * std::abs(vRelBody.Y()),
      -linearLin_.Z() * vRelBody.Z() - quadLin_.Z() * vRelBody.Z() * std::abs(vRelBody.Z())
    );

    math::Vector3d Tbody(
      -linearAng_.X() * wBody.X() - quadAng_.X() * wBody.X() * std::abs(wBody.X()),
      -linearAng_.Y() * wBody.Y() - quadAng_.Y() * wBody.Y() * std::abs(wBody.Y()),
      -linearAng_.Z() * wBody.Z() - quadAng_.Z() * wBody.Z() * std::abs(wBody.Z())
    );

    // (Optional) scale by water density if you want coefficients to be
    // dimensionless (left here as an on/off with density multiplier = 1 by default)
    // If your coefficients are already in N/(m/s) and N·m/(rad/s), keep as-is.
    const bool scaleByDensity = false;
    if (scaleByDensity)
    {
      Fbody *= waterDensity_;
      Tbody *= waterDensity_;
    }

    // Apply at CoM in world frame
    math::Vector3d Fworld = R.RotateVector(Fbody);
    math::Vector3d Tworld = R.RotateVector(Tbody);
    link_.AddWorldWrench(_ecm, Fworld, Tworld);
  }

private:
  Model model_{kNullEntity};
  std::string linkName_;
  Entity linkEntity_{kNullEntity};
  Link link_{kNullEntity};

  // Parameters
  double waterDensity_{998.0};
  math::Vector3d currentWorld_{0,0,0};
  math::Vector3d linearLin_{0,0,0};   // [X_u, Y_v, Z_w]
  math::Vector3d quadLin_{0,0,0};     // [X_u|u|, Y_v|v|, Z_w|w|]
  math::Vector3d linearAng_{0,0,0};   // [K_p, M_q, N_r]
  math::Vector3d quadAng_{0,0,0};     // [K_p|p|, M_q|q|, N_r|r|]
};

} // namespace simple_hydro

// --- Plugin registration ---
GZ_ADD_PLUGIN(simple_hydro::SimpleHydrodynamics,
              gz::sim::System,
              simple_hydro::SimpleHydrodynamics::ISystemConfigure,
              simple_hydro::SimpleHydrodynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(simple_hydro::SimpleHydrodynamics, "simple_hydrodynamics")
