#include <array>
#include <memory>
#include <string>
#include <optional>
#include <mutex>
#include <cmath>

#include <gz/plugin/Register.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/WorldLinearVelocity.hh>
#include <gz/sim/components/WorldAngularVelocity.hh>

using namespace gz;
using namespace sim;
using namespace systems;

namespace complex_hydro
{

// Indices: 0..5 = [u, v, w, p, q, r]
enum {U=0,V,W,P,Q,R};

class ComplexHydrodynamics final
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
      gzerr << "[complex_hydrodynamics] Invalid model.\n";
      return;
    }

    if (!_sdf || !_sdf->HasElement("link_name"))
    {
      gzerr << "[complex_hydrodynamics] <link_name> is required.\n";
      return;
    }
    linkName_ = _sdf->Get<std::string>("link_name");
    linkEntity_ = model_.LinkByName(_ecm, linkName_);
    if (!_ecm.HasEntity(linkEntity_))
    {
      gzerr << "[complex_hydrodynamics] Link '" << linkName_ << "' not found.\n";
      return;
    }
    link_ = Link(linkEntity_);

    waterDensity_ = _sdf->Get<double>("water_density", 998.0).first;
    currentWorld_ = _sdf->Get<math::Vector3d>("default_current", {0,0,0});

    // ---- Added mass diagonal (Ma) ----
    Ma_[U] = _sdf->Get<double>("xDotU", 0.0).first;
    Ma_[V] = _sdf->Get<double>("yDotV", 0.0).first;
    Ma_[W] = _sdf->Get<double>("zDotW", 0.0).first;
    Ma_[P] = _sdf->Get<double>("kDotP", 0.0).first;
    Ma_[Q] = _sdf->Get<double>("mDotQ", 0.0).first;
    Ma_[R] = _sdf->Get<double>("nDotR", 0.0).first;

    // ---- Linear damping diagonal ----
    Dlin_[U] = _sdf->Get<double>("xU", 0.0).first;
    Dlin_[V] = _sdf->Get<double>("yV", 0.0).first;
    Dlin_[W] = _sdf->Get<double>("zW", 0.0).first;
    Dlin_[P] = _sdf->Get<double>("kP", 0.0).first;
    Dlin_[Q] = _sdf->Get<double>("mQ", 0.0).first;
    Dlin_[R] = _sdf->Get<double>("nR", 0.0).first;

    // ---- Quadratic-|v| damping diagonal ----
    Dabs_[U] = _sdf->Get<double>("xUabsU", 0.0).first;
    Dabs_[V] = _sdf->Get<double>("yVabsV", 0.0).first;
    Dabs_[W] = _sdf->Get<double>("zWabsW", 0.0).first;
    Dabs_[P] = _sdf->Get<double>("kPabsP", 0.0).first;
    Dabs_[Q] = _sdf->Get<double>("mQabsQ", 0.0).first;
    Dabs_[R] = _sdf->Get<double>("nRabsR", 0.0).first;

    // Optional toggles
    disableCoriolis_  = _sdf->Get<bool>("disable_coriolis", false).first;
    disableAddedMass_ = _sdf->Get<bool>("disable_added_mass", false).first;

    // Ensure components exist
    if (!_ecm.Component<components::WorldPose>(linkEntity_))
      _ecm.CreateComponent(linkEntity_, components::WorldPose());
    if (!_ecm.Component<components::WorldLinearVelocity>(linkEntity_))
      _ecm.CreateComponent(linkEntity_, components::WorldLinearVelocity());
    if (!_ecm.Component<components::WorldAngularVelocity>(linkEntity_))
      _ecm.CreateComponent(linkEntity_, components::WorldAngularVelocity());

    prevState_.fill(0.0);

    gzdbg << "[complex_hydrodynamics] link=" << linkName_
          << " Ma=[ " << Ma_[U] << " " << Ma_[V] << " " << Ma_[W] << " "
          << Ma_[P] << " " << Ma_[Q] << " " << Ma_[R] << " ] "
          << " Dlin=[ " << Dlin_[U] << " " << Dlin_[V] << " " << Dlin_[W] << " "
          << Dlin_[P] << " " << Dlin_[Q] << " " << Dlin_[R] << " ] "
          << " Dabs=[ " << Dabs_[U] << " " << Dabs_[V] << " " << Dabs_[W] << " "
          << Dabs_[P] << " " << Dabs_[Q] << " " << Dabs_[R] << " ] "
          << " current=" << currentWorld_ << "\n";
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (_info.paused) return;

    // Re-resolve if world reset
    if (linkEntity_ == kNullEntity || !_ecm.HasEntity(linkEntity_))
    {
      linkEntity_ = model_.LinkByName(_ecm, linkName_);
      link_ = Link(linkEntity_);
      if (linkEntity_ == kNullEntity) return;
    }

    auto poseOpt = link_.WorldPose(_ecm);
    if (!poseOpt) return;
    const auto &pose = *poseOpt;

    auto linComp = _ecm.Component<components::WorldLinearVelocity>(linkEntity_);
    auto angComp = _ecm.Component<components::WorldAngularVelocity>(linkEntity_);
    if (!linComp || !angComp) return;

    // World velocities
    math::Vector3d vW = linComp->Data();
    math::Vector3d wW = angComp->Data();

    // Relative water velocity (subtract world current), then to body frame
    math::Quaterniond R = pose.Rot();
    math::Vector3d vRelB = R.Inverse().RotateVector(vW - currentWorld_);
    math::Vector3d wB    = R.Inverse().RotateVector(wW);

    // State [u v w p q r]
    std::array<double,6> x {
      vRelB.X(), vRelB.Y(), vRelB.Z(),
      wB.X(),    wB.Y(),    wB.Z()
    };

    // dt
    const double dt = std::max(1e-6, static_cast<double>(_info.dt.count())/1e9);
    std::array<double,6> xdot;
    for (int i=0;i<6;++i) { xdot[i] = (x[i] - prevState_[i]) / dt; }
    prevState_ = x;

    // ----- Added-mass term:  -Ma * xdot  (diag only) -----
    std::array<double,6> F_Ma{};
    for (int i=0;i<6;++i) F_Ma[i] = - Ma_[i] * xdot[i];

    // ----- Coriolis/Centripetal from added mass:  -C(x)*x (diag-only form) -----
    // Using the same pattern as your reference (only diag terms of Ma used):
    // C(u,v,w,p,q,r) with Ma = diag(Xu_dot, Yv_dot, Zw_dot, Kp_dot, Mq_dot, Nr_dot)
    std::array<double,6> F_C{};
    if (!disableCoriolis_)
    {
      const double Xu = Ma_[U], Yv = Ma_[V], Zw = Ma_[W];
      const double Kp = Ma_[P], Mq = Ma_[Q], Nr = Ma_[R];

      F_C[U] = -( -Zw * x[W]*x[Q] - Yv * x[V]*x[R] );
      F_C[V] = -(  Zw * x[W]*x[P] - Xu * x[U]*x[R] );
      F_C[W] = -( -Yv * x[V]*x[P] + Xu * x[U]*x[Q] );
      F_C[P] = -( -Zw * x[W]*x[V] + Yv * x[V]*x[W] - Nr * x[R]*x[Q] + Mq * x[Q]*x[R] );
      F_C[Q] = -(  Zw * x[W]*x[U] - Xu * x[U]*x[W] + Nr * x[R]*x[P] - Kp * x[P]*x[R] );
      F_C[R] = -(  Xu * x[U]*x[V] + Yv * x[V]*x[U] - Mq * x[Q]*x[P] - Kp * x[P]*x[Q] );
      // Note: this collapses full C(x) to a form consistent with the diagonal Ma usage
      // seen in the EER example you posted.
    }

    // ----- Damping: linear + abs (quadratic-|x|) on each axis -----
    std::array<double,6> F_D{};
    for (int i=0;i<6;++i)
    {
      const double s = x[i];
      F_D[i] = (Dlin_[i]) * s + (Dabs_[i]) * s * std::abs(s);
      // Your SDF uses negative numbers for damping (e.g., -40), so this already
      // yields a resistive force. No extra minus sign here.
    }

    // Total generalized wrench in body frame (surge,sway,heave,roll,pitch,yaw)
    std::array<double,6> tauB{};
    for (int i=0;i<6;++i)
      tauB[i] = F_D[i] + (disableAddedMass_ ? 0.0 : F_Ma[i]) + (disableCoriolis_ ? 0.0 : F_C[i]);

    // Gazebo expects world-frame force & torque applied at CoM
    math::Vector3d Fbody(tauB[U], tauB[V], tauB[W]);
    math::Vector3d Tbody(tauB[P], tauB[Q], tauB[R]);

    math::Vector3d Fworld = R.RotateVector(Fbody);
    math::Vector3d Tworld = R.RotateVector(Tbody);
    link_.AddWorldWrench(_ecm, Fworld, Tworld);
  }

private:
  Model model_{kNullEntity};
  std::string linkName_;
  Entity linkEntity_{kNullEntity};
  Link link_{kNullEntity};

  // Params
  double waterDensity_{998.0};
  math::Vector3d currentWorld_{0,0,0};

  // Diagonal coefficients
  std::array<double,6> Ma_   {{0,0,0,0,0,0}};  // added mass diag [xDotU .. nDotR]
  std::array<double,6> Dlin_ {{0,0,0,0,0,0}};  // linear damping [xU .. nR]
  std::array<double,6> Dabs_ {{0,0,0,0,0,0}};  // abs damping   [xUabsU .. nRabsR]

  bool disableCoriolis_{false};
  bool disableAddedMass_{false};

  std::array<double,6> prevState_{{0,0,0,0,0,0}};
};

} // namespace complex_hydro

// --- Registration ---
// Primary name
GZ_ADD_PLUGIN(complex_hydro::ComplexHydrodynamics,
              gz::sim::System,
              complex_hydro::ComplexHydrodynamics::ISystemConfigure,
              complex_hydro::ComplexHydrodynamics::ISystemPreUpdate)

// Friendly alias so you can reference it by a simple string
GZ_ADD_PLUGIN_ALIAS(complex_hydro::ComplexHydrodynamics, "complex_hydrodynamics")

// **Compatibility alias**: keep using your existing URDF name if you want
GZ_ADD_PLUGIN_ALIAS(complex_hydro::ComplexHydrodynamics, "gz::sim::systems::EER_Hydrodynamics")
