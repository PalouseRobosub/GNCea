#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Link.hh>
#include <gz/math/Vector3.hh>
#include <sdf/sdf.hh>

using namespace gz;
using namespace sim;

namespace gz::sim::systems
{

class SimpleBuoyancy
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public: void Configure(const Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       EntityComponentManager &_ecm,
                       EventManager &) override
{
  this->model = Model(_entity);

  if (!this->model.Valid(_ecm))
  {
    gzerr << "[SimpleBuoyancy] Must be attached to a <model>.\n";
    return;
  }

  if (_sdf->HasElement("link_name"))
    this->linkName = _sdf->Get<std::string>("link_name");

  if (_sdf->HasElement("water_level"))
    this->waterLevel = _sdf->Get<double>("water_level");

  if (_sdf->HasElement("fluid_density"))
    this->fluidDensity = _sdf->Get<double>("fluid_density");

  if (_sdf->HasElement("volume"))
    this->linkVolume = _sdf->Get<double>("volume");

  if (!this->linkName.empty())
  {
    this->linkEntity = this->model.LinkByName(_ecm, this->linkName);
    if (this->linkEntity == kNullEntity)
      gzwarn << "[SimpleBuoyancy] Link '" << this->linkName
             << "' not found yet; will retry in PreUpdate.\n";
  }
}

public: void PreUpdate(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) override
{
  if (_info.paused) return;
  if (!this->model.Valid(_ecm)) return;

  // Resolve link if not yet resolved
  if (this->linkEntity == kNullEntity)
  {
    if (!this->linkName.empty())
      this->linkEntity = this->model.LinkByName(_ecm, this->linkName);
    else
    {
      auto links = _ecm.ChildrenByComponents(this->model.Entity(), components::Link());
      if (!links.empty())
        this->linkEntity = links.front();
    }
    if (this->linkEntity == kNullEntity) return;
  }

  // Gravity
  Entity worldEnt = worldEntity(this->model.Entity(), _ecm);
  auto gravity = _ecm.Component<components::Gravity>(worldEnt);
  if (!gravity) return;
  auto g = gravity->Data(); // [0,0,-9.81]
  if (g.Length() < 1e-9) return;

  // Link mass
  auto inertial = _ecm.Component<components::Inertial>(this->linkEntity);
  if (!inertial) return;
  const double m = inertial->Data().MassMatrix().Mass();

  // Approximate bounding box height = cube root of volume
  double side = std::cbrt(this->linkVolume);
  double halfHeight = side / 2.0;

  // World pose of link
  auto pose = worldPose(this->linkEntity, _ecm);
  double centerZ = pose.Pos().Z();

  // Top and bottom of the cube
  double top = centerZ + halfHeight;
  double bottom = centerZ - halfHeight;

  // Compute submerged height
  double submerged = std::max(0.0, std::min(this->waterLevel, top) - bottom);
  submerged = std::clamp(submerged, 0.0, side);

  // Fraction submerged
  double fraction = submerged / side;

  // Buoyant force = fluid_density * displaced_volume * g
  double displacedVol = this->linkVolume * fraction;
  double buoyantN = this->fluidDensity * displacedVol * g.Length();

  // Apply force upward
  gz::math::Vector3d upDir = -g.Normalized();
  gz::math::Vector3d F = upDir * buoyantN;
  gz::math::Vector3d tau = gz::math::Vector3d::Zero;

  Link link(this->linkEntity);
  link.AddWorldWrench(_ecm, F, tau);
}

private: Model model{kNullEntity};
private: Entity linkEntity{kNullEntity};
private: std::string linkName;

// Parameters
private: double waterLevel{0.0};          // Z = 0 by default
private: double fluidDensity{1000.0};     // water kg/m^3
private: double linkVolume{1.0};          // m^3 (user sets)
};

} // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::SimpleBuoyancy,
              gz::sim::System,
              gz::sim::systems::SimpleBuoyancy::ISystemConfigure,
              gz::sim::systems::SimpleBuoyancy::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::SimpleBuoyancy,
                    "gz::sim::systems::SimpleBuoyancy")
