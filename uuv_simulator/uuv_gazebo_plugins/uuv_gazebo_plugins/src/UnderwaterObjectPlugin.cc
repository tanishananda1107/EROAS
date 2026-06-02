#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Inertial.hh>
#include <sdf/Element.hh>

#include <uuv_gazebo_plugins/UnderwaterObjectPlugin.hh>

namespace
{
std::vector<double> ParseDoubles(const std::string &_value)
{
  std::vector<double> out;
  std::istringstream stream(_value);
  double value = 0.0;
  while (stream >> value)
    out.push_back(value);
  return out;
}

double FirstMagnitude(const std::vector<double> &_values, double _fallback)
{
  for (const auto value : _values)
  {
    if (std::abs(value) > 0.0)
      return std::abs(value);
  }
  return _fallback;
}
}

namespace uuv_gz_plugins
{

void UnderwaterObjectPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
{
  this->modelEntity = _entity;
  this->model = gz::sim::Model(_entity);
  double defaultFluidDensity = 1028.0;
  if (_sdf->HasElement("fluid_density"))
    defaultFluidDensity = _sdf->Get<double>("fluid_density");

  if (_sdf->HasElement("use_global_current"))
    this->useGlobalCurrent = _sdf->Get<bool>("use_global_current");

  if (_sdf->HasElement("flow_velocity_topic"))
  {
    const auto topic = _sdf->Get<std::string>("flow_velocity_topic");
    if (!topic.empty())
      this->node.Subscribe(topic, &UnderwaterObjectPlugin::OnFlowVelocity, this);
  }

  auto sdf = std::const_pointer_cast<sdf::Element>(_sdf);
  for (auto linkElem = sdf->GetElement("link"); linkElem;
       linkElem = linkElem->GetNextElement("link"))
  {
    if (!linkElem->HasAttribute("name"))
      continue;

    LinkModel linkModel;
    linkModel.name = linkElem->Get<std::string>("name");
    linkModel.entity = this->model.LinkByName(_ecm, linkModel.name);
    if (linkModel.entity == gz::sim::kNullEntity)
    {
      gzwarn << "UnderwaterObjectPlugin: link [" << linkModel.name
             << "] was not found in the model.\n";
      continue;
    }

    linkModel.fluidDensity = defaultFluidDensity;
    if (linkElem->HasElement("fluid_density"))
      linkModel.fluidDensity = linkElem->Get<double>("fluid_density");

    if (linkElem->HasElement("volume"))
      linkModel.volume = linkElem->Get<double>("volume");

    if (linkElem->HasElement("center_of_buoyancy"))
    {
      const auto values =
        ParseDoubles(linkElem->Get<std::string>("center_of_buoyancy"));
      if (values.size() >= 3)
      {
        linkModel.centerOfBuoyancy =
          gz::math::Vector3d(values[0], values[1], values[2]);
      }
    }

    if (linkElem->HasElement("neutrally_buoyant"))
      linkModel.neutrallyBuoyant = linkElem->Get<bool>("neutrally_buoyant");

    if (linkElem->HasElement("hydrodynamic_model"))
    {
      auto hydro = linkElem->GetElement("hydrodynamic_model");
      if (hydro->HasElement("linear_damping"))
        linkModel.linearDamping =
          FirstMagnitude(ParseDoubles(hydro->Get<std::string>("linear_damping")),
                        linkModel.linearDamping);
      if (hydro->HasElement("quadratic_damping"))
        linkModel.angularDamping =
          FirstMagnitude(ParseDoubles(hydro->Get<std::string>("quadratic_damping")),
                        linkModel.angularDamping);
    }

    if (linkModel.neutrallyBuoyant || linkModel.volume <= 0.0)
    {
      const auto mass = this->LinkMass(linkModel.entity, _ecm);
      if (mass > 0.0)
        linkModel.volume = mass / linkModel.fluidDensity;
    }

    this->models.push_back(linkModel);
  }

  gzmsg << "UnderwaterObjectPlugin configured " << this->models.size()
        << " link model(s).\n";
}

void UnderwaterObjectPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  for (auto &linkModel : this->models)
  {
    gz::sim::Link link(linkModel.entity);
    const auto pose = link.WorldPose(_ecm).value_or(gz::math::Pose3d::Zero);
    const auto linearVelocity =
      link.WorldLinearVelocity(_ecm).value_or(gz::math::Vector3d::Zero);
    const auto angularVelocity =
      link.WorldAngularVelocity(_ecm).value_or(gz::math::Vector3d::Zero);

    linkModel.submerged = pose.Pos().Z() <= 0.0;
    if (!linkModel.submerged)
      continue;

    const gz::math::Vector3d buoyancyWorld{
      0.0, 0.0, linkModel.volume * linkModel.fluidDensity * this->gravity};
    const auto cobWorld =
      pose.Pos() + pose.Rot().RotateVector(linkModel.centerOfBuoyancy);
    link.AddWorldForce(_ecm, buoyancyWorld, cobWorld);

    const auto relativeLinearVelocity = linearVelocity - this->flowVelocity;
    if (linkModel.linearDamping > 0.0)
      link.AddWorldForce(_ecm, -linkModel.linearDamping * relativeLinearVelocity);

    if (linkModel.angularDamping > 0.0)
      link.AddWorldWrench(
        _ecm, gz::math::Vector3d::Zero,
        -linkModel.angularDamping * angularVelocity);
  }
}

void UnderwaterObjectPlugin::SetUseGlobalCurrent(bool _useGlobal)
{
  this->useGlobalCurrent = _useGlobal;
}

bool UnderwaterObjectPlugin::UseGlobalCurrent() const
{
  return this->useGlobalCurrent;
}

void UnderwaterObjectPlugin::SetFlowVelocity(
    const gz::math::Vector3d &_flowVelocity)
{
  this->flowVelocity = _flowVelocity;
}

gz::math::Vector3d UnderwaterObjectPlugin::FlowVelocity() const
{
  return this->flowVelocity;
}

const std::vector<UnderwaterObjectPlugin::LinkModel> &
UnderwaterObjectPlugin::Models() const
{
  return this->models;
}

void UnderwaterObjectPlugin::OnFlowVelocity(const gz::msgs::Vector3d &_msg)
{
  if (!this->useGlobalCurrent)
    return;

  this->flowVelocity.Set(_msg.x(), _msg.y(), _msg.z());
}

double UnderwaterObjectPlugin::LinkMass(
    gz::sim::Entity _link,
    const gz::sim::EntityComponentManager &_ecm) const
{
  const auto inertial =
    _ecm.Component<gz::sim::components::Inertial>(_link);
  if (!inertial)
    return 0.0;
  return inertial->Data().MassMatrix().Mass();
}

}  // namespace uuv_gz_plugins

GZ_ADD_PLUGIN(uuv_gz_plugins::UnderwaterObjectPlugin,
              gz::sim::System,
              uuv_gz_plugins::UnderwaterObjectPlugin::ISystemConfigure,
              uuv_gz_plugins::UnderwaterObjectPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(uuv_gz_plugins::UnderwaterObjectPlugin,
                    "uuv_gz_plugins::UnderwaterObjectPlugin")
