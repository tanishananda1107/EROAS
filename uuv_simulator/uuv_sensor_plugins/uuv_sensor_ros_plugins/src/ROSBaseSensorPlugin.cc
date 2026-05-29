// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#include <uuv_sensor_ros_plugins/ROSBaseSensorPlugin.hh>
#include <gz/sensors/SensorFactory.hh>

namespace gz { namespace sim {

ROSBaseSensorPlugin::ROSBaseSensorPlugin() {}
ROSBaseSensorPlugin::~ROSBaseSensorPlugin() {}

void ROSBaseSensorPlugin::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& /*_eventMgr*/)
{
  this->sensorEntity = _entity;
  auto sdfPtr = std::const_pointer_cast<sdf::Element>(_sdf);
  this->InitBasePlugin(sdfPtr);
}

void ROSBaseSensorPlugin::Update(
  const UpdateInfo& _info,
  EntityComponentManager& _ecm)
{
  this->OnUpdate(_info, _ecm);
}

bool ROSBaseSensorPlugin::OnUpdate(
  const UpdateInfo& /*_info*/,
  EntityComponentManager& /*_ecm*/)
{
  return true;
}

}} // namespace gz::sim
