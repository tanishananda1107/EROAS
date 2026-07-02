#include "uuv_world_plugins/UnderwaterCurrentPlugin.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>

#include <cmath>
#include <chrono>

namespace uuv_gz_sim
{

void UnderwaterCurrentPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  worldEntity = _entity;

  if (_sdf->HasElement("topic"))
    currentVelocityTopic = _sdf->Get<std::string>("topic");

  if (currentVelocityTopic.empty())
    currentVelocityTopic = "hydrodynamics/current_velocity";

  currentVelModel.SetModel(1.0, 0.0, 2.0, 0.1, 0.2);
  currentHorzAngleModel.SetModel(0.0, -3.14, 3.14, 0.05, 0.1);
  currentVertAngleModel.SetModel(0.0, -1.57, 1.57, 0.05, 0.1);
}

void UnderwaterCurrentPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &)
{
  const double t = std::chrono::duration<double>(
    _info.simTime).count();

  UpdateCurrent(t);
}

void UnderwaterCurrentPlugin::UpdateCurrent(double _time)
{
  const double v = currentVelModel.Update(_time);
  const double h = currentHorzAngleModel.Update(_time);
  const double z = currentVertAngleModel.Update(_time);

  currentVelocity =
    gz::math::Vector3d(
      v * cos(h) * cos(z),
      v * sin(h) * cos(z),
      v * sin(z));

  gz::msgs::Vector3d msg;
  msg.set_x(currentVelocity.X());
  msg.set_y(currentVelocity.Y());
  msg.set_z(currentVelocity.Z());

  node.Advertise<gz::msgs::Vector3d>("~/" + currentVelocityTopic).Publish(msg);
}

} // namespace uuv_gz_sim

GZ_ADD_PLUGIN(
  uuv_gz_sim::UnderwaterCurrentPlugin,
  gz::sim::System,
  uuv_gz_sim::UnderwaterCurrentPlugin::ISystemConfigure,
  uuv_gz_sim::UnderwaterCurrentPlugin::ISystemPreUpdate
)
