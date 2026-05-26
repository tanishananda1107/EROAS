#include "UnderwaterCurrentPlugin.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>

using namespace uuv_gz_sim;

class UnderwaterCurrentPluginPrivate
{
public:
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
};

void UnderwaterCurrentPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  model = gz::sim::Model(_entity);

  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  velModel.SetModel(1.0, 0.0, 2.0, 0.1, 0.2);
  horzModel.SetModel(0.0, -3.14, 3.14, 0.05, 0.1);
  vertModel.SetModel(0.0, -1.57, 1.57, 0.05, 0.1);
}

void UnderwaterCurrentPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &)
{
  double t = std::chrono::duration<double>(
    _info.simTime).count();

  UpdateCurrent(t);
}

void UnderwaterCurrentPlugin::UpdateCurrent(double _time)
{
  double v = velModel.Update(_time);
  double h = horzModel.Update(_time);
  double z = vertModel.Update(_time);

  currentVelocity =
    gz::math::Vector3d(
      v * cos(h) * cos(z),
      v * sin(h) * cos(z),
      v * sin(z));

  static UnderwaterCurrentPluginPrivate data;

  if (!data.pub)
    data.pub = data.node.Advertise<gz::msgs::Vector3d>("~/"+topic);

  gz::msgs::Vector3d msg;
  msg.set_x(currentVelocity.X());
  msg.set_y(currentVelocity.Y());
  msg.set_z(currentVelocity.Z());

  data.pub.Publish(msg);
}

GZ_ADD_PLUGIN(
  UnderwaterCurrentPlugin,
  gz::sim::System,
  UnderwaterCurrentPlugin::ISystemConfigure,
  UnderwaterCurrentPlugin::ISystemPreUpdate
)
