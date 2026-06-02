#ifndef UUV_GZ_THRUSTER_PLUGIN_HH_
#define UUV_GZ_THRUSTER_PLUGIN_HH_

#include <memory>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Entity.hh>

#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/transport/Node.hh>

#include "Dynamics.hh"
#include "ThrusterConversionFcn.hh"

namespace uuv_gz_plugins
{

class ThrusterPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  ThrusterPlugin();
  ~ThrusterPlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

protected:
  void OnInput(const gz::msgs::Double &_msg);
  void Reset();

protected:
  gz::sim::Model model;
  gz::sim::Entity thrusterLinkEntity{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity{gz::sim::kNullEntity};

  std::shared_ptr<Dynamics> thrusterDynamics;
  std::shared_ptr<ConversionFunction> conversionFunction;

  double inputCommand{0.0};
  double thrustForce{0.0};

  double clampMin{-1.0};
  double clampMax{1.0};

  double thrustMin{-10.0};
  double thrustMax{10.0};

  double gain{1.0};
  double thrustEfficiency{1.0};
  double propellerEfficiency{1.0};
  int thrusterID{-1};

  bool isOn{true};

  std::string topicPrefix;
  std::string commandTopic;
  std::string thrustTopic;
  std::string linkName;
  gz::transport::Node node;
  gz::transport::Node::Publisher thrustPub;
  gz::math::Vector3d thrusterAxis{gz::math::Vector3d::UnitX};
};

} // namespace uuv_gz_plugins

#endif
