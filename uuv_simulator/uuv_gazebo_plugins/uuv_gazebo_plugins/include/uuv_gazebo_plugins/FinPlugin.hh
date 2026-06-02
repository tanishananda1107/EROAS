#ifndef UUV_GZ_FIN_PLUGIN_HH_
#define UUV_GZ_FIN_PLUGIN_HH_

#include <memory>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/transport/Node.hh>

#include "LiftDragModel.hh"
#include "Dynamics.hh"

namespace uuv_gz_plugins
{

class FinPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  FinPlugin();
  ~FinPlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

protected:
  void OnInput(const gz::msgs::Double &_msg);
  void OnCurrentVelocity(const gz::msgs::Vector3d &_msg);

protected:
  gz::sim::Model model;
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity{gz::sim::kNullEntity};

  std::unique_ptr<Dynamics> dynamics;
  std::unique_ptr<LiftDrag> liftdrag;

  double inputCommand{0.0};
  double angle{0.0};

  gz::math::Vector3d currentVelocity;
  gz::math::Vector3d finForce;

  int finID{-1};
  std::string topicPrefix;
  std::string commandTopic;
  std::string angleTopic;
  std::string linkName;
  gz::transport::Node node;
  gz::transport::Node::Publisher anglePub;
};

} // namespace uuv_gz_plugins

#endif
