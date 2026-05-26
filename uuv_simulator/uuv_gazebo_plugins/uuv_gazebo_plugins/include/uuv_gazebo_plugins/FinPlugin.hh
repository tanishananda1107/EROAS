#ifndef UUV_GZ_FIN_PLUGIN_HH_
#define UUV_GZ_FIN_PLUGIN_HH_

#include <memory>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <gz/math/Vector3.hh>

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
  FinPlugin() = default;
  virtual ~FinPlugin() = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

protected:
  void UpdateInput(double _msg);
  void UpdateCurrentVelocity(const gz::math::Vector3d &_vel);

protected:
  gz::sim::Entity modelEntity;
  gz::sim::Entity linkEntity;

  std::unique_ptr<Dynamics> dynamics;
  std::unique_ptr<LiftDrag> liftdrag;

  double inputCommand{0.0};
  double angle{0.0};

  gz::math::Vector3d currentVelocity;
  gz::math::Vector3d finForce;
};

} // namespace uuv_gz_plugins

#endif
