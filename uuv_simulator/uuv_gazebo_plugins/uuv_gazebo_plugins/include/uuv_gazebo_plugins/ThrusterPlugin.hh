#ifndef UUV_GZ_THRUSTER_PLUGIN_HH_
#define UUV_GZ_THRUSTER_PLUGIN_HH_

#include <memory>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Entity.hh>

#include <gz/math/Vector3.hh>

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
  ThrusterPlugin() = default;
  virtual ~ThrusterPlugin() = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

protected:
  void UpdateInput(double _cmd);

protected:
  gz::sim::Entity modelEntity;
  gz::sim::Entity linkEntity;

  std::shared_ptr<Dynamics> thrusterDynamics;
  std::shared_ptr<ConversionFunction> conversionFunction;

  double inputCommand{0.0};
  double thrustForce{0.0};

  double clampMin{-1.0};
  double clampMax{1.0};

  double thrustMin{-10.0};
  double thrustMax{10.0};

  bool isOn{true};
};

} // namespace uuv_gz_plugins

#endif
