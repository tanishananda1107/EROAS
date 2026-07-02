#ifndef UUV_GZ_SIM_UNDERWATER_CURRENT_PLUGIN_HH_
#define UUV_GZ_SIM_UNDERWATER_CURRENT_PLUGIN_HH_

#include <map>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>

#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include <sdf/sdf.hh>

#include "uuv_world_plugins/GaussMarkovProcess.hh"

namespace uuv_gz_sim
{

class UnderwaterCurrentPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  UnderwaterCurrentPlugin() = default;
  ~UnderwaterCurrentPlugin() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

protected:
  void UpdateCurrent(double _time);

private:
  gz::sim::Entity worldEntity;

  gz::transport::Node node;

  std::string ns;
  std::string currentVelocityTopic;

  bool hasSurface{false};

  GaussMarkovProcess currentVelModel;
  GaussMarkovProcess currentHorzAngleModel;
  GaussMarkovProcess currentVertAngleModel;

  double lastUpdate{0.0};

  gz::math::Vector3d currentVelocity;
};

} // namespace uuv_gz_sim

#endif
