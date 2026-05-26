#ifndef UNDERWATER_CURRENT_GZ_SIM8_PLUGIN_HH_
#define UNDERWATER_CURRENT_GZ_SIM8_PLUGIN_HH_

#include <map>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include "GaussMarkovProcess.hh"

namespace uuv_gz_sim
{

class UnderwaterCurrentROSPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  UnderwaterCurrentROSPlugin() = default;
  ~UnderwaterCurrentROSPlugin() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

  // Replaced ROS services with runtime setters (ROS2 bridge compatible)
  void UpdateCurrentVelocityModel();
  void UpdateCurrentHorzAngleModel();
  void UpdateCurrentVertAngleModel();

  void UpdateCurrentVelocity();
  void UpdateHorzAngle();
  void UpdateVertAngle();

private:
  void PublishCurrentVelocity();

private:
  gz::transport::Node node;

  std::string ns;

  std::string currentVelocityTopic;

  std::map<std::string, std::string> serviceMap;

  GaussMarkovProcess currentVelModel;
  GaussMarkovProcess currentHorzAngleModel;
  GaussMarkovProcess currentVertAngleModel;

  gz::math::Vector3d currentVelocity;
};

} // namespace uuv_gz_sim

#endif
