#ifndef SC_GZ_SIM8_INTERFACE_PLUGIN_HH_
#define SC_GZ_SIM8_INTERFACE_PLUGIN_HH_

#include <map>
#include <string>

#include <sdf/sdf.hh>

#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

namespace uuv_gz_sim
{

class SphericalCoordinatesROSInterfacePlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure
{
public:
  SphericalCoordinatesROSInterfacePlugin() = default;
  ~SphericalCoordinatesROSInterfacePlugin() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  // Replaced ROS services with Gazebo Transport / ROS2 bridge topics
  void GetOriginSphericalCoord();
  void SetOriginSphericalCoord();

  void TransformToSphericalCoord();
  void TransformFromSphericalCoord();

private:
  gz::sim::Entity worldEntity;

  gz::transport::Node node;

  std::string ns;

  // Replace ROS service map with transport request handling
  std::map<std::string, std::string> serviceMap;
};

} // namespace uuv_gz_sim

#endif
