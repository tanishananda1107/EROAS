// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __ROS_BASE_SENSOR_PLUGIN_HH__
#define __ROS_BASE_SENSOR_PLUGIN_HH__

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sensors/Sensor.hh>
#include <uuv_sensor_ros_plugins/ROSBasePlugin.hh>
#include <memory>

namespace gz { namespace sim {

class ROSBaseSensorPlugin
  : public ROSBasePlugin, public System,
    public ISystemConfigure, public ISystemUpdate
{
public:
  ROSBaseSensorPlugin();
  virtual ~ROSBaseSensorPlugin();

  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;
  void Update(const UpdateInfo& _info, EntityComponentManager& _ecm) override;

protected:
  virtual bool OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm);
  Entity sensorEntity{kNullEntity};
  std::shared_ptr<gz::sensors::Sensor> parentSensor;
};

}}  // namespace gz::sim
#endif  // __ROS_BASE_SENSOR_PLUGIN_HH__
