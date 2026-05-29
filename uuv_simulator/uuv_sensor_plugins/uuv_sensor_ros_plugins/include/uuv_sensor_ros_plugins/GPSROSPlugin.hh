// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __GPS_SENSOR_ROS_PLUGIN_HH__
#define __GPS_SENSOR_ROS_PLUGIN_HH__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <uuv_sensor_ros_plugins/ROSBaseSensorPlugin.hh>

namespace gz { namespace sim {

class GPSROSPlugin : public ROSBaseSensorPlugin {
public:
  GPSROSPlugin();
  virtual ~GPSROSPlugin();
  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;
  bool OnUpdateGPS(const UpdateInfo& _info, EntityComponentManager& _ecm);

protected:
  bool OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override;
  sensor_msgs::msg::NavSatFix gpsMessage;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsPub;
};

}}  // namespace gz::sim
#endif
