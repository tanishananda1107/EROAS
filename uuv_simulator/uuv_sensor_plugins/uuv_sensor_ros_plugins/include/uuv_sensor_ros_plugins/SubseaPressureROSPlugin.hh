// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __UUV_SUBSEA_PRESSURE_ROS_PLUGIN_HH__
#define __UUV_SUBSEA_PRESSURE_ROS_PLUGIN_HH__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>

namespace gz { namespace sim {

class SubseaPressureROSPlugin : public ROSBaseModelPlugin {
public:
  SubseaPressureROSPlugin();
  virtual ~SubseaPressureROSPlugin();
  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;

protected:
  bool OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override;
  double saturation{0.0}, standardPressure{101325.0}, kPaPerM{9.80638};
  bool estimateDepth{false};
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressurePub;
};

}}  // namespace gz::sim
#endif
