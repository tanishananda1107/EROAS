// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __UUV_MAGNETOMETER_ROS_PLUGIN_HH__
#define __UUV_MAGNETOMETER_ROS_PLUGIN_HH__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <gz/math/Vector3.hh>

namespace gz { namespace sim {

struct MagnetometerParameters {
  double intensity, heading, declination, inclination, noiseXY, noiseZ, turnOnBias;
};

class MagnetometerROSPlugin : public ROSBaseModelPlugin {
public:
  MagnetometerROSPlugin();
  virtual ~MagnetometerROSPlugin();
  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;

protected:
  bool OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override;
  MagnetometerParameters parameters;
  gz::math::Vector3d magneticFieldWorld, turnOnBias, measMagneticField;
  sensor_msgs::msg::MagneticField rosMsg;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magPub;
};

}}  // namespace gz::sim
#endif
