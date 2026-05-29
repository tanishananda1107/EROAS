// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __UUV_CHEMICAL_PARTICLE_CONCENTRATION_ROS_PLUGIN_HH__
#define __UUV_CHEMICAL_PARTICLE_CONCENTRATION_ROS_PLUGIN_HH__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <uuv_sensor_ros_plugins_msgs/msg/chemical_particle_concentration.hpp>
#include <uuv_sensor_ros_plugins_msgs/msg/salinity.hpp>

namespace gz { namespace sim {

class CPCROSPlugin : public ROSBaseModelPlugin {
public:
  CPCROSPlugin();
  virtual ~CPCROSPlugin();
  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;

protected:
  bool OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override;
  void OnPlumeParticlesUpdate(const sensor_msgs::msg::PointCloud::SharedPtr _msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr particlesSub;
  rclcpp::Publisher<uuv_sensor_ros_plugins_msgs::msg::Salinity>::SharedPtr salinityPub;
  bool updatingCloud{false};
  double gamma{0.0}, gain{0.0}, smoothingLength{0.0};
  rclcpp::Time lastUpdateTimestamp;
  uuv_sensor_ros_plugins_msgs::msg::ChemicalParticleConcentration outputMsg;
  uuv_sensor_ros_plugins_msgs::msg::Salinity salinityMsg;
  double waterSalinityValue{0.0}, plumeSalinityValue{0.0};
};

}}  // namespace gz::sim
#endif
