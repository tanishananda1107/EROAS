// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __ROS_BASE_PLUGIN_HH__
#define __ROS_BASE_PLUGIN_HH__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <uuv_sensor_ros_plugins/Common.hh>
#include <uuv_sensor_ros_plugins_msgs/srv/change_sensor_state.hpp>
#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <map>

namespace gz { namespace sim {

class ROSBasePlugin {
public:
  ROSBasePlugin();
  virtual ~ROSBasePlugin();
  bool InitBasePlugin(sdf::ElementPtr _sdf);
  virtual bool OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) = 0;
  bool AddNoiseModel(const std::string& _name, double _sigma);

protected:
  std::string robotNamespace, sensorOutputTopic, referenceFrameID;
  std::chrono::steady_clock::duration lastMeasurementTime{0};
  double updateRate{0.0}, noiseSigma{0.0}, noiseAmp{0.0};
  bool gazeboMsgEnabled{false}, isReferenceInit{false};
  std::default_random_engine rndGen;
  std::map<std::string, std::normal_distribution<double>> noiseModels;
  std_msgs::msg::Bool isOn;
  std::shared_ptr<rclcpp::Node> rosNode;
  transport::Node gzNode;
  rclcpp::Publisher<rclcpp::SerializedMessage>::SharedPtr rosSensorOutputPub;
  transport::Node::Publisher gzSensorOutputPub;
  rclcpp::Service<uuv_sensor_ros_plugins_msgs::srv::ChangeSensorState>::SharedPtr changeSensorSrv;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pluginStatePub;
  math::Pose3d referenceFrame;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfStaticSub;
  Entity referenceLink{kNullEntity};

  bool IsOn();
  void PublishState();
  void ChangeSensorState(
    const std::shared_ptr<uuv_sensor_ros_plugins_msgs::srv::ChangeSensorState::Request> _req,
    std::shared_ptr<uuv_sensor_ros_plugins_msgs::srv::ChangeSensorState::Response> _res);
  void GetTFMessage(const tf2_msgs::msg::TFMessage::SharedPtr _msg);
  double GetGaussianNoise(double _amp);
  double GetGaussianNoise(const std::string& _name, double _amp);
  bool EnableMeasurement(const UpdateInfo& _info) const;
  void UpdateReferenceFramePose(EntityComponentManager& _ecm);
};

}}  // namespace gz::sim
#endif  // __ROS_BASE_PLUGIN_HH__
