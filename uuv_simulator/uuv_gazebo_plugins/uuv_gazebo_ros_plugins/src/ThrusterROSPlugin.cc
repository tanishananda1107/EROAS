// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

#include <uuv_gazebo_ros_plugins/ThrusterROSPlugin.hh>
#include <gz/plugin/Register.hh>

namespace uuv_simulator_ros
{

/////////////////////////////////////////////////
ThrusterROSPlugin::ThrusterROSPlugin()
{
  rosPublishPeriod = std::chrono::milliseconds(50);
  lastRosPublishTime = std::chrono::steady_clock::now();
}

/////////////////////////////////////////////////
ThrusterROSPlugin::~ThrusterROSPlugin() = default;

/////////////////////////////////////////////////
void ThrusterROSPlugin::SetThrustReference(
  const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr &_msg)
{
  if (std::isnan(_msg->data))
  {
    RCLCPP_WARN(rosNode->get_logger(), "ThrusterROSPlugin: Ignoring nan command");
    return;
  }
  this->inputCommand = _msg->data;
}

/////////////////////////////////////////////////
std::chrono::nanoseconds ThrusterROSPlugin::GetRosPublishPeriod()
{
  return rosPublishPeriod;
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::SetRosPublishRate(double _hz)
{
  rosPublishPeriod = (_hz > 0.0) ?
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / _hz)) :
    std::chrono::nanoseconds(0);
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::Init()  { ThrusterPlugin::Init(); }
void ThrusterROSPlugin::Reset() { lastRosPublishTime = std::chrono::steady_clock::now(); }

/////////////////////////////////////////////////
void ThrusterROSPlugin::Load(gz::sim::EntityComponentManager &_ecm,
                             const std::shared_ptr<const sdf::Element> &_sdf)
{
  try {
    ThrusterPlugin::Load(_ecm, _sdf);
  } catch (const std::exception &e) {
    gzerr << "Error loading ThrusterPlugin: " << e.what() << "\n";
    return;
  }

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  rosNode = std::make_shared<rclcpp::Node>(
    "thruster_ros_plugin_" + std::to_string(this->thrusterID));

  auto mkSrv = [&](auto srvName, auto handler) {
    services[srvName] = rosNode->create_service
      std::remove_pointer_t<decltype(handler)>>(
        this->topicPrefix + srvName, handler);
  };

  services["set_thrust_force_efficiency"] = rosNode->create_service
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency>(
      this->topicPrefix + "set_thrust_force_efficiency",
      [this](auto req, auto res){ SetThrustForceEfficiency(req, res); });

  services["get_thrust_force_efficiency"] = rosNode->create_service
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency>(
      this->topicPrefix + "get_thrust_force_efficiency",
      [this](auto req, auto res){ GetThrustForceEfficiency(req, res); });

  services["set_dynamic_state_efficiency"] = rosNode->create_service
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency>(
      this->topicPrefix + "set_dynamic_state_efficiency",
      [this](auto req, auto res){ SetDynamicStateEfficiency(req, res); });

  services["get_dynamic_state_efficiency"] = rosNode->create_service
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency>(
      this->topicPrefix + "get_dynamic_state_efficiency",
      [this](auto req, auto res){ GetDynamicStateEfficiency(req, res); });

  services["set_thruster_state"] = rosNode->create_service
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState>(
      this->topicPrefix + "set_thruster_state",
      [this](auto req, auto res){ SetThrusterState(req, res); });

  services["get_thruster_state"] = rosNode->create_service
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState>(
      this->topicPrefix + "get_thruster_state",
      [this](auto req, auto res){ GetThrusterState(req, res); });

  services["get_thruster_conversion_fcn"] = rosNode->create_service
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn>(
      this->topicPrefix + "get_thruster_conversion_fcn",
      [this](auto req, auto res){ GetThrusterConversionFcn(req, res); });

  subThrustReference =
    rosNode->create_subscription<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>(
      this->commandTopic, 10,
      [this](const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr msg) {
        SetThrustReference(msg);
      });

  pubThrust = rosNode->create_publisher
    uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>(this->thrustTopic, 10);

  pubThrustWrench = rosNode->create_publisher
    geometry_msgs::msg::WrenchStamped>(this->thrustTopic + "_wrench", 10);

  pubThrusterState = rosNode->create_publisher<std_msgs::msg::Bool>(
    this->topicPrefix + "is_on", 1);

  pubThrustForceEff = rosNode->create_publisher<std_msgs::msg::Float64>(
    this->topicPrefix + "thrust_efficiency", 1);

  pubDynamicStateEff = rosNode->create_publisher<std_msgs::msg::Float64>(
    this->topicPrefix + "dynamic_state_efficiency", 1);

  gzmsg << "Thruster #" << this->thrusterID << " initialized\n"
        << "\t- Input command topic: " << this->commandTopic << "\n"
        << "\t- Thrust output topic: " << this->thrustTopic << std::endl;
}

/////////////////////////////////////////////////
void ThrusterROSPlugin::RosPublishStates()
{
  auto now = std::chrono::steady_clock::now();
  if (now - lastRosPublishTime < rosPublishPeriod)
    return;
  lastRosPublishTime = now;

  rclcpp::Time stamp = rosNode->now();

  uuv_gazebo_ros_plugins_msgs::msg::FloatStamped thrustMsg;
  thrustMsg.header.stamp = stamp;
  thrustMsg.header.frame_id = this->linkName;
  thrustMsg.data = this->thrustForce;
  pubThrust->publish(thrustMsg);

  geometry_msgs::msg::WrenchStamped wrenchMsg;
  wrenchMsg.header.stamp = stamp;
  wrenchMsg.header.frame_id = this->linkName;
  gz::math::Vector3d tv = this->thrustForce * this->thrusterAxis;
  wrenchMsg.wrench.force.x = tv.X();
  wrenchMsg.wrench.force.y = tv.Y();
  wrenchMsg.wrench.force.z = tv.Z();
  pubThrustWrench->publish(wrenchMsg);

  std_msgs::msg::Bool isOnMsg;
  isOnMsg.data = this->isOn;
  pubThrusterState->publish(isOnMsg);

  std_msgs::msg::Float64 effMsg;
  effMsg.data = this->thrustEfficiency;
  pubThrustForceEff->publish(effMsg);

  std_msgs::msg::Float64 dynMsg;
  dynMsg.data = this->propellerEfficiency;
  pubDynamicStateEff->publish(dynMsg);
}

/////////////////////////////////////////////////
bool ThrusterROSPlugin::SetThrustForceEfficiency(
  uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::SharedPtr _res)
{
  if (_req->efficiency < 0.0 || _req->efficiency > 1.0)
    _res->success = false;
  else
  {
    this->thrustEfficiency = _req->efficiency;
    _res->success = true;
    gzmsg << "Setting thrust efficiency=" << _req->efficiency * 100 << "%\n";
  }
  return true;
}

bool ThrusterROSPlugin::GetThrustForceEfficiency(
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Request::SharedPtr,
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::SharedPtr _res)
{
  _res->efficiency = this->thrustEfficiency;
  return true;
}

bool ThrusterROSPlugin::SetDynamicStateEfficiency(
  uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::SharedPtr _res)
{
  if (_req->efficiency < 0.0 || _req->efficiency > 1.0)
    _res->success = false;
  else
  {
    this->propellerEfficiency = _req->efficiency;
    _res->success = true;
    gzmsg << "Setting propeller efficiency=" << _req->efficiency * 100 << "%\n";
  }
  return true;
}

bool ThrusterROSPlugin::GetDynamicStateEfficiency(
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Request::SharedPtr,
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::SharedPtr _res)
{
  _res->efficiency = this->propellerEfficiency;
  return true;
}

bool ThrusterROSPlugin::SetThrusterState(
  uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Response::SharedPtr _res)
{
  this->isOn = _req->on;
  gzmsg << "Thruster " << (this->isOn ? "ON" : "OFF") << "\n";
  _res->success = true;
  return true;
}

bool ThrusterROSPlugin::GetThrusterState(
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Request::SharedPtr,
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Response::SharedPtr _res)
{
  _res->is_on = this->isOn;
  return true;
}

bool ThrusterROSPlugin::GetThrusterConversionFcn(
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Request::SharedPtr,
  uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Response::SharedPtr _res)
{
  _res->fcn.function_name = this->conversionFunction->GetType();
  double param;

  if (_res->fcn.function_name == "Basic")
  {
    _res->fcn.tags.push_back("rotor_constant");
    this->conversionFunction->GetParam("rotor_constant", param);
    _res->fcn.data.push_back(param);
  }
  else if (_res->fcn.function_name == "Bessa")
  {
    for (const auto &tag : {"rotor_constant_l","rotor_constant_r","delta_l","delta_r"})
    {
      _res->fcn.tags.push_back(tag);
      this->conversionFunction->GetParam(tag, param);
      _res->fcn.data.push_back(param);
    }
  }
  else if (_res->fcn.function_name == "LinearInterp")
  {
    for (auto &item : this->conversionFunction->GetTable())
    {
      _res->fcn.lookup_table_input.push_back(item.first);
      _res->fcn.lookup_table_output.push_back(item.second);
    }
  }
  return true;
}

} // namespace uuv_simulator_ros

GZ_ADD_PLUGIN(uuv_simulator_ros::ThrusterROSPlugin,
              gz::sim::System,
              uuv_simulator_ros::ThrusterROSPlugin::ISystemConfigure,
              uuv_simulator_ros::ThrusterROSPlugin::ISystemUpdate)
