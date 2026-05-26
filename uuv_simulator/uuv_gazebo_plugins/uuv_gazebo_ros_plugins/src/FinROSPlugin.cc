// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

#include <uuv_gazebo_ros_plugins/FinROSPlugin.hh>
#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>

namespace uuv_simulator_ros
{

/////////////////////////////////////////////////
FinROSPlugin::FinROSPlugin()
{
  rosPublishPeriod = std::chrono::milliseconds(50);
  lastRosPublishTime = std::chrono::steady_clock::now();
}

/////////////////////////////////////////////////
FinROSPlugin::~FinROSPlugin() = default;

/////////////////////////////////////////////////
void FinROSPlugin::SetReference(
  const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr &_msg)
{
  if (std::isnan(_msg->data))
  {
    RCLCPP_WARN(rosNode->get_logger(), "FinROSPlugin: Ignoring nan command");
    return;
  }
  this->inputCommand = _msg->data;
}

/////////////////////////////////////////////////
std::chrono::nanoseconds FinROSPlugin::GetRosPublishPeriod()
{
  return rosPublishPeriod;
}

/////////////////////////////////////////////////
void FinROSPlugin::SetRosPublishRate(double _hz)
{
  if (_hz > 0.0)
    rosPublishPeriod = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / _hz));
  else
    rosPublishPeriod = std::chrono::nanoseconds(0);
}

/////////////////////////////////////////////////
void FinROSPlugin::Init() { FinPlugin::Init(); }

/////////////////////////////////////////////////
void FinROSPlugin::Reset()
{
  lastRosPublishTime = std::chrono::steady_clock::now();
}

/////////////////////////////////////////////////
void FinROSPlugin::Load(gz::sim::EntityComponentManager &_ecm,
                        const std::shared_ptr<const sdf::Element> &_sdf)
{
  try {
    FinPlugin::Load(_ecm, _sdf);
  } catch (const std::exception &e) {
    gzerr << "Error loading FinPlugin: " << e.what() << "\n";
    return;
  }

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  rosNode = std::make_shared<rclcpp::Node>("fin_ros_plugin_" +
    std::to_string(this->finID));

  subReference =
    rosNode->create_subscription<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>(
      this->commandTopic, 10,
      [this](const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr msg) {
        SetReference(msg);
      });

  pubState =
    rosNode->create_publisher<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>(
      this->angleTopic, 10);

  std::string wrenchTopic;
  if (_sdf->HasElement("wrench_topic"))
    wrenchTopic = _sdf->Get<std::string>("wrench_topic");
  else
    wrenchTopic = this->topicPrefix + "wrench_topic";

  pubFinForce =
    rosNode->create_publisher<geometry_msgs::msg::WrenchStamped>(wrenchTopic, 10);

  std::string liftDragSrv = this->topicPrefix + "get_lift_drag_params";
  services["get_lift_drag_params"] =
    rosNode->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetListParam>(
      liftDragSrv,
      [this](
        uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Request::SharedPtr req,
        uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Response::SharedPtr res) {
        GetLiftDragParams(req, res);
      });

  gzmsg << "Fin #" << this->finID << " initialized\n"
        << "\t- Input command topic: " << this->commandTopic << "\n"
        << "\t- Output topic: " << this->angleTopic << std::endl;
}

/////////////////////////////////////////////////
void FinROSPlugin::RosPublishStates()
{
  auto now = std::chrono::steady_clock::now();
  if (now - lastRosPublishTime < rosPublishPeriod)
    return;

  lastRosPublishTime = now;
  rclcpp::Time stamp = rosNode->now();

  uuv_gazebo_ros_plugins_msgs::msg::FloatStamped stateMsg;
  stateMsg.header.stamp = stamp;
  stateMsg.header.frame_id = this->linkName;
  stateMsg.data = this->angle;
  pubState->publish(stateMsg);

  geometry_msgs::msg::WrenchStamped wrenchMsg;
  wrenchMsg.header.stamp = stamp;
  wrenchMsg.header.frame_id = this->linkName;
  wrenchMsg.wrench.force.x = this->finForce.X();
  wrenchMsg.wrench.force.y = this->finForce.Y();
  wrenchMsg.wrench.force.z = this->finForce.Z();
  pubFinForce->publish(wrenchMsg);
}

/////////////////////////////////////////////////
bool FinROSPlugin::GetLiftDragParams(
  uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Request::SharedPtr,
  uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Response::SharedPtr _res)
{
  _res->description = this->liftdrag->GetType();
  for (auto &item : this->liftdrag->GetListParams())
  {
    _res->tags.push_back(item.first);
    _res->data.push_back(item.second);
  }
  return true;
}

} // namespace uuv_simulator_ros

GZ_ADD_PLUGIN(uuv_simulator_ros::FinROSPlugin,
              gz::sim::System,
              uuv_simulator_ros::FinROSPlugin::ISystemConfigure,
              uuv_simulator_ros::FinROSPlugin::ISystemUpdate)
