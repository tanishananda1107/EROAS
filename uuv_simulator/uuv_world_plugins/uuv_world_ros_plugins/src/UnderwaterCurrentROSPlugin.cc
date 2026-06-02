// Copyright (c) 2016 The UUV Simulator Authors.
// Converted to ROS2 + Gazebo Harmonic (gz-sim8)

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <uuv_world_ros_plugins_msgs/srv/set_current_direction.hpp>
#include <uuv_world_ros_plugins_msgs/srv/set_current_velocity.hpp>
#include <uuv_world_ros_plugins_msgs/srv/get_current_model.hpp>
#include <uuv_world_ros_plugins_msgs/srv/set_current_model.hpp>

namespace uuv_simulator_ros
{

class UnderwaterCurrentROSPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  UnderwaterCurrentROSPlugin()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    this->rosNode =
      std::make_shared<rclcpp::Node>(
        "underwater_current_ros_plugin");
  }

  ~UnderwaterCurrentROSPlugin() override = default;

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/) override
  {
    (void)_entity;
    (void)_ecm;

    std::string ns = "";

    if (_sdf->HasElement("namespace"))
      ns = _sdf->Get<std::string>("namespace");

    this->flowVelocityPub =
      this->rosNode->create_publisher<
        geometry_msgs::msg::TwistStamped>(
        ns + "/current_velocity",
        10);

    this->setVelocitySrv =
      this->rosNode->create_service<
        uuv_world_ros_plugins_msgs::srv::SetCurrentVelocity>(
        ns + "/set_current_velocity",
        std::bind(
          &UnderwaterCurrentROSPlugin::UpdateCurrentVelocity,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    this->setHorzSrv =
      this->rosNode->create_service<
        uuv_world_ros_plugins_msgs::srv::SetCurrentDirection>(
        ns + "/set_current_horz_angle",
        std::bind(
          &UnderwaterCurrentROSPlugin::UpdateHorzAngle,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    this->setVertSrv =
      this->rosNode->create_service<
        uuv_world_ros_plugins_msgs::srv::SetCurrentDirection>(
        ns + "/set_current_vert_angle",
        std::bind(
          &UnderwaterCurrentROSPlugin::UpdateVertAngle,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    RCLCPP_INFO(
      this->rosNode->get_logger(),
      "UnderwaterCurrentROSPlugin loaded");
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &/*_ecm*/) override
  {
    if (_info.paused)
      return;

    geometry_msgs::msg::TwistStamped msg;

    msg.header.stamp = this->rosNode->now();
    msg.header.frame_id = "world";

    msg.twist.linear.x = this->currentVelocity.X();
    msg.twist.linear.y = this->currentVelocity.Y();
    msg.twist.linear.z = this->currentVelocity.Z();

    this->flowVelocityPub->publish(msg);
  }

private:
  std::shared_ptr<rclcpp::Node> rosNode;

  rclcpp::Publisher<
    geometry_msgs::msg::TwistStamped>::SharedPtr flowVelocityPub;

  rclcpp::Service<
    uuv_world_ros_plugins_msgs::srv::SetCurrentVelocity>::SharedPtr
      setVelocitySrv;

  rclcpp::Service<
    uuv_world_ros_plugins_msgs::srv::SetCurrentDirection>::SharedPtr
      setHorzSrv;

  rclcpp::Service<
    uuv_world_ros_plugins_msgs::srv::SetCurrentDirection>::SharedPtr
      setVertSrv;

  gz::math::Vector3d currentVelocity{0, 0, 0};

  double horizontalAngle{0.0};
  double verticalAngle{0.0};

  void UpdateCurrentVelocity(
      const std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          SetCurrentVelocity::Request> req,
      std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          SetCurrentVelocity::Response> res)
  {
    double vel = req->velocity;

    this->horizontalAngle = req->horizontal_angle;
    this->verticalAngle = req->vertical_angle;

    this->currentVelocity.X(
      vel * cos(this->horizontalAngle));

    this->currentVelocity.Y(
      vel * sin(this->horizontalAngle));

    this->currentVelocity.Z(
      vel * sin(this->verticalAngle));

    RCLCPP_INFO(
      this->rosNode->get_logger(),
      "Current velocity updated");

    res->success = true;
  }

  void UpdateHorzAngle(
      const std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          SetCurrentDirection::Request> req,
      std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          SetCurrentDirection::Response> res)
  {
    this->horizontalAngle = req->angle;
    res->success = true;
  }

  void UpdateVertAngle(
      const std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          SetCurrentDirection::Request> req,
      std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          SetCurrentDirection::Response> res)
  {
    this->verticalAngle = req->angle;
    res->success = true;
  }
};
} // namespace uuv_simulator_ros

GZ_ADD_PLUGIN(
  uuv_simulator_ros::UnderwaterCurrentROSPlugin,
  gz::sim::System,
  uuv_simulator_ros::UnderwaterCurrentROSPlugin::ISystemConfigure,
  uuv_simulator_ros::UnderwaterCurrentROSPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  uuv_simulator_ros::UnderwaterCurrentROSPlugin,
  "uuv_simulator_ros::UnderwaterCurrentROSPlugin")
