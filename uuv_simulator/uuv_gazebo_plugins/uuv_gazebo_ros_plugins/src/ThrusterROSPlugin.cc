#include <uuv_gazebo_ros_plugins/ThrusterROSPlugin.hh>

#include <cmath>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

namespace uuv_simulator_ros
{

ThrusterROSPlugin::ThrusterROSPlugin()
    : rosPublishPeriod(std::chrono::milliseconds(50)),
      lastRosPublishTime(std::chrono::steady_clock::now())
{
}

ThrusterROSPlugin::~ThrusterROSPlugin() = default;

void ThrusterROSPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
  uuv_gz_plugins::ThrusterPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);

  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  this->rosNode = std::make_shared<rclcpp::Node>(
      "thruster_ros_plugin_" + std::to_string(this->thrusterID));

  this->topicPrefix = "/" + this->model.Name(_ecm) + "/thrusters/thruster_" +
                      std::to_string(this->thrusterID) + "/";
  this->commandTopic = this->topicPrefix + "input";
  this->thrustTopic = this->topicPrefix + "thrust";

  using SetEfficiency =
      uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency;
  using GetEfficiency =
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency;
  using SetState = uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState;
  using GetState = uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState;
  using GetConversion =
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn;

  this->services["set_thrust_force_efficiency"] =
      this->rosNode->create_service<SetEfficiency>(
          this->topicPrefix + "set_thrust_force_efficiency",
          [this](SetEfficiency::Request::SharedPtr req,
                 SetEfficiency::Response::SharedPtr res) {
            this->SetThrustForceEfficiency(req, res);
          });

  this->services["get_thrust_force_efficiency"] =
      this->rosNode->create_service<GetEfficiency>(
          this->topicPrefix + "get_thrust_force_efficiency",
          [this](GetEfficiency::Request::SharedPtr req,
                 GetEfficiency::Response::SharedPtr res) {
            this->GetThrustForceEfficiency(req, res);
          });

  this->services["set_dynamic_state_efficiency"] =
      this->rosNode->create_service<SetEfficiency>(
          this->topicPrefix + "set_dynamic_state_efficiency",
          [this](SetEfficiency::Request::SharedPtr req,
                 SetEfficiency::Response::SharedPtr res) {
            this->SetDynamicStateEfficiency(req, res);
          });

  this->services["get_dynamic_state_efficiency"] =
      this->rosNode->create_service<GetEfficiency>(
          this->topicPrefix + "get_dynamic_state_efficiency",
          [this](GetEfficiency::Request::SharedPtr req,
                 GetEfficiency::Response::SharedPtr res) {
            this->GetDynamicStateEfficiency(req, res);
          });

  this->services["set_thruster_state"] =
      this->rosNode->create_service<SetState>(
          this->topicPrefix + "set_thruster_state",
          [this](SetState::Request::SharedPtr req,
                 SetState::Response::SharedPtr res) {
            this->SetThrusterState(req, res);
          });

  this->services["get_thruster_state"] =
      this->rosNode->create_service<GetState>(
          this->topicPrefix + "get_thruster_state",
          [this](GetState::Request::SharedPtr req,
                 GetState::Response::SharedPtr res) {
            this->GetThrusterState(req, res);
          });

  this->services["get_thruster_conversion_fcn"] =
      this->rosNode->create_service<GetConversion>(
          this->topicPrefix + "get_thruster_conversion_fcn",
          [this](GetConversion::Request::SharedPtr req,
                 GetConversion::Response::SharedPtr res) {
            this->GetThrusterConversionFcn(req, res);
          });

  this->subThrustReference =
      this->rosNode
          ->create_subscription<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>(
              this->commandTopic, 10,
              [this](const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::
                         SharedPtr msg) { this->SetThrustReference(msg); });

  this->pubThrust =
      this->rosNode
          ->create_publisher<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>(
              this->thrustTopic, 10);
  this->pubThrustWrench =
      this->rosNode->create_publisher<geometry_msgs::msg::WrenchStamped>(
          this->thrustTopic + "_wrench", 10);
  this->pubThrusterState =
      this->rosNode->create_publisher<std_msgs::msg::Bool>(
          this->topicPrefix + "is_on", 1);
  this->pubThrustForceEff =
      this->rosNode->create_publisher<std_msgs::msg::Float64>(
          this->topicPrefix + "thrust_efficiency", 1);
  this->pubDynamicStateEff =
      this->rosNode->create_publisher<std_msgs::msg::Float64>(
          this->topicPrefix + "dynamic_state_efficiency", 1);

  gzmsg << "Thruster #" << this->thrusterID << " ROS wrapper initialized\n"
        << "\t- Input command topic: " << this->commandTopic << "\n"
        << "\t- Thrust output topic: " << this->thrustTopic << "\n";
}

void ThrusterROSPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                                  gz::sim::EntityComponentManager &_ecm)
{
  uuv_gz_plugins::ThrusterPlugin::PreUpdate(_info, _ecm);

  if (this->rosNode && rclcpp::ok())
  {
    rclcpp::spin_some(this->rosNode);
    this->RosPublishStates();
  }
}

void ThrusterROSPlugin::SetThrustReference(
    const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr &_msg)
{
  if (std::isnan(_msg->data))
  {
    RCLCPP_WARN(this->rosNode->get_logger(),
                "ThrusterROSPlugin: Ignoring NaN command");
    return;
  }

  this->inputCommand = _msg->data;
}

std::chrono::nanoseconds ThrusterROSPlugin::GetRosPublishPeriod() const
{
  return this->rosPublishPeriod;
}

void ThrusterROSPlugin::SetRosPublishRate(double _hz)
{
  if (_hz > 0.0)
  {
    this->rosPublishPeriod =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / _hz));
  }
  else
  {
    this->rosPublishPeriod = std::chrono::nanoseconds(0);
  }
}

void ThrusterROSPlugin::RosPublishStates()
{
  const auto now = std::chrono::steady_clock::now();
  if (this->rosPublishPeriod.count() > 0 &&
      now - this->lastRosPublishTime < this->rosPublishPeriod)
  {
    return;
  }
  this->lastRosPublishTime = now;

  uuv_gazebo_ros_plugins_msgs::msg::FloatStamped thrustMsg;
  thrustMsg.header.stamp = this->rosNode->now();
  thrustMsg.header.frame_id = this->linkName;
  thrustMsg.data = this->thrustForce;
  this->pubThrust->publish(thrustMsg);

  geometry_msgs::msg::WrenchStamped wrenchMsg;
  wrenchMsg.header.stamp = thrustMsg.header.stamp;
  wrenchMsg.header.frame_id = this->linkName;
  const gz::math::Vector3d thrustVector =
      this->thrustForce * this->thrusterAxis;
  wrenchMsg.wrench.force.x = thrustVector.X();
  wrenchMsg.wrench.force.y = thrustVector.Y();
  wrenchMsg.wrench.force.z = thrustVector.Z();
  this->pubThrustWrench->publish(wrenchMsg);

  std_msgs::msg::Bool isOnMsg;
  isOnMsg.data = this->isOn;
  this->pubThrusterState->publish(isOnMsg);

  std_msgs::msg::Float64 thrustEffMsg;
  thrustEffMsg.data = this->thrustEfficiency;
  this->pubThrustForceEff->publish(thrustEffMsg);

  std_msgs::msg::Float64 dynamicEffMsg;
  dynamicEffMsg.data = this->propellerEfficiency;
  this->pubDynamicStateEff->publish(dynamicEffMsg);
}

bool ThrusterROSPlugin::SetThrustForceEfficiency(
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::SharedPtr
        _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::SharedPtr
        _res)
{
  if (_req->efficiency < 0.0 || _req->efficiency > 1.0)
  {
    _res->success = false;
  }
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
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::SharedPtr
        _res)
{
  _res->efficiency = this->thrustEfficiency;
  return true;
}

bool ThrusterROSPlugin::SetDynamicStateEfficiency(
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::SharedPtr
        _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::SharedPtr
        _res)
{
  if (_req->efficiency < 0.0 || _req->efficiency > 1.0)
  {
    _res->success = false;
  }
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
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::SharedPtr
        _res)
{
  _res->efficiency = this->propellerEfficiency;
  return true;
}

bool ThrusterROSPlugin::SetThrusterState(
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Response::SharedPtr
        _res)
{
  this->isOn = _req->on;
  _res->success = true;
  gzmsg << "Thruster " << (this->isOn ? "ON" : "OFF") << "\n";
  return true;
}

bool ThrusterROSPlugin::GetThrusterState(
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Request::SharedPtr,
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Response::SharedPtr
        _res)
{
  _res->is_on = this->isOn;
  return true;
}

bool ThrusterROSPlugin::GetThrusterConversionFcn(
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Request::
        SharedPtr,
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Response::
        SharedPtr _res)
{
  _res->fcn.function_name = this->conversionFunction->GetType();
  double param = 0.0;

  if (_res->fcn.function_name == "Basic")
  {
    _res->fcn.tags.push_back("rotor_constant");
    this->conversionFunction->GetParam("rotor_constant", param);
    _res->fcn.data.push_back(param);
  }
  else if (_res->fcn.function_name == "Bessa")
  {
    for (const auto &tag :
         {"rotor_constant_l", "rotor_constant_r", "delta_l", "delta_r"})
    {
      _res->fcn.tags.push_back(tag);
      this->conversionFunction->GetParam(tag, param);
      _res->fcn.data.push_back(param);
    }
  }
  else if (_res->fcn.function_name == "LinearInterp")
  {
    for (const auto &item : this->conversionFunction->GetTable())
    {
      _res->fcn.lookup_table_input.push_back(item.first);
      _res->fcn.lookup_table_output.push_back(item.second);
    }
  }
  return true;
}

}  // namespace uuv_simulator_ros

GZ_ADD_PLUGIN(uuv_simulator_ros::ThrusterROSPlugin,
              gz::sim::System,
              uuv_simulator_ros::ThrusterROSPlugin::ISystemConfigure,
              uuv_simulator_ros::ThrusterROSPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(uuv_simulator_ros::ThrusterROSPlugin,
                    "uuv_simulator_ros::ThrusterROSPlugin")
