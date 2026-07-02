#include <uuv_gazebo_ros_plugins/FinROSPlugin.hh>

#include <cmath>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

namespace uuv_simulator_ros
{

FinROSPlugin::FinROSPlugin()
    : rosPublishPeriod(std::chrono::milliseconds(50)),
      lastRosPublishTime(std::chrono::steady_clock::now())
{
}

FinROSPlugin::~FinROSPlugin() = default;

void FinROSPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
  uuv_gz_plugins::FinPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);

  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  this->rosNode = std::make_shared<rclcpp::Node>(
      "fin_ros_plugin_" + std::to_string(this->finID));

  this->subReference =
      this->rosNode
          ->create_subscription<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>(
              this->commandTopic, 10,
              [this](const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::
                         SharedPtr msg) { this->SetReference(msg); });

  this->pubState =
      this->rosNode
          ->create_publisher<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>(
              this->angleTopic, 10);

  std::string wrenchTopic = this->topicPrefix + "wrench";
  if (_sdf->HasElement("wrench_topic"))
  {
    wrenchTopic = _sdf->Get<std::string>("wrench_topic");
  }

  this->pubFinForce =
      this->rosNode->create_publisher<geometry_msgs::msg::WrenchStamped>(
          wrenchTopic, 10);

  const std::string liftDragSrv = this->topicPrefix + "get_lift_drag_params";
  this->services["get_lift_drag_params"] =
      this->rosNode
          ->create_service<uuv_gazebo_ros_plugins_msgs::srv::GetListParam>(
              liftDragSrv,
              [this](uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Request::
                         SharedPtr req,
                     uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Response::
                         SharedPtr res) { this->GetLiftDragParams(req, res); });

  gzmsg << "Fin #" << this->finID << " ROS wrapper initialized\n"
        << "\t- Input command topic: " << this->commandTopic << "\n"
        << "\t- Output topic: " << this->angleTopic << "\n";
}

void FinROSPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                             gz::sim::EntityComponentManager &_ecm)
{
  uuv_gz_plugins::FinPlugin::PreUpdate(_info, _ecm);

  if (this->rosNode)
  {
    rclcpp::spin_some(this->rosNode);
    this->RosPublishStates();
  }
}

void FinROSPlugin::SetReference(
    const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr &_msg)
{
  if (std::isnan(_msg->data))
  {
    RCLCPP_WARN(this->rosNode->get_logger(),
                "FinROSPlugin: Ignoring NaN command");
    return;
  }

  this->inputCommand = _msg->data;
}

std::chrono::nanoseconds FinROSPlugin::GetRosPublishPeriod() const
{
  return this->rosPublishPeriod;
}

void FinROSPlugin::SetRosPublishRate(double _hz)
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

void FinROSPlugin::RosPublishStates()
{
  const auto now = std::chrono::steady_clock::now();
  if (this->rosPublishPeriod.count() > 0 &&
      now - this->lastRosPublishTime < this->rosPublishPeriod)
  {
    return;
  }

  this->lastRosPublishTime = now;

  uuv_gazebo_ros_plugins_msgs::msg::FloatStamped stateMsg;
  stateMsg.header.stamp = this->rosNode->now();
  stateMsg.header.frame_id = this->linkName;
  stateMsg.data = this->angle;
  this->pubState->publish(stateMsg);

  geometry_msgs::msg::WrenchStamped wrenchMsg;
  wrenchMsg.header.stamp = stateMsg.header.stamp;
  wrenchMsg.header.frame_id = this->linkName;
  wrenchMsg.wrench.force.x = this->finForce.X();
  wrenchMsg.wrench.force.y = this->finForce.Y();
  wrenchMsg.wrench.force.z = this->finForce.Z();
  this->pubFinForce->publish(wrenchMsg);
}

bool FinROSPlugin::GetLiftDragParams(
    uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Request::SharedPtr,
    uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Response::SharedPtr _res)
{
  _res->description = this->liftdrag->GetType();
  for (const auto &item : this->liftdrag->GetListParams())
  {
    _res->tags.push_back(item.first);
    _res->data.push_back(item.second);
  }
  return true;
}

}  // namespace uuv_simulator_ros

GZ_ADD_PLUGIN(uuv_simulator_ros::FinROSPlugin,
              gz::sim::System,
              uuv_simulator_ros::FinROSPlugin::ISystemConfigure,
              uuv_simulator_ros::FinROSPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(uuv_simulator_ros::FinROSPlugin,
                    "uuv_simulator_ros::FinROSPlugin")
