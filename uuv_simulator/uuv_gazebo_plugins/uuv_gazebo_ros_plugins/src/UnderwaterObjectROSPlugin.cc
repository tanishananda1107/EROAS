#include <uuv_gazebo_ros_plugins/UnderwaterObjectROSPlugin.hh>

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Inertial.hh>

namespace uuv_simulator_ros
{

UnderwaterObjectROSPlugin::UnderwaterObjectROSPlugin() = default;
UnderwaterObjectROSPlugin::~UnderwaterObjectROSPlugin() = default;

void UnderwaterObjectROSPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
  uuv_gz_plugins::UnderwaterObjectPlugin::Configure(
      _entity, _sdf, _ecm, _eventMgr);

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  const auto modelName = this->model.Name(_ecm);
  this->rosNode = std::make_shared<rclcpp::Node>(
      "underwater_object_ros_plugin", modelName);

  this->subLocalCurVel =
    this->rosNode->create_subscription<geometry_msgs::msg::Vector3>(
      "current_velocity", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Vector3::SharedPtr _msg)
      {
        this->UpdateLocalCurrentVelocity(_msg);
      });

  using SetUseGlobal =
      uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel;
  this->services["set_use_global_current_velocity"] =
    this->rosNode->create_service<SetUseGlobal>(
      "set_use_global_current_velocity",
      [this](SetUseGlobal::Request::SharedPtr _req,
             SetUseGlobal::Response::SharedPtr _res)
      {
        this->SetUseGlobalCurrentVel(_req, _res);
      });

  using GetModelProperties =
      uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties;
  this->services["get_model_properties"] =
    this->rosNode->create_service<GetModelProperties>(
      "get_model_properties",
      [this](GetModelProperties::Request::SharedPtr _req,
             GetModelProperties::Response::SharedPtr _res)
      {
        this->GetModelProperties(_req, _res);
      });

  using SetFloat = uuv_gazebo_ros_plugins_msgs::srv::SetFloat;
  this->services["set_fluid_density"] =
    this->rosNode->create_service<SetFloat>(
      "set_fluid_density",
      [this](SetFloat::Request::SharedPtr _req,
             SetFloat::Response::SharedPtr _res)
      {
        this->SetFluidDensity(_req, _res);
      });

  using GetFloat = uuv_gazebo_ros_plugins_msgs::srv::GetFloat;
  this->services["get_fluid_density"] =
    this->rosNode->create_service<GetFloat>(
      "get_fluid_density",
      [this](GetFloat::Request::SharedPtr _req,
             GetFloat::Response::SharedPtr _res)
      {
        this->GetFluidDensity(_req, _res);
      });
}

void UnderwaterObjectROSPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  uuv_gz_plugins::UnderwaterObjectPlugin::PreUpdate(_info, _ecm);
  if (this->rosNode)
    rclcpp::spin_some(this->rosNode);
}

void UnderwaterObjectROSPlugin::UpdateLocalCurrentVelocity(
    const geometry_msgs::msg::Vector3::SharedPtr &_msg)
{
  if (this->UseGlobalCurrent())
    return;

  this->SetFlowVelocity(gz::math::Vector3d(_msg->x, _msg->y, _msg->z));
}

void UnderwaterObjectROSPlugin::SetUseGlobalCurrentVel(
    uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Request::
        SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Response::
        SharedPtr _res)
{
  this->SetUseGlobalCurrent(_req->use_global);
  if (_req->use_global)
    this->SetFlowVelocity(gz::math::Vector3d::Zero);
  _res->success = true;
}

void UnderwaterObjectROSPlugin::GetModelProperties(
    uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Request::SharedPtr,
    uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Response::
        SharedPtr _res)
{
  for (const auto &model : this->Models())
  {
    _res->link_names.push_back(model.name);

    uuv_gazebo_ros_plugins_msgs::msg::UnderwaterObjectModel msg;
    msg.volume = model.volume;
    msg.fluid_density = model.fluidDensity;
    msg.cob.x = model.centerOfBuoyancy.X();
    msg.cob.y = model.centerOfBuoyancy.Y();
    msg.cob.z = model.centerOfBuoyancy.Z();
    msg.neutrally_buoyant = model.neutrallyBuoyant;
    msg.linear_damping = {model.linearDamping};
    msg.quadratic_damping = {model.angularDamping};
    _res->models.push_back(msg);
  }
}

void UnderwaterObjectROSPlugin::SetFluidDensity(
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res)
{
  if (_req->data <= 0.0)
  {
    _res->success = false;
    _res->message = "Fluid density must be positive";
    return;
  }

  for (auto &model : this->models)
    model.fluidDensity = _req->data;

  _res->success = true;
  _res->message = "Fluid density updated";
}

void UnderwaterObjectROSPlugin::GetFluidDensity(
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr,
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res)
{
  _res->data = this->models.empty() ? 0.0 : this->models.front().fluidDensity;
}

}  // namespace uuv_simulator_ros

GZ_ADD_PLUGIN(uuv_simulator_ros::UnderwaterObjectROSPlugin,
              gz::sim::System,
              uuv_simulator_ros::UnderwaterObjectROSPlugin::ISystemConfigure,
              uuv_simulator_ros::UnderwaterObjectROSPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(uuv_simulator_ros::UnderwaterObjectROSPlugin,
                    "uuv_simulator_ros::UnderwaterObjectROSPlugin")
