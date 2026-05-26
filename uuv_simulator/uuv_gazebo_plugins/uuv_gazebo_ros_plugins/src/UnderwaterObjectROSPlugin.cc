// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

#include <uuv_gazebo_ros_plugins/UnderwaterObjectROSPlugin.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/plugin/Register.hh>

namespace uuv_simulator_ros
{

/////////////////////////////////////////////////
UnderwaterObjectROSPlugin::UnderwaterObjectROSPlugin() = default;
UnderwaterObjectROSPlugin::~UnderwaterObjectROSPlugin() = default;

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Load(
  gz::sim::EntityComponentManager &_ecm,
  const std::shared_ptr<const sdf::Element> &_sdf)
{
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  rosNode = std::make_shared<rclcpp::Node>("underwater_object_ros_plugin");
  tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(rosNode);

  try {
    UnderwaterObjectPlugin::Load(_ecm, _sdf);
  } catch (const std::exception &e) {
    gzerr << "Error loading UnderwaterObjectPlugin: " << e.what() << "\n";
    return;
  }

  std::string modelName = _ecm.ComponentData<gz::sim::components::Name>(
    this->modelEntity).value_or("uuv");

  subLocalCurVel = rosNode->create_subscription<geometry_msgs::msg::Vector3>(
    modelName + "/current_velocity", 10,
    [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
      UpdateLocalCurrentVelocity(msg);
    });

  auto addSrv = [&](auto key, auto topic, auto handler) {
    services[key] = rosNode->create_service
      std::remove_pointer_t<decltype(handler)>>(topic, handler);
  };

  // --- Services ---
  services["set_use_global_current_velocity"] = rosNode->create_service
    uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel>(
      modelName + "/set_use_global_current_velocity",
      [this](auto rq, auto rs){ SetUseGlobalCurrentVel(rq, rs); });

#define ADD_FLOAT_SRV(key, topic, fn) \
  services[key] = rosNode->create_service< \
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat>( \
      modelName + "/" + topic, \
      [this](auto rq, auto rs){ fn(rq, rs); });

#define ADD_FLOAT_GET_SRV(key, topic, fn) \
  services[key] = rosNode->create_service< \
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat>( \
      modelName + "/" + topic, \
      [this](auto rq, auto rs){ fn(rq, rs); });

  ADD_FLOAT_SRV("set_added_mass_scaling",   "set_added_mass_scaling",   SetScalingAddedMass)
  ADD_FLOAT_GET_SRV("get_added_mass_scaling","get_added_mass_scaling",   GetScalingAddedMass)
  ADD_FLOAT_SRV("set_damping_scaling",      "set_damping_scaling",      SetScalingDamping)
  ADD_FLOAT_GET_SRV("get_damping_scaling",  "get_damping_scaling",      GetScalingDamping)
  ADD_FLOAT_SRV("set_volume_scaling",       "set_volume_scaling",       SetScalingVolume)
  ADD_FLOAT_GET_SRV("get_volume_scaling",   "get_volume_scaling",       GetScalingVolume)
  ADD_FLOAT_SRV("set_fluid_density",        "set_fluid_density",        SetFluidDensity)
  ADD_FLOAT_GET_SRV("get_fluid_density",    "get_fluid_density",        GetFluidDensity)
  ADD_FLOAT_SRV("set_volume_offset",        "set_volume_offset",        SetOffsetVolume)
  ADD_FLOAT_GET_SRV("get_volume_offset",    "get_volume_offset",        GetOffsetVolume)
  ADD_FLOAT_SRV("set_added_mass_offset",    "set_added_mass_offset",    SetOffsetAddedMass)
  ADD_FLOAT_GET_SRV("get_added_mass_offset","get_added_mass_offset",    GetOffsetAddedMass)
  ADD_FLOAT_SRV("set_linear_damping_offset","set_linear_damping_offset",SetOffsetLinearDamping)
  ADD_FLOAT_GET_SRV("get_linear_damping_offset","get_linear_damping_offset",GetOffsetLinearDamping)
  ADD_FLOAT_SRV("set_linear_forward_speed_damping_offset",
                "set_linear_forward_speed_damping_offset",
                SetOffsetLinearForwardSpeedDamping)
  ADD_FLOAT_GET_SRV("get_linear_forward_speed_damping_offset",
                    "get_linear_forward_speed_damping_offset",
                    GetOffsetLinearForwardSpeedDamping)
  ADD_FLOAT_SRV("set_nonlinear_damping_offset","set_nonlinear_damping_offset",
                SetOffsetNonLinearDamping)
  ADD_FLOAT_GET_SRV("get_nonlinear_damping_offset","get_nonlinear_damping_offset",
                    GetOffsetNonLinearDamping)

  services["get_model_properties"] = rosNode->create_service
    uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties>(
      modelName + "/get_model_properties",
      [this](auto rq, auto rs){ GetModelProperties(rq, rs); });

  rosHydroPub["current_velocity_marker"] =
    rosNode->create_publisher<visualization_msgs::msg::Marker>(
      modelName + "/current_velocity_marker", 0);

  rosHydroPub["using_global_current_velocity"] =
    rosNode->create_publisher<std_msgs::msg::Bool>(
      modelName + "/using_global_current_velocity", 0);

  rosHydroPub["is_submerged"] =
    rosNode->create_publisher<std_msgs::msg::Bool>(
      modelName + "/is_submerged", 0);

  // NED transform
  nedTransform.header.frame_id = modelName + "/base_link";
  nedTransform.child_frame_id  = modelName + "/base_link_ned";
  nedTransform.transform.translation.x = 0;
  nedTransform.transform.translation.y = 0;
  nedTransform.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0, 0);
  nedTransform.transform.rotation.x = quat.x();
  nedTransform.transform.rotation.y = quat.y();
  nedTransform.transform.rotation.z = quat.z();
  nedTransform.transform.rotation.w = quat.w();
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Init()  { UnderwaterObjectPlugin::Init(); }
void UnderwaterObjectROSPlugin::Reset() {}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::Update(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  UnderwaterObjectPlugin::Update(_info, _ecm);

  nedTransform.header.stamp = rclcpp::Time(_info.simTime.count());
  tfBroadcaster->sendTransform(nedTransform);

  rclcpp::spin_some(rosNode);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::InitDebug(
  gz::sim::Entity _linkEntity,
  gz::sim::EntityComponentManager &_ecm,
  gazebo::HydrodynamicModelPtr _hydro)
{
  UnderwaterObjectPlugin::InitDebug(_linkEntity, _ecm, _hydro);

  for (auto &it : this->hydroPub)
  {
    rosHydroPub[it.first] =
      rosNode->create_publisher<geometry_msgs::msg::WrenchStamped>(
        it.second, 10);
    gzmsg << "ROS TOPIC: " << it.second << std::endl;
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishRestoringForce(
  gz::sim::Entity _linkEntity,
  gz::sim::EntityComponentManager &_ecm)
{
  UnderwaterObjectPlugin::PublishRestoringForce(_linkEntity, _ecm);

  if (!this->models.count(_linkEntity)) return;
  if (!this->models[_linkEntity]->GetDebugFlag()) return;

  gz::math::Vector3d restoring =
    this->models[_linkEntity]->GetStoredVector(RESTORING_FORCE);

  geometry_msgs::msg::WrenchStamped msg;
  GenWrenchMsg(restoring, gz::math::Vector3d::Zero, msg);

  auto linkName = _ecm.ComponentData<gz::sim::components::Name>(_linkEntity)
    .value_or("link");
  auto pub = std::dynamic_pointer_cast
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(
      rosHydroPub[linkName + "/restoring"]);
  if (pub) pub->publish(msg);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishIsSubmerged()
{
  auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Bool>>(
    rosHydroPub["is_submerged"]);
  if (!pub) return;

  std_msgs::msg::Bool msg;
  msg.data = this->models[this->baseLinkEntity]->IsSubmerged();
  pub->publish(msg);
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishCurrentVelocityMarker()
{
  auto markerPub = std::dynamic_pointer_cast
    rclcpp::Publisher<visualization_msgs::msg::Marker>>(
      rosHydroPub["current_velocity_marker"]);
  auto flagPub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Bool>>(
    rosHydroPub["using_global_current_velocity"]);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = rosNode->now();
  marker.ns = "current_velocity_marker";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;

  if (this->flowVelocity.Length() > 0)
  {
    marker.action = visualization_msgs::msg::Marker::ADD;

    double yaw   = std::atan2(this->flowVelocity.Y(), this->flowVelocity.X());
    double pitch = std::atan2(this->flowVelocity.Z(),
      std::sqrt(std::pow(this->flowVelocity.X(), 2) +
                std::pow(this->flowVelocity.Y(), 2)));

    gz::math::Quaterniond qt(0.0, -pitch, yaw);
    auto pose = gz::sim::Link(this->baseLinkEntity)
      .WorldPose(this->ecm_).value_or(gz::math::Pose3d::Zero);

    marker.pose.position.x = pose.Pos().X();
    marker.pose.position.y = pose.Pos().Y();
    marker.pose.position.z = pose.Pos().Z() + 1.5;
    marker.pose.orientation.x = qt.X();
    marker.pose.orientation.y = qt.Y();
    marker.pose.orientation.z = qt.Z();
    marker.pose.orientation.w = qt.W();
    marker.scale.x = 1; marker.scale.y = 0.1; marker.scale.z = 0.1;
    marker.color.a = 1.0; marker.color.r = 0.0;
    marker.color.g = 0.0; marker.color.b = 1.0;
  }
  else
    marker.action = visualization_msgs::msg::Marker::DELETE;

  if (markerPub) markerPub->publish(marker);

  if (flagPub)
  {
    std_msgs::msg::Bool useGlobalMsg;
    useGlobalMsg.data = this->useGlobalCurrent;
    flagPub->publish(useGlobalMsg);
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::PublishHydrodynamicWrenches(
  gz::sim::Entity _linkEntity,
  gz::sim::EntityComponentManager &_ecm)
{
  UnderwaterObjectPlugin::PublishRestoringForce(_linkEntity, _ecm);

  if (!this->models.count(_linkEntity)) return;
  if (!this->models[_linkEntity]->GetDebugFlag()) return;

  auto linkName = _ecm.ComponentData<gz::sim::components::Name>(_linkEntity)
    .value_or("link");

  geometry_msgs::msg::WrenchStamped msg;
  gz::math::Vector3d force, torque;

  auto publish = [&](const std::string &topic) {
    GenWrenchMsg(force, torque, msg);
    auto pub = std::dynamic_pointer_cast
      rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(
        rosHydroPub[linkName + "/" + topic]);
    if (pub) pub->publish(msg);
  };

  force  = this->models[_linkEntity]->GetStoredVector(UUV_ADDED_MASS_FORCE);
  torque = this->models[_linkEntity]->GetStoredVector(UUV_ADDED_MASS_TORQUE);
  publish("added_mass");

  force  = this->models[_linkEntity]->GetStoredVector(UUV_DAMPING_FORCE);
  torque = this->models[_linkEntity]->GetStoredVector(UUV_DAMPING_TORQUE);
  publish("damping");

  force  = this->models[_linkEntity]->GetStoredVector(UUV_ADDED_CORIOLIS_FORCE);
  torque = this->models[_linkEntity]->GetStoredVector(UUV_ADDED_CORIOLIS_TORQUE);
  publish("added_coriolis");
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::GenWrenchMsg(
  gz::math::Vector3d _force, gz::math::Vector3d _torque,
  geometry_msgs::msg::WrenchStamped &_output)
{
  _output.wrench.force.x  = _force.X();
  _output.wrench.force.y  = _force.Y();
  _output.wrench.force.z  = _force.Z();
  _output.wrench.torque.x = _torque.X();
  _output.wrench.torque.y = _torque.Y();
  _output.wrench.torque.z = _torque.Z();
  _output.header.stamp = rosNode->now();
}

/////////////////////////////////////////////////
void UnderwaterObjectROSPlugin::UpdateLocalCurrentVelocity(
  const geometry_msgs::msg::Vector3::SharedPtr &_msg)
{
  if (!this->useGlobalCurrent)
  {
    this->flowVelocity.X() = _msg->x;
    this->flowVelocity.Y() = _msg->y;
    this->flowVelocity.Z() = _msg->z;
  }
}

// --- Float service implementations (pattern is identical for all) ---

#define IMPL_SET_FLOAT(fn, param_name, label) \
bool UnderwaterObjectROSPlugin::fn( \
  uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req, \
  uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res) \
{ \
  if (_req->data < 0) { _res->success = false; \
    _res->message = "Value cannot be negative"; } \
  else { \
    for (auto &it : models) it.second->SetParam(param_name, _req->data); \
    _res->success = true; _res->message = label; } \
  return true; \
}

#define IMPL_GET_FLOAT(fn, param_name) \
bool UnderwaterObjectROSPlugin::fn( \
  uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr, \
  uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res) \
{ models.begin()->second->GetParam(param_name, _res->data); return true; }

IMPL_SET_FLOAT(SetScalingAddedMass,  "scaling_added_mass",            "Added-mass scaling set")
IMPL_GET_FLOAT(GetScalingAddedMass,  "scaling_added_mass")
IMPL_SET_FLOAT(SetScalingDamping,    "scaling_damping",               "Damping scaling set")
IMPL_GET_FLOAT(GetScalingDamping,    "scaling_damping")
IMPL_SET_FLOAT(SetScalingVolume,     "scaling_volume",                "Volume scaling set")
IMPL_GET_FLOAT(GetScalingVolume,     "scaling_volume")
IMPL_SET_FLOAT(SetFluidDensity,      "fluid_density",                 "Fluid density set")
IMPL_GET_FLOAT(GetFluidDensity,      "fluid_density")
IMPL_SET_FLOAT(SetOffsetVolume,      "offset_volume",                 "Volume offset set")
IMPL_GET_FLOAT(GetOffsetVolume,      "offset_volume")
IMPL_SET_FLOAT(SetOffsetAddedMass,   "offset_added_mass",             "Added-mass offset set")
IMPL_GET_FLOAT(GetOffsetAddedMass,   "offset_added_mass")
IMPL_SET_FLOAT(SetOffsetLinearDamping,"offset_linear_damping",        "Linear damping offset set")
IMPL_GET_FLOAT(GetOffsetLinearDamping,"offset_linear_damping")
IMPL_SET_FLOAT(SetOffsetLinearForwardSpeedDamping,
               "offset_lin_forward_speed_damping","Lin fwd speed damping offset set")
IMPL_GET_FLOAT(GetOffsetLinearForwardSpeedDamping,"offset_lin_forward_speed_damping")
IMPL_SET_FLOAT(SetOffsetNonLinearDamping,"offset_nonlin_damping",     "Nonlinear damping offset set")
IMPL_GET_FLOAT(GetOffsetNonLinearDamping,"offset_nonlin_damping")

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::SetUseGlobalCurrentVel(
  uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Request::SharedPtr _req,
  uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Response::SharedPtr _res)
{
  if (_req->use_global != this->useGlobalCurrent)
  {
    this->useGlobalCurrent = _req->use_global;
    this->flowVelocity = gz::math::Vector3d::Zero;
    gzmsg << (this->useGlobalCurrent ?
      "Now using global current velocity\n" :
      "Using local current velocity\n");
  }
  _res->success = true;
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterObjectROSPlugin::GetModelProperties(
  uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Request::SharedPtr,
  uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Response::SharedPtr _res)
{
  for (auto &it : models)
  {
    gz::sim::Entity linkEnt = it.first;
    gazebo::HydrodynamicModelPtr hydro = it.second;

    _res->link_names.push_back(
      this->ecm_->ComponentData<gz::sim::components::Name>(linkEnt).value_or(""));

    uuv_gazebo_ros_plugins_msgs::msg::UnderwaterObjectModel model;
    double param; std::vector<double> mat;

    hydro->GetParam("volume",         param); model.volume        = param;
    hydro->GetParam("fluid_density",  param); model.fluid_density = param;
    hydro->GetParam("bbox_height",    param); model.bbox_height   = param;
    hydro->GetParam("bbox_length",    param); model.bbox_length   = param;
    hydro->GetParam("bbox_width",     param); model.bbox_width    = param;

    hydro->GetParam("added_mass",                    mat); model.added_mass                    = mat;
    hydro->GetParam("linear_damping",                mat); model.linear_damping                = mat;
    hydro->GetParam("linear_damping_forward_speed",  mat); model.linear_damping_forward_speed  = mat;
    hydro->GetParam("quadratic_damping",             mat); model.quadratic_damping             = mat;

    model.neutrally_buoyant = hydro->IsNeutrallyBuoyant();
    hydro->GetParam("center_of_buoyancy", mat);
    model.cob.x = mat[0]; model.cob.y = mat[1]; model.cob.z = mat[2];

    auto inertialComp =
      this->ecm_->Component<gz::sim::components::Inertial>(linkEnt);
    if (inertialComp)
    {
      const auto &inertial = inertialComp->Data();
      model.inertia.m   = inertial.MassMatrix().Mass();
      model.inertia.ixx = inertial.MassMatrix().Ixx();
      model.inertia.ixy = inertial.MassMatrix().Ixy();
      model.inertia.ixz = inertial.MassMatrix().Ixz();
      model.inertia.iyy = inertial.MassMatrix().Iyy();
      model.inertia.iyz = inertial.MassMatrix().Iyz();
      model.inertia.izz = inertial.MassMatrix().Izz();
      model.inertia.com.x = inertial.Pose().Pos().X();
      model.inertia.com.y = inertial.Pose().Pos().Y();
      model.inertia.com.z = inertial.Pose().Pos().Z();
    }

    _res->models.push_back(model);
  }
  return true;
}

} // namespace uuv_simulator_ros

GZ_ADD_PLUGIN(uuv_simulator_ros::UnderwaterObjectROSPlugin,
              gz::sim::System,
              uuv_simulator_ros::UnderwaterObjectROSPlugin::ISystemConfigure,
              uuv_simulator_ros::UnderwaterObjectROSPlugin::ISystemUpdate)
