// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#include <uuv_sensor_ros_plugins/SubseaPressureROSPlugin.hh>
#include <gz/sim/components/WorldPose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Util.hh>

namespace gz { namespace sim {

SubseaPressureROSPlugin::SubseaPressureROSPlugin() : ROSBaseModelPlugin() {}
SubseaPressureROSPlugin::~SubseaPressureROSPlugin() {}

void SubseaPressureROSPlugin::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& _eventMgr)
{
  ROSBaseModelPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  auto sdfPtr = std::const_pointer_cast<sdf::Element>(_sdf);

  GetSDFParam<double>(sdfPtr, "saturation",       this->saturation,       3000.0);
  GetSDFParam<bool>  (sdfPtr, "estimate_depth_on", this->estimateDepth,   false);
  GetSDFParam<double>(sdfPtr, "standard_pressure", this->standardPressure, 101.325);
  GetSDFParam<double>(sdfPtr, "kPa_per_meter",     this->kPaPerM,          9.80638);

  this->pressurePub =
    this->rosNode->create_publisher<sensor_msgs::msg::FluidPressure>(
      this->sensorOutputTopic, 1);
}

bool SubseaPressureROSPlugin::OnUpdate(
  const UpdateInfo& _info, EntityComponentManager& _ecm)
{
  this->PublishState();
  if (!this->EnableMeasurement(_info))
    return false;

  const auto* poseComp =
    _ecm.Component<components::WorldPose>(this->linkEntity);
  if (!poseComp)
    return false;

  double depth = std::abs(poseComp->Data().Pos().Z());
  double pressure = this->standardPressure + depth * this->kPaPerM;
  pressure += this->GetGaussianNoise(this->noiseAmp);

  auto linkName = _ecm.ComponentData<components::Name>(this->linkEntity)
    .value_or("pressure");

  sensor_msgs::msg::FluidPressure rosMsg;
  rosMsg.header.stamp    = gz::sim::convert<rclcpp::Time>(_info.simTime);
  rosMsg.header.frame_id = linkName;
  rosMsg.fluid_pressure  = pressure;
  rosMsg.variance        = this->noiseSigma * this->noiseSigma;

  this->pressurePub->publish(rosMsg);
  this->lastMeasurementTime = _info.simTime;
  return true;
}

GZ_ADD_PLUGIN(SubseaPressureROSPlugin, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemUpdate)

}} // namespace gz::sim
