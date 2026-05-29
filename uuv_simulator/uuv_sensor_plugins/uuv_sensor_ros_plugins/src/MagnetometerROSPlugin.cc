// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#include <uuv_sensor_ros_plugins/MagnetometerROSPlugin.hh>
#include <gz/sim/components/WorldPose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Util.hh>

namespace gz { namespace sim {

MagnetometerROSPlugin::MagnetometerROSPlugin() : ROSBaseModelPlugin() {}
MagnetometerROSPlugin::~MagnetometerROSPlugin() {}

void MagnetometerROSPlugin::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& _eventMgr)
{
  ROSBaseModelPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  auto sdfPtr = std::const_pointer_cast<sdf::Element>(_sdf);

  GetSDFParam<double>(sdfPtr, "intensity",          this->parameters.intensity,   1.0);
  GetSDFParam<double>(sdfPtr, "reference_heading",  this->parameters.heading,     M_PI);
  GetSDFParam<double>(sdfPtr, "declination",        this->parameters.declination, 0.0);
  GetSDFParam<double>(sdfPtr, "inclination",        this->parameters.inclination, 60.*M_PI/180.);
  GetSDFParam<double>(sdfPtr, "noise_xy",           this->parameters.noiseXY,     1.0);
  GetSDFParam<double>(sdfPtr, "noise_z",            this->parameters.noiseZ,      1.4);
  GetSDFParam<double>(sdfPtr, "turn_on_bias",       this->parameters.turnOnBias,  2.0);

  double heading = this->parameters.heading - this->parameters.declination;
  this->magneticFieldWorld.X() =
    this->parameters.intensity * std::cos(this->parameters.inclination) * std::cos(heading);
  this->magneticFieldWorld.Y() =
    this->parameters.intensity * std::cos(this->parameters.inclination) * std::sin(heading);
  this->magneticFieldWorld.Z() =
    -this->parameters.intensity * std::sin(this->parameters.inclination);

  this->AddNoiseModel("turn_on_bias", this->parameters.turnOnBias);
  this->turnOnBias = math::Vector3d(
    this->GetGaussianNoise("turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("turn_on_bias", this->noiseAmp));

  auto linkName = _ecm.ComponentData<components::Name>(this->linkEntity).value_or("mag");
  this->rosMsg.header.frame_id = this->enableLocalNEDFrame
    ? linkName + "_ned" : linkName;

  this->AddNoiseModel("noise_xy", this->parameters.noiseXY);
  this->AddNoiseModel("noise_z",  this->parameters.noiseZ);

  this->rosMsg.magnetic_field_covariance[0] =
    this->parameters.noiseXY * this->parameters.noiseXY;
  this->rosMsg.magnetic_field_covariance[4] =
    this->parameters.noiseXY * this->parameters.noiseXY;
  this->rosMsg.magnetic_field_covariance[8] =
    this->parameters.noiseZ * this->parameters.noiseZ;

  this->magPub =
    this->rosNode->create_publisher<sensor_msgs::msg::MagneticField>(
      this->sensorOutputTopic, 1);
}

bool MagnetometerROSPlugin::OnUpdate(
  const UpdateInfo& _info, EntityComponentManager& _ecm)
{
  if (!this->EnableMeasurement(_info))
    return false;

  if (this->enableLocalNEDFrame)
    this->SendLocalNEDTransform();

  const auto* poseComp =
    _ecm.Component<components::WorldPose>(this->linkEntity);
  if (!poseComp)
    return false;

  math::Pose3d pose = poseComp->Data();
  math::Vector3d noise(
    this->GetGaussianNoise("noise_xy", this->noiseAmp),
    this->GetGaussianNoise("noise_xy", this->noiseAmp),
    this->GetGaussianNoise("noise_z",  this->noiseAmp));

  this->measMagneticField =
    pose.Rot().RotateVectorReverse(this->magneticFieldWorld)
    + this->turnOnBias + noise;

  if (this->enableLocalNEDFrame)
    this->measMagneticField =
      this->localNEDFrame.Rot().RotateVector(this->measMagneticField);

  this->rosMsg.header.stamp = gz::sim::convert<rclcpp::Time>(_info.simTime);
  this->rosMsg.magnetic_field.x = this->measMagneticField.X();
  this->rosMsg.magnetic_field.y = this->measMagneticField.Y();
  this->rosMsg.magnetic_field.z = this->measMagneticField.Z();
  this->magPub->publish(this->rosMsg);

  this->lastMeasurementTime = _info.simTime;
  return true;
}

GZ_ADD_PLUGIN(MagnetometerROSPlugin, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemUpdate)

}} // namespace gz::sim
