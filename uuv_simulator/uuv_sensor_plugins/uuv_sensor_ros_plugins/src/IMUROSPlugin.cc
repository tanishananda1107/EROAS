// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#include <uuv_sensor_ros_plugins/IMUROSPlugin.hh>
#include <gz/sim/components/WorldPose.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Util.hh>

namespace gz { namespace sim {

IMUROSPlugin::IMUROSPlugin() : ROSBaseModelPlugin() {}
IMUROSPlugin::~IMUROSPlugin() {}

void IMUROSPlugin::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& _eventMgr)
{
  ROSBaseModelPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  auto sdfPtr = std::const_pointer_cast<sdf::Element>(_sdf);

  GetSDFParam<double>(sdfPtr, "gyroscope_noise_density",
    this->imuParameters.gyroscopeNoiseDensity,
    this->imuParameters.gyroscopeNoiseDensity);
  GetSDFParam<double>(sdfPtr, "gyroscope_bias_random_walk",
    this->imuParameters.gyroscopeRandomWalk,
    this->imuParameters.gyroscopeRandomWalk);
  GetSDFParam<double>(sdfPtr, "gyroscope_bias_correlation_time",
    this->imuParameters.gyroscopeBiasCorrelationTime,
    this->imuParameters.gyroscopeBiasCorrelationTime);
  GetSDFParam<double>(sdfPtr, "gyroscope_turn_on_bias_sigma",
    this->imuParameters.gyroscopeTurnOnBiasSigma,
    this->imuParameters.gyroscopeTurnOnBiasSigma);
  GetSDFParam<double>(sdfPtr, "accelerometer_noise_density",
    this->imuParameters.accelerometerNoiseDensity,
    this->imuParameters.accelerometerNoiseDensity);
  GetSDFParam<double>(sdfPtr, "accelerometer_random_walk",
    this->imuParameters.accelerometerRandomWalk,
    this->imuParameters.accelerometerRandomWalk);
  GetSDFParam<double>(sdfPtr, "accelerometer_bias_correlation_time",
    this->imuParameters.accelerometerBiasCorrelationTime,
    this->imuParameters.accelerometerBiasCorrelationTime);
  GetSDFParam<double>(sdfPtr, "accelerometer_turn_on_bias_sigma",
    this->imuParameters.accelerometerTurnOnBiasSigma,
    this->imuParameters.accelerometerTurnOnBiasSigma);
  GetSDFParam<double>(sdfPtr, "orientation_noise",
    this->imuParameters.orientationNoise,
    this->imuParameters.orientationNoise);

  auto linkName = _ecm.ComponentData<components::Name>(this->linkEntity)
    .value_or("imu_link");
  this->imuROSMessage.header.frame_id = linkName;

  // Noise models
  this->AddNoiseModel("gyro_noise_density",
    this->imuParameters.gyroscopeNoiseDensity);
  double gyroVar = this->imuParameters.gyroscopeNoiseDensity *
    this->imuParameters.gyroscopeNoiseDensity;
  this->imuROSMessage.angular_velocity_covariance[0] = gyroVar;
  this->imuROSMessage.angular_velocity_covariance[4] = gyroVar;
  this->imuROSMessage.angular_velocity_covariance[8] = gyroVar;

  this->AddNoiseModel("acc_noise_density",
    this->imuParameters.accelerometerNoiseDensity);
  double accelVar = this->imuParameters.accelerometerNoiseDensity *
    this->imuParameters.accelerometerNoiseDensity;
  this->imuROSMessage.linear_acceleration_covariance[0] = accelVar;
  this->imuROSMessage.linear_acceleration_covariance[4] = accelVar;
  this->imuROSMessage.linear_acceleration_covariance[8] = accelVar;

  this->AddNoiseModel("orientation_noise_density",
    this->imuParameters.orientationNoise);
  double orientVar = this->imuParameters.orientationNoise *
    this->imuParameters.orientationNoise;
  this->imuROSMessage.orientation_covariance[0] = orientVar;
  this->imuROSMessage.orientation_covariance[4] = orientVar;
  this->imuROSMessage.orientation_covariance[8] = orientVar;

  // Turn-on biases
  this->AddNoiseModel("gyro_turn_on_bias",
    this->imuParameters.gyroscopeTurnOnBiasSigma);
  this->AddNoiseModel("acc_turn_on_bias",
    this->imuParameters.accelerometerTurnOnBiasSigma);

  this->gyroscopeTurnOnBias = math::Vector3d(
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("gyro_turn_on_bias", this->noiseAmp));
  this->accelerometerTurnOnBias = math::Vector3d(
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp),
    this->GetGaussianNoise("acc_turn_on_bias", this->noiseAmp));

  this->gyroscopeBias    = math::Vector3d::Zero;
  this->accelerometerBias = math::Vector3d::Zero;

  // Gravity from world
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (worldEntity != kNullEntity)
  {
    const auto* gravity = _ecm.Component<components::Gravity>(worldEntity);
    if (gravity)
      this->gravityWorld = gravity->Data();
  }

  this->imuPub = this->rosNode->create_publisher<sensor_msgs::msg::Imu>(
    this->sensorOutputTopic, 1);
}

bool IMUROSPlugin::OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
  this->PublishState();
  if (!this->EnableMeasurement(_info))
    return false;

  if (this->enableLocalNEDFrame)
    this->SendLocalNEDTransform();

  using namespace std::chrono;
  double dt = duration<double>(_info.simTime - this->lastMeasurementTime).count();
  if (dt <= 0.0)
    return false;

  this->link.EnableVelocityChecks(_ecm, true);
  this->link.EnableAccelerationChecks(_ecm, true);

  auto angVelOpt  = this->link.RelativeAngularVelocity(_ecm);
  auto linAccOpt  = this->link.RelativeLinearAcceleration(_ecm);
  auto worldPoseOpt = _ecm.Component<components::WorldPose>(this->linkEntity);

  if (!angVelOpt || !linAccOpt || !worldPoseOpt)
    return false;

  math::Vector3d bodyAngVel  = angVelOpt.value();
  math::Vector3d bodyLinAcc  = linAccOpt.value();
  math::Pose3d worldLinkPose = worldPoseOpt->Data();

  this->UpdateReferenceFramePose(_ecm);

  worldLinkPose.Pos() -= this->referenceFrame.Pos();
  worldLinkPose.Pos()  = this->referenceFrame.Rot().RotateVectorReverse(worldLinkPose.Pos());
  worldLinkPose.Rot() *= this->referenceFrame.Rot().Inverse();

  math::Vector3d gravityBody =
    worldLinkPose.Rot().RotateVectorReverse(this->gravityWorld);

  if (this->enableLocalNEDFrame)
  {
    bodyAngVel  = this->localNEDFrame.Rot().RotateVector(bodyAngVel);
    bodyLinAcc  = this->localNEDFrame.Rot().RotateVector(bodyLinAcc);
    gravityBody = this->localNEDFrame.Rot().RotateVector(gravityBody);
  }

  this->measLinearAcc  = bodyLinAcc - gravityBody;
  this->measAngularVel = bodyAngVel;
  this->measOrientation = worldLinkPose.Rot();

  this->AddNoise(this->measLinearAcc, this->measAngularVel,
    this->measOrientation, dt);

  auto stamp = gz::sim::convert<rclcpp::Time>(_info.simTime);
  this->imuROSMessage.header.stamp = stamp;
  this->imuROSMessage.orientation.x = this->measOrientation.X();
  this->imuROSMessage.orientation.y = this->measOrientation.Y();
  this->imuROSMessage.orientation.z = this->measOrientation.Z();
  this->imuROSMessage.orientation.w = this->measOrientation.W();
  this->imuROSMessage.linear_acceleration.x = this->measLinearAcc.X();
  this->imuROSMessage.linear_acceleration.y = this->measLinearAcc.Y();
  this->imuROSMessage.linear_acceleration.z = this->measLinearAcc.Z();
  this->imuROSMessage.angular_velocity.x = this->measAngularVel.X();
  this->imuROSMessage.angular_velocity.y = this->measAngularVel.Y();
  this->imuROSMessage.angular_velocity.z = this->measAngularVel.Z();

  this->imuPub->publish(this->imuROSMessage);
  this->lastMeasurementTime = _info.simTime;
  return true;
}

void IMUROSPlugin::AddNoise(
  math::Vector3d& _linAcc, math::Vector3d& _angVel,
  math::Quaterniond& _orientation, double _dt)
{
  GZ_ASSERT(_dt > 0.0, "Invalid time step");

  // Gyroscope
  double tauG   = this->imuParameters.gyroscopeBiasCorrelationTime;
  double sigmaGD = 1.0 / std::sqrt(_dt) * this->imuParameters.gyroscopeNoiseDensity;
  double sigmaBA = this->imuParameters.gyroscopeRandomWalk;
  double sigmaBGD = std::sqrt(-sigmaBA * sigmaBA * tauG / 2.0 *
    (std::exp(-2.0 * _dt / tauG) - 1.0));
  double phiGD = std::exp(-_dt / tauG);

  this->AddNoiseModel("bgd", sigmaBGD);
  this->AddNoiseModel("gd",  sigmaGD);

  this->gyroscopeBias = phiGD * this->gyroscopeBias + math::Vector3d(
    this->GetGaussianNoise("bgd", this->noiseAmp),
    this->GetGaussianNoise("bgd", this->noiseAmp),
    this->GetGaussianNoise("bgd", this->noiseAmp));
  _angVel = _angVel + this->gyroscopeBias + this->gyroscopeTurnOnBias +
    math::Vector3d(
      this->GetGaussianNoise("gd", this->noiseAmp),
      this->GetGaussianNoise("gd", this->noiseAmp),
      this->GetGaussianNoise("gd", this->noiseAmp));

  // Accelerometer
  double tauA    = this->imuParameters.accelerometerBiasCorrelationTime;
  double sigmaAD = 1.0 / std::sqrt(_dt) * this->imuParameters.accelerometerNoiseDensity;
  double sigmaBAw = this->imuParameters.accelerometerRandomWalk;
  double sigmaBAD = std::sqrt(-sigmaBAw * sigmaBAw * tauA / 2.0 *
    (std::exp(-2.0 * _dt / tauA) - 1.0));
  double phiAD = std::exp(-_dt / tauA);

  this->AddNoiseModel("bad", sigmaBAD);
  this->AddNoiseModel("ad",  sigmaAD);

  this->accelerometerBias = phiAD * this->accelerometerBias + math::Vector3d(
    this->GetGaussianNoise("bad", this->noiseAmp),
    this->GetGaussianNoise("bad", this->noiseAmp),
    this->GetGaussianNoise("bad", this->noiseAmp));
  _linAcc = _linAcc + this->accelerometerBias + this->accelerometerTurnOnBias +
    math::Vector3d(
      this->GetGaussianNoise("ad", this->noiseAmp),
      this->GetGaussianNoise("ad", this->noiseAmp),
      this->GetGaussianNoise("ad", this->noiseAmp));

  // Orientation error quaternion
  double scale = 0.5 * this->imuParameters.orientationNoise;
  math::Quaterniond error(1.0,
    this->GetGaussianNoise("orientation_noise_density", scale),
    this->GetGaussianNoise("orientation_noise_density", scale),
    this->GetGaussianNoise("orientation_noise_density", scale));
  error.Normalize();
  _orientation = _orientation * error;
}

GZ_ADD_PLUGIN(IMUROSPlugin, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemUpdate)

}} // namespace gz::sim
