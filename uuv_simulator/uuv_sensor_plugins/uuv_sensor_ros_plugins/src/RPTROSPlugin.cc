// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#include <uuv_sensor_ros_plugins/RPTROSPlugin.hh>
#include <gz/sim/components/WorldPose.hh>
#include <gz/sim/Util.hh>

namespace gz { namespace sim {

RPTROSPlugin::RPTROSPlugin() : ROSBaseModelPlugin() {}
RPTROSPlugin::~RPTROSPlugin() {}

void RPTROSPlugin::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& _eventMgr)
{
  ROSBaseModelPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);

  double variance = this->noiseSigma * this->noiseSigma;
  this->rosMessage.pos.covariance.fill(0.0);
  this->rosMessage.pos.covariance[0] =
  this->rosMessage.pos.covariance[4] =
  this->rosMessage.pos.covariance[8] = variance;

  this->posPub =
    this->rosNode->create_publisher
      uuv_sensor_ros_plugins_msgs::msg::PositionWithCovarianceStamped>(
        this->sensorOutputTopic, 1);
}

bool RPTROSPlugin::OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
  this->PublishState();
  if (!this->EnableMeasurement(_info))
    return false;

  const auto* poseComp =
    _ecm.Component<components::WorldPose>(this->linkEntity);
  if (!poseComp)
    return false;

  this->position = poseComp->Data().Pos();

  this->UpdateReferenceFramePose(_ecm);
  this->position -= this->referenceFrame.Pos();
  this->position  = this->referenceFrame.Rot().RotateVectorReverse(this->position);

  this->position.X() += this->GetGaussianNoise(this->noiseAmp);
  this->position.Y() += this->GetGaussianNoise(this->noiseAmp);
  this->position.Z() += this->GetGaussianNoise(this->noiseAmp);

  this->rosMessage.header.stamp    = gz::sim::convert<rclcpp::Time>(_info.simTime);
  this->rosMessage.header.frame_id = this->referenceFrameID;
  this->rosMessage.pos.pos.x = this->position.X();
  this->rosMessage.pos.pos.y = this->position.Y();
  this->rosMessage.pos.pos.z = this->position.Z();

  this->posPub->publish(this->rosMessage);
  this->lastMeasurementTime = _info.simTime;
  return true;
}

GZ_ADD_PLUGIN(RPTROSPlugin, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemUpdate)

}} // namespace gz::sim
