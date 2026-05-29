// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#include <uuv_sensor_ros_plugins/PoseGTROSPlugin.hh>
#include <gz/sim/components/WorldPose.hh>
#include <gz/sim/components/WorldLinearVelocity.hh>
#include <gz/sim/components/WorldAngularVelocity.hh>
#include <gz/sim/Util.hh>

namespace gz { namespace sim {

PoseGTROSPlugin::PoseGTROSPlugin() : ROSBaseModelPlugin()
{
  this->offset      = math::Pose3d::Zero;
  this->nedTransform = math::Pose3d::Zero;
  this->nedTransformIsInit = true;
  this->refLinAcc = this->refAngAcc = math::Vector3d::Zero;
}

PoseGTROSPlugin::~PoseGTROSPlugin() {}

void PoseGTROSPlugin::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& _eventMgr)
{
  ROSBaseModelPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  auto sdfPtr = std::const_pointer_cast<sdf::Element>(_sdf);

  math::Vector3d vec;
  GetSDFParam<math::Vector3d>(sdfPtr, "position_offset", vec, math::Vector3d::Zero);
  this->offset.Pos() = vec;
  GetSDFParam<math::Vector3d>(sdfPtr, "orientation_offset", vec, math::Vector3d::Zero);
  this->offset.Rot() = math::Quaterniond(vec);

  GetSDFParam<bool>(sdfPtr, "publish_ned_odom", this->publishNEDOdom, false);

  if (this->publishNEDOdom)
  {
    auto linkName = _ecm.ComponentData<components::Name>(this->linkEntity)
      .value_or("base_link");
    this->nedFrameID = linkName + "_ned";
    this->nedOdomPub =
      this->rosNode->create_publisher<nav_msgs::msg::Odometry>(
        this->sensorOutputTopic + "_ned", 1);
    this->nedTransformIsInit = false;
  }

  this->lastRefLinVel = this->lastRefAngVel = math::Vector3d::Zero;

  this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->rosNode->get_clock());
  this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer);

  this->odomPub =
    this->rosNode->create_publisher<nav_msgs::msg::Odometry>(
      this->sensorOutputTopic, 1);
}

bool PoseGTROSPlugin::OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
  if (!this->EnableMeasurement(_info))
    return false;

  using namespace std::chrono;
  double dt = duration<double>(_info.simTime - this->lastMeasurementTime).count();
  if (dt <= 0.0)
    return false;

  this->link.EnableVelocityChecks(_ecm, true);

  const auto* poseComp = _ecm.Component<components::WorldPose>(this->linkEntity);
  auto linVelOpt = this->link.WorldLinearVelocity(_ecm);
  auto angVelOpt = this->link.WorldAngularVelocity(_ecm);

  if (!poseComp || !linVelOpt || !angVelOpt)
    return false;

  this->UpdateNEDTransform(_ecm);
  this->UpdateReferenceFramePose(_ecm);

  math::Pose3d  linkPose   = poseComp->Data();
  math::Vector3d linkLinVel = linVelOpt.value();
  math::Vector3d linkAngVel = angVelOpt.value();

  math::Vector3d refLinVel = math::Vector3d::Zero;
  math::Vector3d refAngVel = math::Vector3d::Zero;

  if (this->referenceLink != kNullEntity)
  {
    Link refLink(this->referenceLink);
    refLink.EnableVelocityChecks(_ecm, true);
    auto rlv = refLink.WorldLinearVelocity(_ecm);
    auto rav = refLink.WorldAngularVelocity(_ecm);
    if (rlv) refLinVel = rlv.value();
    if (rav) refAngVel = rav.value();
  }

  linkLinVel -= refLinVel;
  linkAngVel -= refAngVel;

  linkLinVel += math::Vector3d(
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp));
  linkAngVel += math::Vector3d(
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp),
    this->GetGaussianNoise(this->noiseAmp));

  auto rosTime = gz::sim::convert<rclcpp::Time>(_info.simTime);
  this->PublishOdomMessage(rosTime, linkPose, linkLinVel, linkAngVel);
  this->PublishNEDOdomMessage(rosTime, linkPose, linkLinVel, linkAngVel);

  this->lastMeasurementTime = _info.simTime;
  return true;
}

void PoseGTROSPlugin::PublishOdomMessage(
  const rclcpp::Time& _time, const math::Pose3d& _poseIn,
  const math::Vector3d& _linVel, const math::Vector3d& _angVel)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp    = _time;

  auto linkName = "base_link"; // set from cached name in practice
  odom.child_frame_id  = linkName;

  math::Pose3d pose = _poseIn + this->offset;
  odom.pose.pose.position.x    = pose.Pos().X();
  odom.pose.pose.position.y    = pose.Pos().Y();
  odom.pose.pose.position.z    = pose.Pos().Z();
  odom.pose.pose.orientation.x = pose.Rot().X();
  odom.pose.pose.orientation.y = pose.Rot().Y();
  odom.pose.pose.orientation.z = pose.Rot().Z();
  odom.pose.pose.orientation.w = pose.Rot().W();
  odom.twist.twist.linear.x  = _linVel.X();
  odom.twist.twist.linear.y  = _linVel.Y();
  odom.twist.twist.linear.z  = _linVel.Z();
  odom.twist.twist.angular.x = _angVel.X();
  odom.twist.twist.angular.y = _angVel.Y();
  odom.twist.twist.angular.z = _angVel.Z();

  double gn2 = this->noiseSigma * this->noiseSigma;
  for (int i : {0,7,14,21,28,35})
  {
    odom.pose.covariance[i]  = gn2;
    odom.twist.covariance[i] = gn2;
  }
  this->odomPub->publish(odom);
}

void PoseGTROSPlugin::PublishNEDOdomMessage(
  const rclcpp::Time& _time, const math::Pose3d& _poseIn,
  const math::Vector3d& _linVel, const math::Vector3d& _angVel)
{
  if (!this->publishNEDOdom || !this->nedTransformIsInit)
    return;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = this->referenceFrameID;
  odom.header.stamp    = _time;
  odom.child_frame_id  = this->nedFrameID;

  math::Pose3d pose = _poseIn;
  pose.Pos() -= this->referenceFrame.Pos();
  pose.Pos()  = this->referenceFrame.Rot().RotateVectorReverse(pose.Pos());
  math::Quaterniond q = this->nedTransform.Rot();
  q = pose.Rot() * q;
  q = this->referenceFrame.Rot() * q;
  pose.Rot() = q;
  pose += this->offset;

  math::Vector3d lv = this->referenceFrame.Rot().RotateVector(_linVel);
  math::Vector3d av = this->referenceFrame.Rot().RotateVector(_angVel);

  odom.pose.pose.position.x    = pose.Pos().X();
  odom.pose.pose.position.y    = pose.Pos().Y();
  odom.pose.pose.position.z    = pose.Pos().Z();
  odom.pose.pose.orientation.x = pose.Rot().X();
  odom.pose.pose.orientation.y = pose.Rot().Y();
  odom.pose.pose.orientation.z = pose.Rot().Z();
  odom.pose.pose.orientation.w = pose.Rot().W();
  odom.twist.twist.linear.x  = lv.X();
  odom.twist.twist.linear.y  = lv.Y();
  odom.twist.twist.linear.z  = lv.Z();
  odom.twist.twist.angular.x = av.X();
  odom.twist.twist.angular.y = av.Y();
  odom.twist.twist.angular.z = av.Z();

  double gn2 = this->noiseSigma * this->noiseSigma;
  for (int i : {0,7,14,21,28,35})
  {
    odom.pose.covariance[i]  = gn2;
    odom.twist.covariance[i] = gn2;
  }
  this->nedOdomPub->publish(odom);
}

void PoseGTROSPlugin::UpdateNEDTransform(EntityComponentManager& /*_ecm*/)
{
  if (!this->publishNEDOdom || this->nedTransformIsInit)
    return;

  geometry_msgs::msg::TransformStamped ts;
  try
  {
    ts = this->tfBuffer->lookupTransform(
      this->nedFrameID,
      this->tfLocalNEDFrame.header.frame_id,  // link frame
      tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    gzmsg << "NED transform not yet available: " << ex.what() << std::endl;
    return;
  }

  this->nedTransform.Pos() = math::Vector3d(
    ts.transform.translation.x,
    ts.transform.translation.y,
    ts.transform.translation.z);
  this->nedTransform.Rot() = math::Quaterniond(
    ts.transform.rotation.w,
    ts.transform.rotation.x,
    ts.transform.rotation.y,
    ts.transform.rotation.z);
  this->nedTransformIsInit = true;
}

GZ_ADD_PLUGIN(PoseGTROSPlugin, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemUpdate)

}} // namespace gz::sim
