// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#include <uuv_sensor_ros_plugins/DVLROSPlugin.hh>
#include <gz/sim/components/WorldPose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/Util.hh>

namespace gz { namespace sim {

DVLROSPlugin::DVLROSPlugin() : ROSBaseModelPlugin()
{
  this->beamTransformsInitialized = false;
}

DVLROSPlugin::~DVLROSPlugin() {}

void DVLROSPlugin::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& _eventMgr)
{
  ROSBaseModelPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  auto sdfPtr = std::const_pointer_cast<sdf::Element>(_sdf);

  // Read beam link names and topics
  for (int i = 0; i < 4; i++)
  {
    std::string beamLinkName, beamTopic;
    GetSDFParam<std::string>(sdfPtr, "beam_link_name_" + std::to_string(i), beamLinkName, "");
    GZ_ASSERT(!beamLinkName.empty(), ("Beam " + std::to_string(i) + " link name empty").c_str());
    this->beamsLinkNames.push_back(beamLinkName);

    GetSDFParam<std::string>(sdfPtr, "beam_topic_" + std::to_string(i), beamTopic, "");
    GZ_ASSERT(!beamTopic.empty(), ("Beam " + std::to_string(i) + " topic empty").c_str());
    this->beamTopics.push_back(beamTopic);

    this->dvlBeamMsgs.push_back(uuv_sensor_ros_plugins_msgs::msg::DVLBeam());
  }

  // tf2 buffer + listener
  this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->rosNode->get_clock());
  this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer);

  // message_filters subscribers (need rclcpp node)
  this->beamSub0 = std::make_shared<RangeSub>(this->rosNode, this->beamTopics[0]);
  this->beamSub1 = std::make_shared<RangeSub>(this->rosNode, this->beamTopics[1]);
  this->beamSub2 = std::make_shared<RangeSub>(this->rosNode, this->beamTopics[2]);
  this->beamSub3 = std::make_shared<RangeSub>(this->rosNode, this->beamTopics[3]);

  this->syncBeamMessages = std::make_shared<Synchronizer>(
    SyncPolicy(10),
    *this->beamSub0, *this->beamSub1,
    *this->beamSub2, *this->beamSub3);

  this->syncBeamMessages->registerCallback(
    std::bind(&DVLROSPlugin::OnBeamCallback, this,
      std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3, std::placeholders::_4));

  // Publishers
  this->rosSensorOutputPub =
    this->rosNode->create_publisher<uuv_sensor_ros_plugins_msgs::msg::DVL>(
      this->sensorOutputTopic, 1);

  this->twistPub =
    this->rosNode->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      this->sensorOutputTopic + "_twist", 1);

  // Frame IDs
  auto linkName = _ecm.ComponentData<components::Name>(this->linkEntity).value_or("dvl");
  if (this->enableLocalNEDFrame)
  {
    this->dvlROSMsg.header.frame_id = linkName + "_ned";
    this->twistROSMsg.header.frame_id = linkName + "_ned";
  }
  else
  {
    this->dvlROSMsg.header.frame_id = linkName;
    this->twistROSMsg.header.frame_id = linkName;
  }

  double variance = this->noiseSigma * this->noiseSigma;
  this->dvlROSMsg.velocity_covariance.fill(0.0);
  this->dvlROSMsg.velocity_covariance[0] = variance;
  this->dvlROSMsg.velocity_covariance[4] = variance;
  this->dvlROSMsg.velocity_covariance[8] = variance;

  this->twistROSMsg.twist.covariance.fill(0.0);
  this->twistROSMsg.twist.covariance[0]  = variance;
  this->twistROSMsg.twist.covariance[7]  = variance;
  this->twistROSMsg.twist.covariance[14] = variance;
  this->twistROSMsg.twist.covariance[21] = -1;
  this->twistROSMsg.twist.covariance[28] = -1;
  this->twistROSMsg.twist.covariance[35] = -1;
}

bool DVLROSPlugin::OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
  this->PublishState();
  if (!this->EnableMeasurement(_info))
    return false;

  if (this->enableLocalNEDFrame)
    this->SendLocalNEDTransform();

  if (!this->UpdateBeamTransforms())
    return false;

  // Body-relative linear velocity
  this->link.EnableVelocityChecks(_ecm, true);
  auto vel = this->link.RelativeLinearVelocity(_ecm);
  if (!vel.has_value())
    return false;

  math::Vector3d bodyVel = vel.value();
  bodyVel.X() += this->GetGaussianNoise(this->noiseAmp);
  bodyVel.Y() += this->GetGaussianNoise(this->noiseAmp);
  bodyVel.Z() += this->GetGaussianNoise(this->noiseAmp);

  if (this->enableLocalNEDFrame)
    bodyVel = this->localNEDFrame.Rot().RotateVector(bodyVel);

  auto stamp = gz::sim::convert<rclcpp::Time>(_info.simTime);

  this->dvlROSMsg.header.stamp = stamp;
  this->dvlROSMsg.altitude = this->altitude;
  this->dvlROSMsg.beams = this->dvlBeamMsgs;
  this->dvlROSMsg.velocity.x = bodyVel.X();
  this->dvlROSMsg.velocity.y = bodyVel.Y();
  this->dvlROSMsg.velocity.z = bodyVel.Z();
  this->rosSensorOutputPub->publish(this->dvlROSMsg);

  this->twistROSMsg.header.stamp = stamp;
  this->twistROSMsg.twist.twist.linear.x = bodyVel.X();
  this->twistROSMsg.twist.twist.linear.y = bodyVel.Y();
  this->twistROSMsg.twist.twist.linear.z = bodyVel.Z();
  this->twistPub->publish(this->twistROSMsg);

  this->lastMeasurementTime = _info.simTime;
  return true;
}

void DVLROSPlugin::OnBeamCallback(
  const sensor_msgs::msg::Range::ConstSharedPtr& _r0,
  const sensor_msgs::msg::Range::ConstSharedPtr& _r1,
  const sensor_msgs::msg::Range::ConstSharedPtr& _r2,
  const sensor_msgs::msg::Range::ConstSharedPtr& _r3)
{
  if ((_r0->range == _r0->min_range && _r1->range == _r1->min_range &&
       _r2->range == _r2->min_range && _r3->range == _r3->min_range) ||
      (_r0->range == _r0->max_range && _r1->range == _r1->max_range &&
       _r2->range == _r2->max_range && _r3->range == _r3->max_range))
  {
    this->altitude = ALTITUDE_OUT_OF_RANGE;
    return;
  }

  this->altitude = 0.25 * (_r0->range + _r1->range + _r2->range + _r3->range);
  this->dvlBeamMsgs[0].range = _r0->range;
  this->dvlBeamMsgs[1].range = _r1->range;
  this->dvlBeamMsgs[2].range = _r2->range;
  this->dvlBeamMsgs[3].range = _r3->range;
}

bool DVLROSPlugin::UpdateBeamTransforms()
{
  if (this->beamPoses.size() == 4)
    return true;

  for (size_t i = 0; i < this->beamsLinkNames.size(); i++)
  {
    const std::string& sourceFrame = this->beamsLinkNames[i];
    const std::string targetFrame = this->enableLocalNEDFrame
      ? this->tfLocalNEDFrame.child_frame_id
      : this->tfLocalNEDFrame.header.frame_id;

    geometry_msgs::msg::TransformStamped ts;
    try
    {
      ts = this->tfBuffer->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
      gzwarn << "DVL beam transform not yet available: " << ex.what() << std::endl;
      return false;
    }

    math::Pose3d pose;
    pose.Pos() = math::Vector3d(
      ts.transform.translation.x,
      ts.transform.translation.y,
      ts.transform.translation.z);
    pose.Rot() = math::Quaterniond(
      ts.transform.rotation.w,
      ts.transform.rotation.x,
      ts.transform.rotation.y,
      ts.transform.rotation.z);

    this->dvlBeamMsgs[i].pose.header.stamp = this->rosNode->now();
    this->dvlBeamMsgs[i].pose.header.frame_id = sourceFrame;
    this->dvlBeamMsgs[i].pose.pose.position.x = ts.transform.translation.x;
    this->dvlBeamMsgs[i].pose.pose.position.y = ts.transform.translation.y;
    this->dvlBeamMsgs[i].pose.pose.position.z = ts.transform.translation.z;
    this->dvlBeamMsgs[i].pose.pose.orientation = ts.transform.rotation;

    this->beamPoses.push_back(pose);
  }
  return true;
}

GZ_ADD_PLUGIN(DVLROSPlugin, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemUpdate)

}} // namespace gz::sim
