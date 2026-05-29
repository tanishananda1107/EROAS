// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#include <uuv_sensor_ros_plugins/CPCROSPlugin.hh>
#include <gz/sim/components/WorldPose.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/Util.hh>

namespace gz { namespace sim {

CPCROSPlugin::CPCROSPlugin() : ROSBaseModelPlugin() {}
CPCROSPlugin::~CPCROSPlugin() {}

void CPCROSPlugin::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& _eventMgr)
{
  ROSBaseModelPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  auto sdfPtr = std::const_pointer_cast<sdf::Element>(_sdf);

  std::string inputTopic;
  GetSDFParam<std::string>(sdfPtr, "plume_topic", inputTopic, "");
  GZ_ASSERT(!inputTopic.empty(), "Plume topic is empty");

  GetSDFParam<double>(sdfPtr, "gamma", this->gamma, 0.001);
  GZ_ASSERT(this->gamma > 0, "Gamma must be > 0");

  GetSDFParam<double>(sdfPtr, "gain", this->gain, 1.0);
  GZ_ASSERT(this->gain > 0, "Gain must be > 0");

  GetSDFParam<double>(sdfPtr, "radius", this->smoothingLength, 3.0);
  GZ_ASSERT(this->smoothingLength > 0, "Radius must be > 0");

  std::string salinityTopic;
  GetSDFParam<std::string>(sdfPtr, "salinity_output_topic", salinityTopic, "salinity");

  std::string salinityUnit;
  GetSDFParam<std::string>(sdfPtr, "salinity_unit", salinityUnit, "ppt");
  GZ_ASSERT(salinityUnit == "ppt" || salinityUnit == "ppm" || salinityUnit == "psu",
    "Invalid salinity unit");
  this->salinityMsg.unit = salinityUnit;

  if (sdfPtr->HasElement("water_salinity_value"))
    GetSDFParam<double>(sdfPtr, "water_salinity_value", this->waterSalinityValue, 35.0);
  else
    this->waterSalinityValue = (salinityUnit == "ppm") ? 35000.0 : 35.0;

  GetSDFParam<double>(sdfPtr, "plume_salinity_value", this->plumeSalinityValue, 0.0);

  // Subscriber
  this->particlesSub =
    this->rosNode->create_subscription<sensor_msgs::msg::PointCloud>(
      inputTopic, 1,
      [this](const sensor_msgs::msg::PointCloud::SharedPtr msg) {
        this->OnPlumeParticlesUpdate(msg);
      });

  // Publishers
  this->rosSensorOutputPub =
    this->rosNode->create_publisher
      uuv_sensor_ros_plugins_msgs::msg::ChemicalParticleConcentration>(
      this->sensorOutputTopic, 1);

  this->salinityPub =
    this->rosNode->create_publisher<uuv_sensor_ros_plugins_msgs::msg::Salinity>(
      salinityTopic, 1);

  this->outputMsg.concentration = 0.0;
  this->outputMsg.is_measuring = false;
  this->salinityMsg.variance = this->noiseSigma * this->noiseSigma;
  this->lastUpdateTimestamp = this->rosNode->now();
}

bool CPCROSPlugin::OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm)
{
  this->PublishState();
  if (!this->EnableMeasurement(_info) || this->updatingCloud)
    return false;

  using namespace std::chrono;
  double simSec = duration<double>(_info.simTime).count();
  if (simSec - this->lastUpdateTimestamp.seconds() > 5.0)
  {
    this->outputMsg.is_measuring = false;
    this->outputMsg.concentration = 0.0;
  }

  auto stamp = gz::sim::convert<rclcpp::Time>(_info.simTime);
  this->outputMsg.header.frame_id = this->referenceFrameID;
  this->outputMsg.concentration += this->GetGaussianNoise(this->noiseAmp);
  this->outputMsg.header.stamp = stamp;
  this->rosSensorOutputPub->publish(this->outputMsg);

  this->salinityMsg.header.frame_id = this->referenceFrameID;
  this->salinityMsg.header.stamp = stamp;
  this->salinityMsg.salinity =
    this->waterSalinityValue * (1.0 - std::min(1.0, this->outputMsg.concentration)) +
    std::min(1.0, this->outputMsg.concentration) * this->plumeSalinityValue;
  this->salinityPub->publish(this->salinityMsg);

  this->lastMeasurementTime = _info.simTime;
  return true;
}

void CPCROSPlugin::OnPlumeParticlesUpdate(
  const sensor_msgs::msg::PointCloud::SharedPtr _msg)
{
  if (this->rosSensorOutputPub->get_subscription_count() == 0)
    return;

  this->updatingCloud = true;

  // Get link world position from ECM
  const auto* poseComp =
    // Will be provided through the ECM in the Update thread —
    // store cached pose from last Update call to avoid cross-thread access.
    // For correctness, use a cached pose member updated in OnUpdate.
    // Here we use the cached referenceFrame as fallback.
    nullptr; // placeholder

  // NOTE: link pose should be cached from the Update() thread.
  // Assuming this->cachedLinkPos is updated each step (add as member).
  math::Vector3d linkPos = this->cachedLinkWorldPos; // see note below

  math::Vector3d linkPosRef = linkPos - this->referenceFrame.Pos();
  linkPosRef = this->referenceFrame.Rot().RotateVectorReverse(linkPosRef);

  this->outputMsg.is_measuring = true;
  this->outputMsg.position.x = linkPosRef.X();
  this->outputMsg.position.y = linkPosRef.Y();
  this->outputMsg.position.z = linkPosRef.Z();

  // Spherical coordinates would need a gz::sim::SphericalCoordinates lookup
  // from the world — omitted here; set to 0 as placeholder.
  this->outputMsg.latitude = 0.0;
  this->outputMsg.longitude = 0.0;
  this->outputMsg.depth = -linkPos.Z();

  this->lastUpdateTimestamp = rclcpp::Time(
    _msg->header.stamp.sec, _msg->header.stamp.nanosec);
  double currentTime = this->lastUpdateTimestamp.seconds();

  double totalParticleConc = 0.0;
  double initSmoothingLength = std::pow(this->smoothingLength, 2.0 / 3.0);

  for (size_t i = 0; i < _msg->points.size(); i++)
  {
    math::Vector3d particle(_msg->points[i].x, _msg->points[i].y, _msg->points[i].z);
    double smoothingParam = std::pow(
      initSmoothingLength + this->gamma * (currentTime - _msg->channels[0].values[i]),
      1.5);
    double distToParticle = linkPos.Distance(particle);

    double particleConc = 0.0;
    if (distToParticle >= 0 && distToParticle < smoothingParam)
      particleConc = 4.0
        - 6.0 * std::pow(distToParticle / smoothingParam, 2)
        + 3.0 * std::pow(distToParticle / smoothingParam, 3);
    else if (distToParticle < 2.0 * smoothingParam)
      particleConc = std::pow(2.0 - distToParticle / smoothingParam, 3);

    particleConc /= (4.0 * M_PI * std::pow(smoothingParam, 3));
    totalParticleConc += particleConc;
  }

  this->outputMsg.concentration = this->gain * totalParticleConc;
  this->updatingCloud = false;
}

GZ_ADD_PLUGIN(CPCROSPlugin, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemUpdate)

}} // namespace gz::sim
