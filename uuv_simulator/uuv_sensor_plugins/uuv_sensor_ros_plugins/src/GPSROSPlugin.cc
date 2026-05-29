// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#include <uuv_sensor_ros_plugins/GPSROSPlugin.hh>
#include <gz/sim/components/NavSat.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/Util.hh>

namespace gz { namespace sim {

GPSROSPlugin::GPSROSPlugin() : ROSBaseSensorPlugin() {}
GPSROSPlugin::~GPSROSPlugin() {}

void GPSROSPlugin::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& _eventMgr)
{
  ROSBaseSensorPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
  auto sdfPtr = std::const_pointer_cast<sdf::Element>(_sdf);

  this->gpsPub =
    this->rosNode->create_publisher<sensor_msgs::msg::NavSatFix>(
      this->sensorOutputTopic, 10);

  this->gpsMessage.header.frame_id = this->robotNamespace + "/gps_link";
  this->gpsMessage.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;

  double hStdDev = 0.0, vStdDev = 0.0;
  GetSDFParam(sdfPtr, "horizontal_pos_std_dev", hStdDev, 0.0);
  GetSDFParam(sdfPtr, "vertical_pos_std_dev",   vStdDev, 0.0);

  this->gpsMessage.position_covariance[0] = hStdDev * hStdDev;
  this->gpsMessage.position_covariance[4] = hStdDev * hStdDev;
  this->gpsMessage.position_covariance[8] = vStdDev * vStdDev;

  this->gpsMessage.status.status  = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  this->gpsMessage.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  // Enable NavSat component so the ECM populates it
  _ecm.SetComponentData<components::NavSat>(
    this->sensorEntity, components::NavSatData());
}

bool GPSROSPlugin::OnUpdate(
  const UpdateInfo& _info,
  EntityComponentManager& _ecm)
{
  return OnUpdateGPS(_info, _ecm);
}

bool GPSROSPlugin::OnUpdateGPS(
  const UpdateInfo& _info,
  EntityComponentManager& _ecm)
{
  this->PublishState();

  const auto* navSat =
    _ecm.Component<components::NavSat>(this->sensorEntity);
  if (!navSat)
    return false;

  this->gpsMessage.header.stamp =
    gz::sim::convert<rclcpp::Time>(_info.simTime);

  this->gpsMessage.latitude  = navSat->Data().LatitudeDeg();
  this->gpsMessage.longitude = navSat->Data().LongitudeDeg();
  this->gpsMessage.altitude  = navSat->Data().Altitude();

  this->gpsPub->publish(this->gpsMessage);
  this->lastMeasurementTime = _info.simTime;
  return true;
}

GZ_ADD_PLUGIN(GPSROSPlugin, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemUpdate)

}} // namespace gz::sim
