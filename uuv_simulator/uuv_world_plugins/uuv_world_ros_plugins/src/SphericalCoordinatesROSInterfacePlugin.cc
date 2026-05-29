// Copyright (c) 2016 The UUV Simulator Authors.
// Converted to ROS2 + Gazebo Harmonic (gz-sim8)

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/World.hh>

#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>

#include <uuv_world_ros_plugins_msgs/srv/get_origin_spherical_coord.hpp>
#include <uuv_world_ros_plugins_msgs/srv/set_origin_spherical_coord.hpp>
#include <uuv_world_ros_plugins_msgs/srv/transform_to_spherical_coord.hpp>
#include <uuv_world_ros_plugins_msgs/srv/transform_from_spherical_coord.hpp>

namespace gazebo
{
class SphericalCoordinatesROSInterfacePlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure
{
public:
  SphericalCoordinatesROSInterfacePlugin()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    this->rosNode =
      std::make_shared<rclcpp::Node>(
        "spherical_coordinates_ros_interface");
  }

  ~SphericalCoordinatesROSInterfacePlugin() override = default;

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/) override
  {
    (void)_entity;
    (void)_sdf;
    (void)_ecm;

    this->getOriginSrv =
      this->rosNode->create_service<
        uuv_world_ros_plugins_msgs::srv::GetOriginSphericalCoord>(
        "/gazebo/get_origin_spherical_coordinates",
        std::bind(
          &SphericalCoordinatesROSInterfacePlugin::
            GetOriginSphericalCoord,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    this->setOriginSrv =
      this->rosNode->create_service<
        uuv_world_ros_plugins_msgs::srv::SetOriginSphericalCoord>(
        "/gazebo/set_origin_spherical_coordinates",
        std::bind(
          &SphericalCoordinatesROSInterfacePlugin::
            SetOriginSphericalCoord,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    this->toSphericalSrv =
      this->rosNode->create_service<
        uuv_world_ros_plugins_msgs::srv::TransformToSphericalCoord>(
        "/gazebo/transform_to_spherical_coordinates",
        std::bind(
          &SphericalCoordinatesROSInterfacePlugin::
            TransformToSphericalCoord,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    this->fromSphericalSrv =
      this->rosNode->create_service<
        uuv_world_ros_plugins_msgs::srv::TransformFromSphericalCoord>(
        "/gazebo/transform_from_spherical_coordinates",
        std::bind(
          &SphericalCoordinatesROSInterfacePlugin::
            TransformFromSphericalCoord,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    RCLCPP_INFO(
      this->rosNode->get_logger(),
      "SphericalCoordinatesROSInterfacePlugin loaded");
  }

private:
  std::shared_ptr<rclcpp::Node> rosNode;

  gz::math::SphericalCoordinates sphericalCoords;

  rclcpp::Service<
    uuv_world_ros_plugins_msgs::srv::GetOriginSphericalCoord>::SharedPtr
      getOriginSrv;

  rclcpp::Service<
    uuv_world_ros_plugins_msgs::srv::SetOriginSphericalCoord>::SharedPtr
      setOriginSrv;

  rclcpp::Service<
    uuv_world_ros_plugins_msgs::srv::TransformToSphericalCoord>::SharedPtr
      toSphericalSrv;

  rclcpp::Service<
    uuv_world_ros_plugins_msgs::srv::TransformFromSphericalCoord>::SharedPtr
      fromSphericalSrv;

  void TransformToSphericalCoord(
      const std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          TransformToSphericalCoord::Request> req,
      std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          TransformToSphericalCoord::Response> res)
  {
    gz::math::Vector3d local(
      req->input.x,
      req->input.y,
      req->input.z);

    auto spherical =
      this->sphericalCoords.SphericalFromLocalPosition(local);

    res->latitude_deg = spherical.X();
    res->longitude_deg = spherical.Y();
    res->altitude = spherical.Z();
  }

  void TransformFromSphericalCoord(
      const std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          TransformFromSphericalCoord::Request> req,
      std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          TransformFromSphericalCoord::Response> res)
  {
    gz::math::Vector3d spherical(
      req->latitude_deg,
      req->longitude_deg,
      req->altitude);

    auto local =
      this->sphericalCoords.LocalFromSphericalPosition(spherical);

    res->output.x = local.X();
    res->output.y = local.Y();
    res->output.z = local.Z();
  }

  void GetOriginSphericalCoord(
      const std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          GetOriginSphericalCoord::Request> /*req*/,
      std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          GetOriginSphericalCoord::Response> res)
  {
    res->latitude_deg =
      this->sphericalCoords.LatitudeReference().Degree();

    res->longitude_deg =
      this->sphericalCoords.LongitudeReference().Degree();

    res->altitude =
      this->sphericalCoords.ElevationReference();
  }

  void SetOriginSphericalCoord(
      const std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          SetOriginSphericalCoord::Request> req,
      std::shared_ptr<
        uuv_world_ros_plugins_msgs::srv::
          SetOriginSphericalCoord::Response> res)
  {
    gz::math::Angle lat;
    lat.Degree(req->latitude_deg);

    gz::math::Angle lon;
    lon.Degree(req->longitude_deg);

    this->sphericalCoords.SetLatitudeReference(lat);
    this->sphericalCoords.SetLongitudeReference(lon);
    this->sphericalCoords.SetElevationReference(req->altitude);

    res->success = true;
  }
};

GZ_ADD_PLUGIN(
  SphericalCoordinatesROSInterfacePlugin,
  gz::sim::System,
  SphericalCoordinatesROSInterfacePlugin::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(
  SphericalCoordinatesROSInterfacePlugin,
  "gazebo::SphericalCoordinatesROSInterfacePlugin")

} // namespace gazebo
