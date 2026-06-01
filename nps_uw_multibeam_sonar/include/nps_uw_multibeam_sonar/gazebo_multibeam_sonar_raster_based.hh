#ifndef GAZEBO_MULTIBEAM_SONAR_RASTER_BASED_HPP
#define GAZEBO_MULTIBEAM_SONAR_RASTER_BASED_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

#include <opencv2/core.hpp>

namespace nps_sonar
{

class MultibeamSonarRaster :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:

    MultibeamSonarRaster();

    ~MultibeamSonarRaster() override;

    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

private:

    void ComputeSonarImage();

    void ComputePointCloud();

    cv::Mat ComputeNormalImage(cv::Mat &depth);

private:

    rclcpp::Node::SharedPtr ros_node_;

    rclcpp::Publisher<
        sensor_msgs::msg::Image
    >::SharedPtr sonar_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2
    >::SharedPtr cloud_pub_;

private:

    gz::sim::Entity cameraEntity_;

    int width_;
    int height_;

    double sonarFreq_;
    double bandwidth_;
    double soundSpeed_;

    cv::Mat depthImage_;
    cv::Mat reflectivityImage_;
};

}

#endif
