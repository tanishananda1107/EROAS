// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __UUV_UNDERWATER_CAMERA_ROS_PLUGIN_HH__
#define __UUV_UNDERWATER_CAMERA_ROS_PLUGIN_HH__

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sensors/DepthCameraSensor.hh>
#include <gz/rendering/DepthCamera.hh>
#include <uuv_sensor_ros_plugins/Common.hh>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

namespace gz { namespace sim {

class UnderwaterCameraROSPlugin
  : public System, public ISystemConfigure, public ISystemUpdate
{
public:
  UnderwaterCameraROSPlugin();
  virtual ~UnderwaterCameraROSPlugin();
  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;
  void Update(const UpdateInfo& _info, EntityComponentManager& _ecm) override;

  virtual void OnNewDepthFrame(const float* _image,
    unsigned int _width, unsigned int _height, unsigned int _channels, const std::string& _format);
  virtual void OnNewRGBPointCloud(const float* _pcd,
    unsigned int _width, unsigned int _height, unsigned int _channels, const std::string& _format);
  virtual void OnNewImageFrame(const unsigned char* _image,
    unsigned int _width, unsigned int _height, unsigned int _channels, const std::string& _format);

protected:
  virtual void SimulateUnderwater(const cv::Mat& _inputImage,
    const cv::Mat& _inputDepth, cv::Mat& _outputImage);

  const float* lastDepth{nullptr};
  unsigned char* lastImage{nullptr};
  float* depth2rangeLUT{nullptr};
  float attenuation[3]{0.0f, 0.0f, 0.0f};
  unsigned char background[3]{0, 0, 0};

  std::shared_ptr<rclcpp::Node> rosNode;
  image_transport::Publisher imagePub, depthPub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPub;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cameraInfoManager;
  std::shared_ptr<gz::sensors::DepthCameraSensor> depthSensor;
  Entity sensorEntity{kNullEntity};
  gz::common::ConnectionPtr newDepthFrameConn, newRGBPointCloudConn, newImageFrameConn;
};

}}  // namespace gz::sim
#endif
