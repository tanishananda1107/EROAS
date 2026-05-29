// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef GZ_ROS_IMAGE_SONAR_HH
#define GZ_ROS_IMAGE_SONAR_HH

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sensors/DepthCameraSensor.hh>
#include <gz/rendering/DepthCamera.hh>
#include <gz/common/Event.hh>
#include <opencv2/core.hpp>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace gz { namespace sim {

class GazeboRosImageSonar
  : public System, public ISystemConfigure, public ISystemUpdate
{
public:
  GazeboRosImageSonar();
  ~GazeboRosImageSonar();
  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;
  void Update(const UpdateInfo& _info, EntityComponentManager& _ecm) override;
  virtual void Advertise();

protected:
  virtual void OnNewDepthFrame(const float* _image,
    unsigned int _width, unsigned int _height, unsigned int _depth, const std::string& _format);
  virtual void OnNewRGBPointCloud(const float* _pcd,
    unsigned int _width, unsigned int _height, unsigned int _depth, const std::string& _format);
  virtual void OnNewImageFrame(const unsigned char* _image,
    unsigned int _width, unsigned int _height, unsigned int _depth, const std::string& _format);

private:
  void FillPointCloud(const float* _src);
  void FillDepthImage(const float* _src);
  void ComputeSonarImage(const float* _src);
  cv::Mat ComputeNormalImage(cv::Mat& depth);
  cv::Mat ConstructSonarImage(cv::Mat& depth, cv::Mat& normals);
  cv::Mat ConstructScanImage(cv::Mat& depth, cv::Mat& SNR);
  void ApplySpeckleNoise(cv::Mat& scan, float fov);
  void ApplySmoothing(cv::Mat& scan, float fov);
  void ApplyMedianFilter(cv::Mat& scan);
  cv::Mat ConstructVisualScanImage(cv::Mat& raw_scan);
  bool FillPointCloudHelper(sensor_msgs::msg::PointCloud2& pc_msg,
    uint32_t rows, uint32_t cols, uint32_t step, void* data);
  bool FillDepthImageHelper(sensor_msgs::msg::Image& img_msg,
    uint32_t rows, uint32_t cols, uint32_t step, void* data);

  std::shared_ptr<rclcpp::Node> rosNode;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthImagePub, normalImagePub,
    multibeamImagePub, sonarImagePub, rawSonarImagePub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPub;

  sensor_msgs::msg::PointCloud2 pointCloudMsg;
  sensor_msgs::msg::Image depthImageMsg, normalImageMsg, multibeamImageMsg,
    sonarImageMsg, rawSonarImageMsg;

  double pointCloudCutoff{0.0};
  std::string pointCloudTopicName, depthImageTopicName, format;
  Entity sensorEntity{kNullEntity};
  std::shared_ptr<gz::sensors::DepthCameraSensor> parentSensor;
  std::shared_ptr<gz::rendering::DepthCamera> depthCamera;
  unsigned int width{0}, height{0}, depth{0};
  gz::common::ConnectionPtr newDepthFrameConnection, newRGBPointCloudConnection, newImageFrameConnection;
  cv::Mat distMatrix;
  std::vector<std::vector<int>> angleRangeIndices;
  std::vector<int> angleNbrIndices;
  std::default_random_engine generator;
};

}}  // namespace gz::sim
#endif
