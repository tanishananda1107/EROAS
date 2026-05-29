// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
// DepthCameraPlugin + GazeboRosCameraUtils no longer exist in gz-sim 8.
// Sensor data comes from gz::sensors::DepthCameraSensor signals.
#include <uuv_sensor_ros_plugins/UnderwaterCameraROSPlugin.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Manager.hh>
#include <gz/sim/components/Sensor.hh>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace gz { namespace sim {

UnderwaterCameraROSPlugin::UnderwaterCameraROSPlugin()
  : lastImage(nullptr), depth2rangeLUT(nullptr)
{}

UnderwaterCameraROSPlugin::~UnderwaterCameraROSPlugin()
{
  delete[] lastImage;
  delete[] depth2rangeLUT;
  if (newDepthFrameConn)   newDepthFrameConn.reset();
  if (newRGBPointCloudConn) newRGBPointCloudConn.reset();
  if (newImageFrameConn)   newImageFrameConn.reset();
}

void UnderwaterCameraROSPlugin::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& _eventMgr)
{
  this->sensorEntity = _entity;
  auto sdfPtr = std::const_pointer_cast<sdf::Element>(_sdf);

  if (!rclcpp::ok())
  {
    gzerr << "ROS 2 not initialized.\n";
    return;
  }

  std::string ns;
  gz::sim::GetSDFParam<std::string>(sdfPtr, "robot_namespace", ns, "underwater_camera");
  this->rosNode = std::make_shared<rclcpp::Node>(ns);

  // image_transport publishers
  image_transport::ImageTransport it(this->rosNode);
  this->imagePub = it.advertise("camera/image_raw", 1);
  this->depthPub = it.advertise("camera/depth/image_raw", 1);
  this->pointCloudPub =
    this->rosNode->create_publisher<sensor_msgs::msg::PointCloud2>(
      "camera/points", 1);
  this->cameraInfoPub =
    this->rosNode->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera/camera_info", 1);

  // attenuation / background from SDF
  GetSDFParam<float>(sdfPtr, "attenuationR", this->attenuation[0], 1.f/30.f);
  GetSDFParam<float>(sdfPtr, "attenuationG", this->attenuation[1], 1.f/30.f);
  GetSDFParam<float>(sdfPtr, "attenuationB", this->attenuation[2], 1.f/30.f);

  if (sdfPtr->HasElement("backgroundR"))
    this->background[0] = (unsigned char)sdfPtr->GetElement("backgroundR")->Get<int>();
  if (sdfPtr->HasElement("backgroundG"))
    this->background[1] = (unsigned char)sdfPtr->GetElement("backgroundG")->Get<int>();
  if (sdfPtr->HasElement("backgroundB"))
    this->background[2] = (unsigned char)sdfPtr->GetElement("backgroundB")->Get<int>();

  // Sensor signals are connected in Update() once the sensor manager
  // has initialised the sensor (sensorEntity → DepthCameraSensor).
}

void UnderwaterCameraROSPlugin::Update(
  const UpdateInfo& _info, EntityComponentManager& _ecm)
{
  // Lazily acquire the gz sensor handle
  if (!this->depthSensor)
  {
    // gz-sim 8: retrieve sensor from the sensor manager via the entity
    auto* sensorManager = _ecm.Component<gz::sim::components::Sensor>(
      this->sensorEntity);
    if (!sensorManager)
      return;

    // In practice you would obtain the sensor from gz::sensors::Manager,
    // which is accessible through the EventManager or a SensorsSystem.
    // This is left as an integration point:
    // this->depthSensor = gz::sensors::Manager::Instance()->
    //   Sensor<gz::sensors::DepthCameraSensor>(this->sensorEntity);
    return;
  }
}

void UnderwaterCameraROSPlugin::OnNewDepthFrame(
  const float* _image, unsigned int /*_w*/, unsigned int /*_h*/,
  unsigned int /*_ch*/, const std::string& /*_fmt*/)
{
  this->lastDepth = _image;
}

void UnderwaterCameraROSPlugin::OnNewRGBPointCloud(
  const float* /*_pcd*/, unsigned int /*_w*/, unsigned int /*_h*/,
  unsigned int /*_ch*/, const std::string& /*_fmt*/)
{}

void UnderwaterCameraROSPlugin::OnNewImageFrame(
  const unsigned char* _image, unsigned int _width, unsigned int _height,
  unsigned int _depth, const std::string& /*_fmt*/)
{
  if (!this->lastDepth || !this->depth2rangeLUT)
    return;

  if (!this->lastImage)
    this->lastImage = new unsigned char[_width * _height * _depth];

  const cv::Mat input(_height, _width, CV_8UC3,
    const_cast<unsigned char*>(_image));
  const cv::Mat depth(_height, _width, CV_32FC1,
    const_cast<float*>(this->lastDepth));
  cv::Mat output(_height, _width, CV_8UC3, this->lastImage);

  this->SimulateUnderwater(input, depth, output);

  // Publish via image_transport
  std_msgs::msg::Header hdr;
  hdr.stamp = this->rosNode->now();
  hdr.frame_id = "camera_optical_frame";

  auto imgMsg = cv_bridge::CvImage(hdr,
    sensor_msgs::image_encodings::BGR8, output).toImageMsg();
  this->imagePub.publish(*imgMsg);
}

void UnderwaterCameraROSPlugin::SimulateUnderwater(
  const cv::Mat& _inputImage, const cv::Mat& _inputDepth, cv::Mat& _outputImage)
{
  const float* lutPtr = this->depth2rangeLUT;
  for (int row = 0; row < _inputImage.rows; ++row)
  {
    const cv::Vec3b* inrow   = _inputImage.ptr<cv::Vec3b>(row);
    const float*     depthrow = _inputDepth.ptr<float>(row);
    cv::Vec3b*       outrow   = _outputImage.ptr<cv::Vec3b>(row);

    for (int col = 0; col < _inputImage.cols; ++col, ++lutPtr)
    {
      float r = (*lutPtr) * depthrow[col];
      if (r < 1e-3f) r = 1e10f;

      const cv::Vec3b& in = inrow[col];
      cv::Vec3b& out = outrow[col];
      for (int c = 0; c < 3; ++c)
      {
        float e = std::exp(-r * this->attenuation[c]);
        out[c] = static_cast<unsigned char>(e * in[c] + (1.f - e) * this->background[c]);
      }
    }
  }
}

GZ_ADD_PLUGIN(UnderwaterCameraROSPlugin, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemUpdate)

}} // namespace gz::sim
