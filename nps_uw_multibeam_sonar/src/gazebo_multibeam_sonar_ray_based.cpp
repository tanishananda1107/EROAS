/*
<<<<<<< HEAD
 * Copyright (C) 2012 Open Source Robotics Foundation
=======
 * Copyright 2020 Naval Postgraduate School
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
<<<<<<< HEAD
 */

// ============================================================
// ROS 2 + Gazebo Harmonic (gz-sim 8) port
// Key changes from ROS1/Gazebo Classic:
//  - Plugin base: gz::sim::System  (replaces SensorPlugin)
//  - Sensor access via gz::sim::EntityComponentManager
//  - GpuLidar sensor (replaces GpuRaySensor / GpuLaser)
//  - rclcpp replaces ros::NodeHandle / roscpp
//  - sensor_msgs, geometry_msgs, cv_bridge all use rclcpp equivalents
//  - pcl_conversions replaces pcl_ros
//  - marine_acoustic_msgs stays (assumed ported to ROS 2 already)
//  - CSV logging unchanged
// ============================================================

#include <assert.h>
#include <sys/stat.h>
#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <limits>
#include <algorithm>
#include <thread>
#include <mutex>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL (ROS 2 / pcl_conversions)
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

// marine_acoustic_msgs (ROS 2 version)
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Gazebo Harmonic (gz-sim 8)
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/GpuLidar.hh>       // replaces GpuRaySensor
#include <gz/sensors/GpuLidarSensor.hh>
#include <gz/sensors/Manager.hh>
#include <gz/math/Angle.hh>

// CUDA sonar calculation (unchanged)
#include <nps_uw_multibeam_sonar/sonar_calculation_cuda.cuh>

// SDF
#include <sdf/Element.hh>

namespace nps_uw_multibeam_sonar
{

/// \brief ROS 2 / Gazebo Harmonic multibeam sonar ray-based plugin.
///
/// Implements gz::sim::System with ISystemConfigure and ISystemPostUpdate.
class NpsGazeboRosMultibeamSonarRay
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  NpsGazeboRosMultibeamSonarRay();
  ~NpsGazeboRosMultibeamSonarRay() override;

  // ISystemConfigure
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  // ISystemPostUpdate  (called every sim step after physics + sensors)
  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  // ---- ROS 2 ----
  rclcpp::Node::SharedPtr ros_node_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       normal_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       sonar_image_pub_;
  rclcpp::Publisher<
    marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_image_raw_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  // Subscriber callback (replaces UpdatePointCloud)
  void UpdatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);

  // ---- Topic / frame names ----
  std::string point_cloud_topic_name_;
  std::string sonar_image_raw_topic_name_;
  std::string sonar_image_topic_name_;
  std::string frame_name_;

  // ---- Sensor geometry ----
  unsigned int width{0}, height{0};   // nBeams, nRays
  std::string format_;

  // ---- Sonar parameters ----
  double verticalFOV{10.0};
  double sonarFreq{900e3};
  double bandwidth{29.5e6};
  double soundSpeed{1500.0};
  double maxDistance{60.0};
  double sourceLevel{220.0};
  int    raySkips{10};
  float  plotScaler{10.0f};
  float  sensorGain{0.02f};
  double absorption{0.0354};
  double attenuation{0.0};
  bool   constMu{true};
  double mu{1e-3};
  int    nBeams{0}, nRays{0};
  int    ray_nElevationRays{0}, ray_nAzimuthRays{1};
  int    nFreq{0};
  float* rangeVector{nullptr};
  float* window{nullptr};
  float* elevation_angles{nullptr};
  float** beamCorrector{nullptr};
  float  beamCorrectorSum{0.0f};
  std::vector<double> azimuth_angles;
  double point_cloud_cutoff_{0.01};

  // ---- Sensor entity / handle ----
  gz::sim::Entity sensor_entity_{gz::sim::kNullEntity};
  // Raw lidar data is read from gz::sensors::GpuLidarSensor
  std::shared_ptr<gz::sensors::GpuLidarSensor> lidar_sensor_;

  // Cached sensor FOV/angle info (populated on first Configure)
  double hFOV_{0.0}, vFOV_{0.0};
  double hPixelSize_{0.0}, vPixelSize_{0.0};
  gz::math::Angle hAngleMin_, hAngleMax_;
  gz::math::Angle vAngleMin_, vAngleMax_;

  // ---- Image / computation state ----
  cv::Mat point_cloud_image_;
  cv::Mat rand_image_;
  cv::Mat reflectivityImage;

  // ---- Messages ----
  sensor_msgs::msg::Image         normal_image_msg_;
  sensor_msgs::msg::Image         sonar_image_msg_;
  marine_acoustic_msgs::msg::ProjectedSonarImage sonar_image_raw_msg_;
  sensor_msgs::msg::PointCloud2   point_cloud_msg_;

  // ---- Synchronisation ----
  std::mutex lock_;

  // ---- Debug / logging ----
  bool  debugFlag{false};
  bool  writeLogFlag{false};
  int   writeInterval{10};
  int   writeCounter{0};
  int   writeNumber{1};
  std::ofstream writeLog;

  // ---- Helpers ----
  void ComputeSonarImage();
  void ComputeCorrector();
  cv::Mat ComputeNormalImage(cv::Mat & depth);

  inline float unnormalized_sinc(float t)
  {
    if (std::abs(t) < 1e-8f) return 1.0f;
    return std::sin(t) / t;
  }

  double focal_length_{1.0};          // used in ComputeNormalImage

  rclcpp::Time sensor_update_time_;   // replaces sensor_update_time_ (ros::Time)

  // Connection count (simple atomic suffices without gazebo's SignalCount)
  int point_cloud_connect_count_{0};
  int sonar_image_connect_count_{0};
};

// ------------------------------------------------------------------ //
//  Constructor / Destructor
// ------------------------------------------------------------------ //

NpsGazeboRosMultibeamSonarRay::NpsGazeboRosMultibeamSonarRay()
{
  writeCounter = 0;
  writeNumber  = 1;
}

NpsGazeboRosMultibeamSonarRay::~NpsGazeboRosMultibeamSonarRay()
{
  writeLog.close();

  delete[] rangeVector;
  delete[] window;
  delete[] elevation_angles;
  if (beamCorrector)
  {
    for (int i = 0; i < nBeams; ++i)
      delete[] beamCorrector[i];
    delete[] beamCorrector;
  }
}

// ------------------------------------------------------------------ //
//  Configure  (replaces Load)
// ------------------------------------------------------------------ //

void NpsGazeboRosMultibeamSonarRay::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  sensor_entity_ = _entity;

  // ---- ROS 2 node ----
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  ros_node_ = std::make_shared<rclcpp::Node>("nps_multibeam_sonar_ray");

  // ---- Topic names from SDF ----
  if (_sdf->HasElement("pointCloudTopicName"))
    point_cloud_topic_name_ = _sdf->Get<std::string>("pointCloudTopicName");
  else
    point_cloud_topic_name_ = "points";

  if (_sdf->HasElement("pointCloudCutoff"))
    point_cloud_cutoff_ = _sdf->Get<double>("pointCloudCutoff");
  else
    point_cloud_cutoff_ = 0.01;

  if (_sdf->HasElement("sonarImageRawTopicName"))
    sonar_image_raw_topic_name_ = _sdf->Get<std::string>("sonarImageRawTopicName");
  else
    sonar_image_raw_topic_name_ = "sonar_image_raw";

  if (_sdf->HasElement("sonarImageTopicName"))
    sonar_image_topic_name_ = _sdf->Get<std::string>("sonarImageTopicName");
  else
    sonar_image_topic_name_ = "sonar_image";

  if (_sdf->HasElement("frameName"))
    frame_name_ = _sdf->Get<std::string>("frameName");
  else
    frame_name_ = "sonar_frame";

  // ---- Sonar physical parameters ----
  verticalFOV  = _sdf->HasElement("verticalFOV")  ? _sdf->Get<double>("verticalFOV")  : 10.0;
  sonarFreq    = _sdf->HasElement("sonarFreq")     ? _sdf->Get<double>("sonarFreq")    : 900e3;
  bandwidth    = _sdf->HasElement("bandwidth")     ? _sdf->Get<double>("bandwidth")    : 29.5e6;
  soundSpeed   = _sdf->HasElement("soundSpeed")    ? _sdf->Get<double>("soundSpeed")   : 1500.0;
  maxDistance  = _sdf->HasElement("maxDistance")   ? _sdf->Get<double>("maxDistance")  : 60.0;
  sourceLevel  = _sdf->HasElement("sourceLevel")   ? _sdf->Get<double>("sourceLevel")  : 220.0;
  raySkips     = _sdf->HasElement("raySkips")      ? _sdf->Get<int>("raySkips")        : 10;
  plotScaler   = _sdf->HasElement("plotScaler")    ? _sdf->Get<float>("plotScaler")    : 10.0f;
  sensorGain   = _sdf->HasElement("sensorGain")    ? _sdf->Get<float>("sensorGain")    : 0.02f;

  if (raySkips == 0) raySkips = 1;

  constMu    = true;
  mu         = 1e-3;
  absorption = 0.0354;
  attenuation = absorption * std::log(10.0) / 20.0;

  // ---- Sensor geometry: obtain from GpuLidar component ----
  // In Gazebo Harmonic the lidar description is in the SDF; actual sensor
  // dimensions (ray counts) come from the SDF <ray> element.
  // We read them here so that nBeams / nRays are available during Configure.
  {
    auto * sdfSensor = _sdf->GetParent() ? _sdf->GetParent().get() : nullptr;
    // Navigate: plugin sdf -> sensor sdf -> lidar/ray
    // Fallback: read directly from <horizontal><samples> and <vertical><samples>
    auto readSamples = [&](const std::string & dir, int def) -> int {
      if (_sdf->GetParent() &&
          _sdf->GetParent()->HasElement(dir) &&
          _sdf->GetParent()->GetElement(dir)->HasElement("samples"))
        return _sdf->GetParent()->GetElement(dir)->Get<int>("samples");
      return def;
    };
    width  = static_cast<unsigned int>(readSamples("horizontal", 512));  // nBeams
    height = static_cast<unsigned int>(readSamples("vertical",   32));   // nRays
  }

  nBeams = static_cast<int>(width);
  nRays  = static_cast<int>(height);
  ray_nElevationRays = nRays;
  ray_nAzimuthRays   = 1;
  elevation_angles   = new float[nRays];

  // ---- Range / frequency vectors ----
  const float max_T  = static_cast<float>(maxDistance * 2.0 / soundSpeed);
  float delta_f      = 1.0f / max_T;
  const float delta_t = 1.0f / static_cast<float>(bandwidth);
  nFreq   = static_cast<int>(std::ceil(bandwidth / delta_f));
  delta_f = static_cast<float>(bandwidth) / nFreq;

  rangeVector = new float[nFreq];
  for (int i = 0; i < nFreq; ++i)
    rangeVector[i] = delta_t * i * static_cast<float>(soundSpeed) / 2.0f;

  // ---- Hamming window ----
  window = new float[nFreq];
  float windowSum = 0.0f;
  for (int f = 0; f < nFreq; ++f)
  {
    window[f] = 0.54f - 0.46f * std::cos(2.0f * M_PI * (f + 1) / nFreq);
    windowSum += window[f] * window[f];
  }
  for (int f = 0; f < nFreq; ++f)
    window[f] /= std::sqrt(windowSum);

  // ---- Beam corrector ----
  beamCorrector = new float*[nBeams];
  for (int i = 0; i < nBeams; ++i)
    beamCorrector[i] = new float[nBeams];
  beamCorrectorSum = 0.0f;

  // ---- Random noise image ----
  rand_image_ = cv::Mat(height, width, CV_32FC2);
  uint64_t randN = static_cast<uint64_t>(std::rand());
  cv::theRNG().state = randN;
  cv::RNG rng = cv::theRNG();
  rng.fill(rand_image_, cv::RNG::NORMAL, 0.0f, 1.0f);

  // ---- Debug / write log flags ----
  debugFlag    = _sdf->HasElement("debugFlag")     ? _sdf->Get<bool>("debugFlag")     : false;
  writeLogFlag = _sdf->HasElement("writeLog")      ? _sdf->Get<bool>("writeLog")      : false;
  if (writeLogFlag)
  {
    writeInterval = _sdf->HasElement("writeFrameInterval") ?
                    _sdf->Get<int>("writeFrameInterval") : 10;

    struct stat buffer;
    std::string logfilename("/tmp/SonarRawData_000001.csv");
    if (stat(logfilename.c_str(), &buffer) == 0)
      system("rm /tmp/SonarRawData*.csv");

    RCLCPP_INFO(ros_node_->get_logger(),
      "Raw data at /tmp/SonarRawData_{numbers}.csv every %d frames", writeInterval);
    RCLCPP_INFO(ros_node_->get_logger(),
      "Beam angles at /tmp/SonarRawData_beam_angles.csv");
  }

  // ---- ROS 2 publishers ----
  point_cloud_pub_ =
    ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_name_, rclcpp::SensorDataQoS());

  normal_image_pub_ =
    ros_node_->create_publisher<sensor_msgs::msg::Image>(
      "/" + point_cloud_topic_name_ + "_normal_image", rclcpp::SensorDataQoS());

  sonar_image_raw_pub_ =
    ros_node_->create_publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>(
      sonar_image_raw_topic_name_, rclcpp::SensorDataQoS());

  sonar_image_pub_ =
    ros_node_->create_publisher<sensor_msgs::msg::Image>(
      sonar_image_topic_name_, rclcpp::SensorDataQoS());

  // ---- ROS 2 subscriber ----
  point_cloud_sub_ =
    ros_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/" + point_cloud_topic_name_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
      {
        this->UpdatePointCloud(msg);
      });

  RCLCPP_INFO(ros_node_->get_logger(), "");
  RCLCPP_INFO(ros_node_->get_logger(), "==================================================");
  RCLCPP_INFO(ros_node_->get_logger(), "============   SONAR PLUGIN LOADED   =============");
  RCLCPP_INFO(ros_node_->get_logger(), "==================================================");
  RCLCPP_INFO(ros_node_->get_logger(), "============       RAY VERSION       =============");
  RCLCPP_INFO(ros_node_->get_logger(), "==================================================");
  RCLCPP_INFO(ros_node_->get_logger(), "Maximum view range  [m] = %.2f", maxDistance);
  RCLCPP_INFO(ros_node_->get_logger(), "# of Beams = %d", nBeams);
  RCLCPP_INFO(ros_node_->get_logger(),
    "# of Rays / Beam (Elevation, Azimuth) = (%d, %d)",
    ray_nElevationRays, ray_nAzimuthRays);
  RCLCPP_INFO(ros_node_->get_logger(), "Calculation skips (Elevation) = %d", raySkips);
  RCLCPP_INFO(ros_node_->get_logger(), "# of Time data / Beam = %d", nFreq);
  RCLCPP_INFO(ros_node_->get_logger(), "==================================================");
}

// ------------------------------------------------------------------ //
//  PostUpdate  (called every sim step)
// ------------------------------------------------------------------ //

void NpsGazeboRosMultibeamSonarRay::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & /*_ecm*/)
{
  // Spin the ROS 2 node to process any incoming point-cloud messages
  rclcpp::spin_some(ros_node_);

  // sensor_update_time_ tracks sim time for message stamps
  sensor_update_time_ = rclcpp::Time(
    static_cast<int64_t>(_info.simTime.count()));

  std::lock_guard<std::mutex> guard(lock_);
  if (sonar_image_connect_count_ > 0 &&
      point_cloud_image_.size().width != 0)
  {
    ComputeSonarImage();
  }
}

// ------------------------------------------------------------------ //
//  UpdatePointCloud  (replaces ROS1 callback)
// ------------------------------------------------------------------ //

void NpsGazeboRosMultibeamSonarRay::UpdatePointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
{
  std::lock_guard<std::mutex> guard(lock_);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*_msg, *pcl_pointcloud);

  point_cloud_image_.create(height, width, CV_32FC1);

  bool angles_calculation_flag = azimuth_angles.empty();

  // Retrieve FOV/angle limits from the sensor SDF.
  // In Gazebo Harmonic these come from gz::sensors::GpuLidarSensor::AngleMin/Max
  // We approximate using the stored hAngleMin_/Max_ populated during Configure.
  // If lidar_sensor_ is available (set externally) use it; otherwise fall back.
  double horzAngleMin = hAngleMin_.Radian();
  double horzAngleMax = hAngleMax_.Radian();
  double vertAngleMin = vAngleMin_.Radian();
  double vertAngleMax = vAngleMax_.Radian();

  for (int j = 0; j < static_cast<int>(nRays); ++j)
  {
    if (angles_calculation_flag)
    {
      const double diff = vertAngleMax - vertAngleMin;
      elevation_angles[j] = static_cast<float>(
        j * diff / (nRays - 1) + vertAngleMin);
    }

    for (int i = 0; i < nBeams; ++i)
    {
      pcl::PointXYZI point = pcl_pointcloud->at(j, width - i - 1);

      point_cloud_image_.at<float>(j, i) =
        std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

      if (angles_calculation_flag && j == 0)
      {
        const double diff = horzAngleMax - horzAngleMin;
        azimuth_angles.push_back(
          i * diff / (nBeams - 1) + horzAngleMin);
      }

      float & px = point_cloud_image_.at<float>(j, i);
      if (std::isnan(px)) px = 100000.0f;
    }
  }

  if (point_cloud_connect_count_ > 0)
    point_cloud_pub_->publish(*_msg);   // re-publish as-is

  sonar_image_connect_count_ = static_cast<int>(
    sonar_image_raw_pub_->get_subscription_count() +
    sonar_image_pub_->get_subscription_count() +
    normal_image_pub_->get_subscription_count());
}

// ------------------------------------------------------------------ //
//  ComputeSonarImage  (main processing, largely unchanged)
// ------------------------------------------------------------------ //

void NpsGazeboRosMultibeamSonarRay::ComputeSonarImage()
{
  cv::Mat depth_image  = point_cloud_image_;
  cv::Mat normal_image = ComputeNormalImage(depth_image);

  // Use cached FOV values
  double vFOV       = vFOV_;
  double hFOV       = hFOV_;
  double vPixelSize = (height > 1) ? vFOV / (height - 1) : vFOV;
  double hPixelSize = (width  > 1) ? hFOV / (width  - 1) : hFOV;

  if (beamCorrectorSum == 0)
    ComputeCorrector();

  if (reflectivityImage.rows == 0)
    reflectivityImage = cv::Mat(width, height, CV_32FC1, cv::Scalar(mu));

  auto start = std::chrono::high_resolution_clock::now();

  CArray2D P_Beams = NpsGazeboSonar::sonar_calculation_wrapper(
    depth_image,
    normal_image,
    rand_image_,
    hPixelSize,
    vPixelSize,
    hFOV,
    vFOV,
    hPixelSize,
    verticalFOV / 180.0 * M_PI,
    hPixelSize,
    elevation_angles,
    vPixelSize * (raySkips + 1),
    soundSpeed,
    maxDistance,
    sourceLevel,
    nBeams,
    nRays,
    raySkips,
    sonarFreq,
    bandwidth,
    nFreq,
    reflectivityImage,
    attenuation,
    window,
    beamCorrector,
    beamCorrectorSum,
    debugFlag);

  auto stop     = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  if (debugFlag)
  {
    RCLCPP_INFO(ros_node_->get_logger(),
      "GPU Sonar Frame Calc Time %ld/100 [s]", duration.count() / 10000);
  }

  // ---- CSV logging ----
  if (writeLogFlag)
  {
    writeCounter++;
    if (writeCounter == 1 || writeCounter % writeInterval == 0)
    {
      double sim_sec = sensor_update_time_.seconds();
      std::stringstream filename;
      filename << "/tmp/SonarRawData_"
               << std::setw(6) << std::setfill('0') << writeNumber << ".csv";
      writeLog.open(filename.str(), std::ios_base::app);
      writeLog << "# Raw Sonar Data Log (Row: beams, Col: time series data)\n";
      writeLog << "# First column is range vector\n";
      writeLog << "#  nBeams : " << nBeams << "\n";
      writeLog << "# Simulation time : " << sim_sec << "\n";
      for (size_t i = 0; i < P_Beams[0].size(); ++i)
      {
        writeLog << rangeVector[i];
        for (size_t b = 0; b < static_cast<size_t>(nBeams); ++b)
        {
          if (P_Beams[b][i].imag() >= 0)
=======
 *
*/
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <assert.h>
#include <sys/stat.h>
#include <tf/tf.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <nps_uw_multibeam_sonar/sonar_calculation_cuda.cuh>

#include <opencv2/core/core.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <nps_uw_multibeam_sonar/gazebo_multibeam_sonar_raster_based.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>

#include <algorithm>
#include <string>
#include <vector>
#include <limits>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(NpsGazeboRosMultibeamSonar)


// Constructor
NpsGazeboRosMultibeamSonar::NpsGazeboRosMultibeamSonar() :
  SensorPlugin(), width(0), height(0), depth(0)
{
  this->depth_image_connect_count_ = 0;
  this->depth_info_connect_count_ = 0;
  this->point_cloud_connect_count_ = 0;
  this->sonar_image_connect_count_ = 0;
  this->last_depth_image_camera_info_update_time_ = common::Time(0);

  // frame counter for variational reflectivity
  this->maxDepth_before = 0.0;
  this->maxDepth_beforebefore = 0.0;
  this->maxDepth_prev = 0.0;

  // FIX 2: calculateReflectivity was never initialised — UB when first read in
  // OnNewImageFrame before any assignment.
  this->calculateReflectivity = false;

  // for csv write logs
  this->writeCounter = 0;
  this->writeNumber = 1;
}


// Destructor
NpsGazeboRosMultibeamSonar::~NpsGazeboRosMultibeamSonar()
{
  this->newDepthFrameConnection.reset();
  this->newImageFrameConnection.reset();
  this->newRGBPointCloudConnection.reset();

  this->parentSensor.reset();
  this->depthCamera.reset();

  // CSV log write stream close
  writeLog.close();
}


// Load the controller
void NpsGazeboRosMultibeamSonar::Load(sensors::SensorPtr _parent,
                                  sdf::ElementPtr _sdf)
{
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_parent);

  // FIX 1: The original code dereferenced depthCamera (via parentSensor) to read
  // width/height/depth/format BEFORE checking whether parentSensor is non-null.
  // A null parentSensor means depthCamera is also null, so those reads crash.
  // Guard must come immediately after the cast, before any dereference.
  if (!this->parentSensor)
  {
    gzerr << "DepthCameraPlugin not attached to a depthCamera sensor\n";
    return;
  }

  this->depthCamera = this->parentSensor->DepthCamera();
  this->world = physics::get_world(parentSensor->WorldName());

  this->width = this->depthCamera->ImageWidth();
  this->height = this->depthCamera->ImageHeight();
  this->depth = this->depthCamera->ImageDepth();
  this->format = this->depthCamera->ImageFormat();

  // FIX 4: focal_length_ is used in ComputeNormalImage (divide by it for the
  // blue channel) but was never assigned anywhere in Load().  A zero value
  // produces divide-by-zero / inf normals.  Compute it from HFOV and image
  // width exactly as every other place in this file does.
  double hfov_load = this->depthCamera->HFOV().Radian();
  this->focal_length_ = static_cast<double>(this->width) /
                        (2.0 * tan(hfov_load / 2.0));

  this->newDepthFrameConnection =
    this->depthCamera->ConnectNewDepthFrame(
        std::bind(&NpsGazeboRosMultibeamSonar::OnNewDepthFrame,
                  this, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5));

  this->newImageFrameConnection =
    this->depthCamera->ConnectNewImageFrame(
        std::bind(&NpsGazeboRosMultibeamSonar::OnNewImageFrame,
                  this, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5));

  this->parentSensor->SetActive(true);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("depth_camera", "A ROS node for Gazebo "
        << "has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so'"
        << " in the gazebo_ros package)");
    return;
  }

  // copying from DepthCameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->depthCamera;

  // not using default GazeboRosCameraUtils topics
  if (!_sdf->HasElement("imageTopicName"))
    this->image_topic_name_ = "ir/image_raw";
  if (!_sdf->HasElement("cameraInfoTopicName"))
    this->camera_info_topic_name_ = "ir/camera_info";

  // depth image stuff
  if (!_sdf->HasElement("depthImageTopicName"))
    this->depth_image_topic_name_ = "depth/image_raw";
  else
    this->depth_image_topic_name_ =
      _sdf->GetElement("depthImageTopicName")->Get<std::string>();

  if (!_sdf->HasElement("depthImageCameraInfoTopicName"))
    this->depth_image_camera_info_topic_name_ = "depth/camera_info";
  else
    this->depth_image_camera_info_topic_name_ =
      _sdf->GetElement("depthImageCameraInfoTopicName")->Get<std::string>();

  if (!_sdf->HasElement("pointCloudTopicName"))
    this->point_cloud_topic_name_ = "points";
  else
    this->point_cloud_topic_name_ =
        _sdf->GetElement("pointCloudTopicName")->Get<std::string>();

  if (!_sdf->HasElement("pointCloudCutoff"))
    this->point_cloud_cutoff_ = 0.01;
  else
    this->point_cloud_cutoff_ =
        _sdf->GetElement("pointCloudCutoff")->Get<double>();

  // sonar stuff
  if (!_sdf->HasElement("sonarImageRawTopicName"))
    this->sonar_image_raw_topic_name_ = "sonar_image_raw";
  else
    this->sonar_image_raw_topic_name_ =
      _sdf->GetElement("sonarImageRawTopicName")->Get<std::string>();
  if (!_sdf->HasElement("sonarImageTopicName"))
    this->sonar_image_topic_name_ = "sonar_image";
  else
    this->sonar_image_topic_name_ =
      _sdf->GetElement("sonarImageTopicName")->Get<std::string>();

  // Read sonar properties from model.sdf
  if (!_sdf->HasElement("verticalFOV"))
    this->verticalFOV = 10;  // Blueview P900 -> 10 degrees
  else
    this->verticalFOV =
      _sdf->GetElement("verticalFOV")->Get<double>();
  if (!_sdf->HasElement("sonarFreq"))
    this->sonarFreq = 900e3;  // Blueview P900 [Hz]
  else
    this->sonarFreq =
      _sdf->GetElement("sonarFreq")->Get<double>();
  if (!_sdf->HasElement("bandwidth"))
    this->bandwidth = 29.5e6;  // Blueview P900 [Hz]
  else
    this->bandwidth =
      _sdf->GetElement("bandwidth")->Get<double>();
  if (!_sdf->HasElement("soundSpeed"))
    this->soundSpeed = 1500;
  else
    this->soundSpeed =
      _sdf->GetElement("soundSpeed")->Get<double>();
  if (!_sdf->HasElement("maxDistance"))
    this->maxDistance = 60;
  else
    this->maxDistance =
      _sdf->GetElement("maxDistance")->Get<double>();
  if (!_sdf->HasElement("sourceLevel"))
    this->sourceLevel = 220;
  else
    this->sourceLevel =
      _sdf->GetElement("sourceLevel")->Get<double>();
  if (!_sdf->HasElement("constantReflectivity"))
    this->constMu = true;
  else
    this->constMu =
      _sdf->GetElement("constantReflectivity")->Get<bool>();
  if (!_sdf->HasElement("artificialVehicleVibration"))
    this->artificialVehicleVibration = false;
  else
    this->artificialVehicleVibration =
      _sdf->GetElement("artificialVehicleVibration")->Get<bool>();
  if (!_sdf->HasElement("customSDFTagReflectivity"))
    this->customTag = false;
  else
    this->customTag =
      _sdf->GetElement("customSDFTagReflectivity")->Get<bool>();
  if (!_sdf->HasElement("raySkips"))
    this->raySkips = 10;
  else
    this->raySkips =
      _sdf->GetElement("raySkips")->Get<int>();
  if (!_sdf->HasElement("plotScaler"))
    this->plotScaler = 10;
  else
    this->plotScaler =
      _sdf->GetElement("plotScaler")->Get<float>();
  if (!_sdf->HasElement("sensorGain"))
    this->sensorGain = 0.02;
  else
    this->sensorGain =
      _sdf->GetElement("sensorGain")->Get<float>();
  // Configure skips
  if (this->raySkips == 0) this->raySkips = 1;

  // --- Variational Reflectivity --- //
  // Read the variational reflectivity database file path from the SDF file
  if (!this->constMu)
  {
    if (!this->customTag)
    {
      if (!_sdf->HasElement("reflectivityDatabaseFile"))
      {
        this->reflectivityDatabaseFileName = "variationalReflectivityDatabase.csv";
      }
      else
      {
        this->reflectivityDatabaseFileName =
          _sdf->GetElement("reflectivityDatabaseFile")->Get<std::string>();
        GZ_ASSERT(!this->reflectivityDatabaseFileName.empty(),
          "Empty variational reflectivity database file name");
      }
    }
    else
    {
      if (!_sdf->HasElement("customSDFTagDatabaseFile"))
      {
        this->customTagDatabaseFileName = "customSDFTagDatabase.csv";
      }
      else
      {
        this->customTagDatabaseFileName =
          _sdf->GetElement("customSDFTagDatabaseFile")->Get<std::string>();
        GZ_ASSERT(!this->customTagDatabaseFileName.empty(),
          "Empty custom SDF Tag database file name");
      }
    }

    // FIX 3: The original code unconditionally built these paths and opened the
    // CSV regardless of constMu.  When constMu==true the filename strings are
    // default-constructed (empty), so fopen/open silently opens a file called
    // ".../worlds/" and the getline loop reads garbage or nothing, leaving
    // objectNames/reflectivities empty — but only after wasting time trying.
    // Keep path construction and CSV read inside the !constMu guard.
    this->reflectivityDatabaseFilePath =
      ros::package::getPath("nps_uw_multibeam_sonar")
          + "/worlds/" + this->reflectivityDatabaseFileName;
    this->customTagDatabaseFilePath =
      ros::package::getPath("nps_uw_multibeam_sonar")
          + "/worlds/" + this->customTagDatabaseFileName;

    // Read csv file
    std::ifstream csvFile; std::string line;
    if (!this->customTag)
      csvFile.open(this->reflectivityDatabaseFilePath);
    else
      csvFile.open(this->customTagDatabaseFilePath);
    // skip the 3 header lines
    getline(csvFile, line); getline(csvFile, line); getline(csvFile, line);
    while (getline(csvFile, line))
    {
        if (line.empty())  // skip empty lines:
        {
            continue;
        }
        std::istringstream iss(line);
        std::string lineStream;
        std::string::size_type sz;
        std::vector <std::string> row;
        while (getline(iss, lineStream, ','))
        {
            row.push_back(lineStream);
        }
        this->objectNames.push_back(row[0]);
        this->reflectivities.push_back(stof(row[1], &sz));
    }

    // Read coefficient for Biofouling and roughness
    if (this->customTag)
    {
      for (int k=0; k<(int)objectNames.size(); k++)
      {
        if (objectNames[k] == "biofouling_rating")
          this->biofouling_rating_coeff = reflectivities[k];
        if (objectNames[k] == "roughness")
          this->roughness_coeff = reflectivities[k];
      }
    }
  }  // end of !constMu block

  this->mu = 1e-3;  // default constant mu

  // From FiducialCameraPlugin
  if (this->depthCamera)
  {
    this->scene = this->depthCamera->GetScene();
  }
  if (!this->depthCamera || !this->scene)
  {
    gzerr << "SonarDummy failed to load. "
        << "Camera and/or Scene not found" << std::endl;
  }
  // load the fiducials
  if (_sdf->HasElement("fiducial"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("fiducial");
    while (elem)
    {
      this->fiducials.insert(elem->Get<std::string>());
      elem = elem->GetNextElement("fiducial");
    }
  }
  else
  {
    gzmsg << "No fiducials specified. All models will be tracked."
        << std::endl;
    this->detectAll = true;
  }

  // Transmission path properties (typical model used here)
  // More sophisticated model by Francois-Garrison model is available
  this->absorption = 0.0354;  // [dB/m]
  this->attenuation = this->absorption*log(10)/20.0;

  // Range vector
  const float max_T = this->maxDistance*2.0/this->soundSpeed;
  float delta_f = 1.0/max_T;
  const float delta_t = 1.0/this->bandwidth;
  this->nFreq = ceil(this->bandwidth/delta_f);
  delta_f = this->bandwidth/this->nFreq;
  const int nTime = nFreq;
  this->rangeVector = new float[nTime];
  for (int i = 0; i < nTime; i++)
  {
    this->rangeVector[i] = delta_t*i*this->soundSpeed/2.0;
  }

  // FOV, Number of beams, number of rays are defined at model.sdf
  // Currently, this->width equals # of beams, and this->height equals # of rays
  // Each beam consists of (elevation,azimuth)=(this->height,1) rays
  // Beam patterns
  this->nBeams = this->width;
  this->nRays = this->height;
  this->ray_nElevationRays = this->height;
  this->ray_nAzimuthRays = 1;
  this->elevation_angles = new float[this->nRays];

  // Print sonar calculation settings
  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("==================================================");
  ROS_INFO_STREAM("============   SONAR PLUGIN LOADED   =============");
  ROS_INFO_STREAM("==================================================");
  ROS_INFO_STREAM("============      RASTER VERSION     =============");
  ROS_INFO_STREAM("==================================================");
  ROS_INFO_STREAM("Maximum view range  [m] = " << this->maxDistance);
  ROS_INFO_STREAM("Distance resolution [m] = " <<
                    this->soundSpeed*(1.0/(this->nFreq*delta_f)));
  ROS_INFO_STREAM("# of Beams = " << this->nBeams);
  ROS_INFO_STREAM("# of Rays / Beam (Elevation, Azimuth) = ("
      << ray_nElevationRays << ", " << ray_nAzimuthRays << ")");
  ROS_INFO_STREAM("Calculation skips (Elevation) = "
      << this->raySkips);
  ROS_INFO_STREAM("# of Time data / Beam = " << this->nFreq);
  if (!this->constMu)
  {
    if (this->customTag)
      ROS_INFO_STREAM("Reflectivity method : Variational (based on custon SDF tag)");
    else
      ROS_INFO_STREAM("Reflectivity method : Variational (based on model name)");
  }
  else
  {
      ROS_INFO_STREAM("Reflectivity method : Constant");
  }
  ROS_INFO_STREAM("==================================================");
  ROS_INFO_STREAM("");

  // get writeLog Flag
  if (!_sdf->HasElement("writeLog"))
    this->writeLogFlag = false;
  else
  {
    this->writeLogFlag = _sdf->Get<bool>("writeLog");
    if (this->writeLogFlag)
    {
      if (_sdf->HasElement("writeFrameInterval"))
        this->writeInterval = _sdf->Get<int>("writeFrameInterval");
      else
        this->writeInterval = 10;
      ROS_INFO_STREAM("Raw data at " << "/tmp/SonarRawData_{numbers}.csv");
      ROS_INFO_STREAM("every " << this->writeInterval << " frames");
      ROS_INFO_STREAM("");

      struct stat buffer;
      std::string logfilename("/tmp/SonarRawData_000001.csv");
      if (stat (logfilename.c_str(), &buffer) == 0)
        system("rm /tmp/SonarRawData*.csv");
    }
  }

  // Get debug flag for computation time display
  if (!_sdf->HasElement("debugFlag"))
    this->debugFlag = false;
  else
    this->debugFlag =
      _sdf->GetElement("debugFlag")->Get<bool>();

  // -- Pre calculations for sonar -- //
  // rand number generator
  this->rand_image = cv::Mat(this->height, this->width, CV_32FC2);
  uint64 randN = static_cast<uint64>(std::rand());
  cv::theRNG().state = randN;
  cv::RNG rng = cv::theRNG();
  rng.fill(this->rand_image, cv::RNG::NORMAL, 0.f, 1.0f);

  // Hamming window
  this->window = new float[this->nFreq];
  float windowSum = 0;
  for (size_t f = 0; f < this->nFreq; f++)
  {
    this->window[f] = 0.54 - 0.46 * cos(2.0*M_PI*(f+1)/this->nFreq);
    windowSum += pow(this->window[f], 2.0);
  }
  for (size_t f = 0; f < this->nFreq; f++)
    this->window[f] = this->window[f]/sqrt(windowSum);

  // Sonar corrector preallocation
  this->beamCorrector = new float*[nBeams];
  for (int i = 0; i < nBeams; i++)
      this->beamCorrector[i] = new float[nBeams];
  this->beamCorrectorSum = 0.0;

  load_connection_ =
    GazeboRosCameraUtils::OnLoad(
            boost::bind(&NpsGazeboRosMultibeamSonar::Advertise, this));
  GazeboRosCameraUtils::Load(_parent, _sdf);
}

void NpsGazeboRosMultibeamSonar::PopulateFiducials()
{
  this->fiducials.clear();

  // Check all models for inclusion in the frustum.
  rendering::VisualPtr worldVis = this->scene->WorldVisual();
  for (unsigned int i = 0; i < worldVis->GetChildCount(); ++i)
  {
    rendering::VisualPtr childVis = worldVis->GetChild(i);
    if (childVis->GetType() == rendering::Visual::VT_MODEL)
      this->fiducials.insert(childVis->Name());
  }
}

void NpsGazeboRosMultibeamSonar::Advertise()
{
  ros::AdvertiseOptions depth_image_ao =
    ros::AdvertiseOptions::create<sensor_msgs::Image>(
      this->depth_image_topic_name_, 1,
      boost::bind(&NpsGazeboRosMultibeamSonar::DepthImageConnect, this),
      boost::bind(&NpsGazeboRosMultibeamSonar::DepthImageDisconnect, this),
      ros::VoidPtr(), &this->camera_queue_);
  this->depth_image_pub_ = this->rosnode_->advertise(depth_image_ao);

  ros::AdvertiseOptions depth_image_camera_info_ao =
    ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
        this->depth_image_camera_info_topic_name_, 1,
        boost::bind(&NpsGazeboRosMultibeamSonar::DepthInfoConnect, this),
        boost::bind(&NpsGazeboRosMultibeamSonar::DepthInfoDisconnect, this),
        ros::VoidPtr(), &this->camera_queue_);
  this->depth_image_camera_info_pub_ =
    this->rosnode_->advertise(depth_image_camera_info_ao);

  ros::AdvertiseOptions normal_image_ao =
    ros::AdvertiseOptions::create<sensor_msgs::Image>(
      this->depth_image_topic_name_+"_normals", 1,
      boost::bind(&NpsGazeboRosMultibeamSonar::NormalImageConnect, this),
      boost::bind(&NpsGazeboRosMultibeamSonar::NormalImageDisconnect, this),
      ros::VoidPtr(), &this->camera_queue_);
  this->normal_image_pub_ = this->rosnode_->advertise(normal_image_ao);

  ros::AdvertiseOptions point_cloud_ao =
    ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
      this->point_cloud_topic_name_, 1,
      boost::bind(&NpsGazeboRosMultibeamSonar::PointCloudConnect, this),
      boost::bind(&NpsGazeboRosMultibeamSonar::PointCloudDisconnect, this),
      ros::VoidPtr(), &this->camera_queue_);
  this->point_cloud_pub_ = this->rosnode_->advertise(point_cloud_ao);

  // FIX 6: The original code wired both sonar publishers to DepthImageConnect/
  // DepthImageDisconnect.  Every sonar subscriber therefore incremented
  // depth_image_connect_count_ instead of sonar_image_connect_count_, making
  // the sensor-activation logic in OnNewDepthFrame miscount subscribers and
  // either never deactivate the sensor or never reactivate it correctly.
  // Use the dedicated SonarImageConnect/Disconnect callbacks instead.
  ros::AdvertiseOptions sonar_image_raw_ao =
    ros::AdvertiseOptions::create<marine_acoustic_msgs::ProjectedSonarImage>(
      this->sonar_image_raw_topic_name_, 1,
      boost::bind(&NpsGazeboRosMultibeamSonar::SonarImageConnect, this),
      boost::bind(&NpsGazeboRosMultibeamSonar::SonarImageDisconnect, this),
      ros::VoidPtr(), &this->camera_queue_);
  this->sonar_image_raw_pub_ = this->rosnode_->advertise(sonar_image_raw_ao);

  ros::AdvertiseOptions sonar_image_ao =
    ros::AdvertiseOptions::create<sensor_msgs::Image>(
      this->sonar_image_topic_name_, 1,
      boost::bind(&NpsGazeboRosMultibeamSonar::SonarImageConnect, this),
      boost::bind(&NpsGazeboRosMultibeamSonar::SonarImageDisconnect, this),
      ros::VoidPtr(), &this->camera_queue_);
  this->sonar_image_pub_ = this->rosnode_->advertise(sonar_image_ao);
}


//----------------------------------------------------------------
// Increment and decriment a connection counter so that the sensor
// is only active and ROS messages being published when required
//----------------------------------------------------------------

void NpsGazeboRosMultibeamSonar::DepthImageConnect()
{
  this->depth_image_connect_count_++;
  this->parentSensor->SetActive(true);
}

void NpsGazeboRosMultibeamSonar::DepthImageDisconnect()
{
  this->depth_image_connect_count_--;
}

void NpsGazeboRosMultibeamSonar::NormalImageConnect()
{
  this->depth_image_connect_count_++;
  this->parentSensor->SetActive(true);
}

void NpsGazeboRosMultibeamSonar::NormalImageDisconnect()
{
  this->depth_image_connect_count_--;
}

void NpsGazeboRosMultibeamSonar::DepthInfoConnect()
{
  this->depth_info_connect_count_++;
}

void NpsGazeboRosMultibeamSonar::DepthInfoDisconnect()
{
  this->depth_info_connect_count_--;
}

// FIX 6 (continued): These callbacks now actually serve the sonar publishers.
void NpsGazeboRosMultibeamSonar::SonarImageConnect()
{
  this->sonar_image_connect_count_++;
  this->parentSensor->SetActive(true);
}

void NpsGazeboRosMultibeamSonar::SonarImageDisconnect()
{
  this->sonar_image_connect_count_--;
}

void NpsGazeboRosMultibeamSonar::PointCloudConnect()
{
  this->point_cloud_connect_count_++;
  (*this->image_connect_count_)++;
  this->parentSensor->SetActive(true);
}

void NpsGazeboRosMultibeamSonar::PointCloudDisconnect()
{
  this->point_cloud_connect_count_--;
  (*this->image_connect_count_)--;
  if (this->point_cloud_connect_count_ <= 0)
    this->parentSensor->SetActive(false);
}

// Update everything when Gazebo provides a new depth frame (texture)
void NpsGazeboRosMultibeamSonar::OnNewDepthFrame(const float *_image,
                                             unsigned int _width,
                                             unsigned int _height,
                                             unsigned int _depth,
                                             const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->depth_sensor_update_time_ = this->parentSensor->LastMeasurementTime();

  if (this->parentSensor->IsActive())
  {
    // Deactivate if no subscribers on any topic
    if (this->depth_image_connect_count_ <= 0 &&
        this->point_cloud_connect_count_ <= 0 &&
        this->sonar_image_connect_count_ <= 0 &&
        (*this->image_connect_count_) <= 0)
    {
      this->parentSensor->SetActive(false);
    }
    else
    {
      this->ComputePointCloud(_image);

      if (this->depth_image_connect_count_ > 0 ||
          this->sonar_image_connect_count_ > 0)
        this->ComputeSonarImage(_image);
    }
  }
  else
  {
    // FIX 7: The original condition was:
    //   depth_image_connect_count_ <= 0 || point_cloud_connect_count_ > 0
    // The first clause is backwards (fires when there are NO depth subscribers),
    // and sonar subscribers were not considered at all.  The sensor should
    // reactivate whenever ANY subscriber is present.
    if (this->depth_image_connect_count_ > 0 ||
        this->point_cloud_connect_count_ > 0 ||
        this->sonar_image_connect_count_ > 0)
      this->parentSensor->SetActive(true);
  }
}


// Process the camera image when Gazebo provides one.
void NpsGazeboRosMultibeamSonar::OnNewImageFrame(const unsigned char *_image,
                                             unsigned int _width,
                                             unsigned int _height,
                                             unsigned int _depth,
                                             const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->sensor_update_time_ = this->parentSensor->LastMeasurementTime();

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor
      // to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
    {
      this->PutCameraData(_image);
    }
  }

  // Calculate only if the maxDepth from depth camera is changed and stabled
  double min; cv::minMaxLoc(this->point_cloud_image_, &min, &this->maxDepth);
  if (this->maxDepth == this->maxDepth_before
      && this->maxDepth == this->maxDepth_beforebefore
      && this->calculateReflectivity == false
      && this->maxDepth != this->maxDepth_prev)
  {
    this->calculateReflectivity = true;
    this->maxDepth_prev = this->maxDepth;

    // Regenerate rand image
    uint64 randN = static_cast<uint64>(std::rand());
    cv::theRNG().state = randN;
    cv::RNG rng = cv::theRNG();
    rng.fill(this->rand_image, cv::RNG::NORMAL, 0.f, 1.f);
  }
  else
    this->calculateReflectivity = false;

  this->maxDepth_beforebefore = this->maxDepth_before;
  this->maxDepth_before = this->maxDepth;

  // For variational reflectivity
  if (!this->constMu)
  {
    if (calculateReflectivity)
    {
      // Generate reflectivity opencv image palette
      cv::Mat reflectivity_image = cv::Mat(width, height, CV_32FC1, cv::Scalar(this->mu));

      if (!this->selectionBuffer)
      {
        std::string cameraName = this->camera_->OgreCamera()->getName();
        this->selectionBuffer.reset(
            new rendering::SelectionBuffer(cameraName,
            this->scene->OgreSceneManager(),
            this->camera_->RenderTexture()->getBuffer()->
            getRenderTarget()));
      }

      if (this->detectAll)
        this->PopulateFiducials();

      std::vector<FiducialData> results;
      for (const auto &f : this->fiducials)
      {
        // check if fiducial is visible within the frustum
        rendering::VisualPtr vis = this->scene->GetVisual(f);
        if (!vis)
          continue;

        if (!this->depthCamera->IsVisible(vis))
          continue;

        ROS_INFO_STREAM("Calculating Reflectivity of captured objects using custom SDF Tags");
        ROS_INFO_STREAM("This may take quite some time for the first frame");

        // Loop over every pixel
        for (int i=0; i<reflectivity_image.rows; i++)
        {
          for (int j=0; j<reflectivity_image.cols; j+=raySkips)
          {
            // target pixel
            ignition::math::Vector2i pt = ignition::math::Vector2i(i, j);

            // use selection buffer to check if visual is occluded by other entities
            // in the camera view
            Ogre::Entity *entity =
              this->selectionBuffer->OnSelectionClick(pt.X(), pt.Y());

            rendering::VisualPtr result;
            if (entity && !entity->getUserObjectBindings().getUserAny().isEmpty())
            {
              try
              {
                result = this->scene->GetVisual(
                    Ogre::any_cast<std::string>(
                    entity->getUserObjectBindings().getUserAny()));
              }
              catch(Ogre::Exception &_e)
              {
                gzerr << "Ogre Error:" << _e.getFullDescription() << "\n";
                continue;
              }
            }

            if (result && result->GetRootVisual() == vis)
            {
              FiducialData fd;
              fd.id = vis->Name();
              fd.pt = pt;

              // Assign variational reflectivity
              if (!this->customTag)
              {
                for (int k=0; k<(int)objectNames.size(); k++)
                  if (vis->Name() == objectNames[k])
                    reflectivity_image.at<float>(j, i) = reflectivities[k];
              }
              else
              {
                // Read custom tags for surface properties
                sdf::ElementPtr modelElt =
                  this->world->BaseByName(vis->Name())->GetSDF();

                int biofoulingRating = 0; // Biofouling rating, [0, 100]
                if (modelElt->HasElement("surface_props:biofouling_rating"))
                  biofoulingRating = modelElt->Get<int>("surface_props:biofouling_rating");

                double roughness = 0.0; // Surface roughness, [0.0, 1.0]
                if (modelElt->HasElement("surface_props:roughness"))
                  roughness = modelElt->Get<double>("surface_props:roughness");

                std::string material = "default"; // Surface material
                if (modelElt->HasElement("surface_props:material"))
                  material = modelElt->Get<std::string>("surface_props:material");

                for (int k=0; k<(int)objectNames.size(); k++)
                  if (material == objectNames[k])
                    reflectivity_image.at<float>(j, i) =
                      reflectivities[k] * (1.0/(roughness + 1)) / this->roughness_coeff
                      * (1.0/(biofoulingRating + 1)) / this->biofouling_rating_coeff;

              }
            }
          }
        }  // end of pixel loop
      }  // end of selection buffer

      // Save reflectivity image
      this->reflectivityImage = reflectivity_image;
    }  // end of variational reflectivity calculation
  }  // end of variational reflectivity bool

}

// Most of the plugin work happens here
void NpsGazeboRosMultibeamSonar::ComputeSonarImage(const float *_src)
{
  this->lock_.lock();
  cv::Mat depth_image = this->point_cloud_image_;
  cv::Mat normal_image = this->ComputeNormalImage(depth_image);
  double vFOV = this->parentSensor->DepthCamera()->VFOV().Radian();
  double hFOV = this->parentSensor->DepthCamera()->HFOV().Radian();
  double vPixelSize = vFOV / this->height;
  double hPixelSize = hFOV / this->width;

  if (this->beamCorrectorSum == 0)
    ComputeCorrector();

  // Default value for reflectivity
  if (this->reflectivityImage.rows == 0)
    this->reflectivityImage = cv::Mat(width, height, CV_32FC1, cv::Scalar(this->mu));

  // If artifical vehicle vibration flag is on
  if (this->artificialVehicleVibration)
  {
    // Regenerate rand image
    uint64 randN = static_cast<uint64>(std::rand());
    cv::theRNG().state = randN;
    cv::RNG rng = cv::theRNG();
    rng.fill(this->rand_image, cv::RNG::NORMAL, 0.f, 1.f);
  }

  // For calc time measure
  auto start = std::chrono::high_resolution_clock::now();
  // ------------------------------------------------//
  // --------      Sonar calculations       -------- //
  // ------------------------------------------------//
  CArray2D P_Beams = NpsGazeboSonar::sonar_calculation_wrapper(
                  depth_image,   // cv::Mat& depth_image
                  normal_image,  // cv::Mat& normal_image
                  rand_image,    // cv::Mat& rand_image
                  hPixelSize,    // hPixelSize
                  vPixelSize,    // vPixelSize
                  hFOV,          // hFOV
                  vFOV,          // VFOV
                  hPixelSize,    // _beam_azimuthAngleWidth
                  verticalFOV/180*M_PI,  // _beam_elevationAngleWidth
                  hPixelSize,    // _ray_azimuthAngleWidth
                  this->elevation_angles, // _ray_elevationAngles
                  vPixelSize*(raySkips+1),  // _ray_elevationAngleWidth
                  this->soundSpeed,    // _soundSpeed
                  this->maxDistance,   // _maxDistance
                  this->sourceLevel,   // _sourceLevel
                  this->nBeams,        // _nBeams
                  this->nRays,         // _nRays
                  this->raySkips,      // _raySkips
                  this->sonarFreq,     // _sonarFreq
                  this->bandwidth,     // _bandwidth
                  this->nFreq,         // _nFreq
                  this->reflectivityImage,  // reflectivity_image
                  this->attenuation,   // _attenuation
                  this->window,        // _window
                  this->beamCorrector,      // _beamCorrector
                  this->beamCorrectorSum,   // _beamCorrectorSum
                  this->debugFlag);

  // For calc time measure
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<
                  std::chrono::microseconds>(stop - start);
  if (debugFlag)
  {
    ROS_INFO_STREAM("GPU Sonar Frame Calc Time " <<
                    duration.count()/10000 << "/100 [s]\n");
  }

  // CSV log write stream
  // Each cols corresponds to each beams
  if (this->writeLogFlag)
  {
    this->writeCounter = this->writeCounter + 1;
    if (this->writeCounter == 1
        ||this->writeCounter % this->writeInterval == 0)
    {
      double time = this->parentSensor_->LastMeasurementTime().Double();
      std::stringstream filename;
      filename << "/tmp/SonarRawData_" << std::setw(6) <<  std::setfill('0')
               << this->writeNumber << ".csv";
      writeLog.open(filename.str().c_str(), std::ios_base::app);
      filename.clear();
      writeLog << "# Raw Sonar Data Log (Row: beams, Col: time series data)\n";
      writeLog << "# First column is range vector\n";
      writeLog << "#  nBeams : " << nBeams << "\n";
      writeLog << "# Simulation time : " << time << "\n";
      for (size_t i = 0; i < P_Beams[0].size(); i++)
      {
        // writing range vector at first column
        writeLog << this->rangeVector[i];
        for (size_t b = 0; b < nBeams; b ++)
        {
          if (P_Beams[b][i].imag() > 0)
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
            writeLog << "," << P_Beams[b][i].real()
                     << "+" << P_Beams[b][i].imag() << "i";
          else
            writeLog << "," << P_Beams[b][i].real()
                     << P_Beams[b][i].imag() << "i";
        }
        writeLog << "\n";
      }
      writeLog.close();

<<<<<<< HEAD
      if (writeNumber == 1)
      {
        std::ofstream angle_log("/tmp/SonarRawData_beam_angles.csv", std::ios_base::app);
        angle_log << "# Raw Sonar Data Log \n";
        angle_log << "# Beam (azimuth) angles of rays\n";
        angle_log << "#  nBeams : " << nBeams << "\n";
        angle_log << "# Simulation time : " << sim_sec << "\n";
        for (auto & a : azimuth_angles)
          angle_log << a << "\n";
        angle_log.close();
      }
      writeNumber++;
    }
  }

  // ---- Build sonar_image_raw (marine_acoustic_msgs) ----
  auto & raw = sonar_image_raw_msg_;
  raw.header.frame_id        = frame_name_;
  raw.header.stamp           = sensor_update_time_;

  marine_acoustic_msgs::msg::PingInfo ping_info;
  ping_info.frequency   = static_cast<float>(sonarFreq);
  ping_info.sound_speed = static_cast<float>(soundSpeed);
  for (int beam = 0; beam < nBeams; ++beam)
  {
    ping_info.rx_beamwidths.push_back(
      static_cast<float>(hFOV / std::floor(nBeams * 2.0 - 2.0) * 2.0));
    ping_info.tx_beamwidths.push_back(static_cast<float>(vFOV));
  }
  raw.ping_info = ping_info;

  raw.beam_directions.clear();
  for (int beam = 0; beam < nBeams; ++beam)
  {
    geometry_msgs::msg::Vector3 dir;
    dir.x = std::cos(azimuth_angles[beam]);
    dir.y = std::sin(azimuth_angles[beam]);
    dir.z = 0.0;
    raw.beam_directions.push_back(dir);
  }

  std::vector<float> ranges;
  ranges.reserve(nFreq);
  for (int i = 0; i < nFreq; ++i)
    ranges.push_back(rangeVector[i]);
  raw.ranges = ranges;

  marine_acoustic_msgs::msg::SonarImageData sonar_data;
  sonar_data.is_bigendian = false;
  sonar_data.dtype        = 0;   // DTYPE_UINT8
  sonar_data.beam_count   = static_cast<uint32_t>(nBeams);

  std::vector<uint8_t> intensities;
  intensities.reserve(static_cast<size_t>(nBeams * nFreq));
  for (int f = 0; f < nFreq; ++f)
  {
    for (int beam = 0; beam < nBeams; ++beam)
    {
      int val = static_cast<int>(sensorGain * std::abs(P_Beams[beam][f]));
      intensities.push_back(
        static_cast<uint8_t>(std::min(static_cast<int>(UCHAR_MAX), val)));
    }
  }
  sonar_data.data = intensities;
  raw.image       = sonar_data;
  sonar_image_raw_pub_->publish(raw);

  // ---- Build visual sonar image ----
  cv::Mat Intensity_image = cv::Mat::zeros(cv::Size(nBeams, nFreq), CV_8UC1);

  const float rangeMax         = static_cast<float>(maxDistance);
  const float rangeRes         = ranges[1] - ranges[0];
  const int   nEffectiveRanges = static_cast<int>(std::ceil(rangeMax / rangeRes));
  const unsigned int radius    = static_cast<unsigned int>(Intensity_image.size().height);
  const cv::Point origin(Intensity_image.size().width / 2,
                         Intensity_image.size().height);
  const float binThickness     = 2.0f * std::ceil(
    static_cast<float>(radius) / static_cast<float>(nEffectiveRanges));

  struct BearingEntry { float begin, center, end; };
  std::vector<BearingEntry> angles;
  angles.reserve(nBeams);

  for (int b = 0; b < nBeams; ++b)
  {
    const float center = static_cast<float>(azimuth_angles[b]);
    float begin = 0.0f, end = 0.0f;
    if (b == 0)
    {
      end   = (static_cast<float>(azimuth_angles[b + 1]) + center) / 2.0f;
      begin = 2.0f * center - end;
=======
      this->writeNumber = this->writeNumber + 1;
    }
  }

  // Sonar image ROS msg
  this->sonar_image_raw_msg_.header.frame_id
        = this->frame_name_.c_str();
  this->sonar_image_raw_msg_.header.stamp.sec
        = this->depth_sensor_update_time_.sec;
  this->sonar_image_raw_msg_.header.stamp.nsec
        = this->depth_sensor_update_time_.nsec;
  marine_acoustic_msgs::PingInfo ping_info_msg_;
  ping_info_msg_.frequency = this->sonarFreq;
  ping_info_msg_.sound_speed = this->soundSpeed;
  std::vector<float> azimuth_angles;
  double fl = static_cast<double>(width) / (2.0 * tan(hFOV/2.0));
  for (size_t beam = 0; beam < nBeams; beam ++)
  {
    ping_info_msg_.rx_beamwidths.push_back(static_cast<float>(
      abs(atan2(static_cast<double>(beam) - 1.0 * static_cast<double>(width), fl)
      - atan2(static_cast<double>(beam), fl))));
    ping_info_msg_.tx_beamwidths.push_back(static_cast<float>(vFOV));
    azimuth_angles.push_back(atan2(static_cast<double>(beam) -
                    0.5 * static_cast<double>(width), fl));
  }
  this->sonar_image_raw_msg_.ping_info = ping_info_msg_;

  std::vector<geometry_msgs::Vector3> beam_directions_stack;
  for (size_t beam = 0; beam < nBeams; beam ++)
  {
    geometry_msgs::Vector3 beam_direction;
    beam_direction.x = cos(azimuth_angles[beam]);
    beam_direction.y = sin(azimuth_angles[beam]);
    beam_direction.z = 0.0;
    beam_directions_stack.push_back(beam_direction);
  }
  this->sonar_image_raw_msg_.beam_directions = beam_directions_stack;

  std::vector<float> ranges;
  for (size_t i = 0; i < P_Beams[0].size(); i ++)
    ranges.push_back(rangeVector[i]);
  this->sonar_image_raw_msg_.ranges = ranges;
  marine_acoustic_msgs::SonarImageData sonar_image_data;
  sonar_image_data.is_bigendian = false;
  sonar_image_data.dtype = 0; //DTYPE_UINT8
  sonar_image_data.beam_count = nBeams;

  std::vector<uchar> intensities;
  // FIX 5: The original code declared `int Intensity[nBeams][nFreq]` — a
  // Variable Length Array (VLA).  VLAs are a GCC extension, not standard C++,
  // and become stack-overflow candidates when nBeams*nFreq is large.  Worse,
  // the 2D array was written into on every iteration but never read back; only
  // the local `counts` variable (derived from the same expression) was actually
  // used.  Remove the dead VLA entirely and compute counts inline.
  for (size_t f = 0; f < nFreq; f ++)
  {
    for (size_t beam = 0; beam < nBeams; beam ++)
    {
      // Serialize beams in reverse order to flip the data left to right
      const size_t beam_idx = nBeams - beam - 1;
      int intensity_val = static_cast<int>(
          this->sensorGain * std::abs(P_Beams[beam_idx][f]));
      uchar counts = static_cast<uchar>(std::min(UCHAR_MAX, intensity_val));
      intensities.push_back(counts);
    }
  }
  sonar_image_data.data = intensities;
  this->sonar_image_raw_msg_.image = sonar_image_data;
  this->sonar_image_raw_pub_.publish(this->sonar_image_raw_msg_);

  // Construct visual sonar image for rqt plot in sensor::image msg format
  cv_bridge::CvImage img_bridge;

  // Generate image of CV_8UC1
  cv::Mat Intensity_image = cv::Mat::zeros(cv::Size(nBeams, nFreq), CV_8UC1);

  const float rangeMax = maxDistance;
  const float rangeRes = ranges[1]-ranges[0];
  const int nEffectiveRanges = ceil(rangeMax / rangeRes);
  const unsigned int radius = Intensity_image.size().height;
  const cv::Point origin(Intensity_image.size().width/2,
                         Intensity_image.size().height);
  const float binThickness = 2 * ceil(radius / nEffectiveRanges);

  struct BearingEntry
  {
    float begin, center, end;
    BearingEntry(float b, float c, float e)
      : begin(b), center(c), end(e)
        {;}
  };

  std::vector<BearingEntry> angles;
  angles.reserve(nBeams);

  for ( int b = 0; b < nBeams; ++b )
  {
    const float center = azimuth_angles[b];
    float begin = 0.0, end = 0.0;
    if (b == 0)
    {
      end = (azimuth_angles[b + 1] + center) / 2.0;
      begin = 2 * center - end;
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
    }
    else if (b == nBeams - 1)
    {
      begin = angles[b - 1].end;
<<<<<<< HEAD
      end   = 2.0f * center - begin;
=======
      end = 2 * center - begin;
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
    }
    else
    {
      begin = angles[b - 1].end;
<<<<<<< HEAD
      end   = (static_cast<float>(azimuth_angles[b + 1]) + center) / 2.0f;
    }
    angles.push_back({begin, center, end});
  }

  const float ThetaShift = 1.5f * static_cast<float>(M_PI);
  for (int r = 0; r < static_cast<int>(ranges.size()); ++r)
  {
    if (ranges[r] > rangeMax) continue;
    for (int b = 0; b < nBeams; ++b)
    {
      const float range     = ranges[r];
      const int intensity   = static_cast<int>(
        std::floor(10.0 * std::log(std::abs(P_Beams[nBeams - 1 - b][r]))));
      const float begin_ang = angles[b].begin + ThetaShift;
      const float end_ang   = angles[b].end   + ThetaShift;
      const float rad       = static_cast<float>(radius) * range / rangeMax;
      cv::ellipse(Intensity_image, origin, cv::Size(
        static_cast<int>(rad), static_cast<int>(rad)),
        0.0,
        static_cast<double>(begin_ang) * 180.0 / M_PI,
        static_cast<double>(end_ang)   * 180.0 / M_PI,
        intensity,
        static_cast<int>(binThickness));
    }
  }

  cv::normalize(Intensity_image, Intensity_image,
                -255 + plotScaler / 10.0f * 255.0f, 255,
                cv::NORM_MINMAX);
  cv::Mat Intensity_image_color;
  cv::applyColorMap(Intensity_image, Intensity_image_color, cv::COLORMAP_HOT);

  // Publish visual sonar image
  sonar_image_msg_.header.frame_id = frame_name_;
  sonar_image_msg_.header.stamp    = sensor_update_time_;
  cv_bridge::CvImage img_bridge(sonar_image_msg_.header,
                                sensor_msgs::image_encodings::BGR8,
                                Intensity_image_color);
  img_bridge.toImageMsg(sonar_image_msg_);
  sonar_image_pub_->publish(sonar_image_msg_);

  // Publish normal image
  normal_image_msg_.header.frame_id = frame_name_;
  normal_image_msg_.header.stamp    = sensor_update_time_;
  cv::Mat normal_image8;
  normal_image.convertTo(normal_image8, CV_8UC3, 255.0);
  cv_bridge::CvImage normal_bridge(normal_image_msg_.header,
                                   sensor_msgs::image_encodings::RGB8,
                                   normal_image8);
  normal_bridge.toImageMsg(normal_image_msg_);
  normal_image_pub_->publish(normal_image_msg_);
}

// ------------------------------------------------------------------ //
//  ComputeCorrector
// ------------------------------------------------------------------ //

void NpsGazeboRosMultibeamSonarRay::ComputeCorrector()
{
  double hFOV       = hFOV_;
  double hPixelSize = (width > 1) ? hFOV / (width - 1) : hFOV;

  for (int beam = 0; beam < nBeams; ++beam)
  {
    for (int beam_other = 0; beam_other < nBeams; ++beam_other)
    {
      float azimuthBeamPattern = unnormalized_sinc(
        static_cast<float>(M_PI * 0.884 / hPixelSize
          * std::sin(azimuth_angles[beam] - azimuth_angles[beam_other])));
      beamCorrector[beam][beam_other] = std::abs(azimuthBeamPattern);
      beamCorrectorSum += azimuthBeamPattern * azimuthBeamPattern;
    }
  }
  beamCorrectorSum = std::sqrt(beamCorrectorSum);
}

// ------------------------------------------------------------------ //
//  ComputeNormalImage  (unchanged logic, uses cv::Mat)
// ------------------------------------------------------------------ //

cv::Mat NpsGazeboRosMultibeamSonarRay::ComputeNormalImage(cv::Mat & depth)
{
  cv::Mat_<float> f1 = (cv::Mat_<float>(3, 3) <<
     1,  2,  1,
     0,  0,  0,
    -1, -2, -1) / 8.0f;

  cv::Mat_<float> f2 = (cv::Mat_<float>(3, 3) <<
     1,  0, -1,
     2,  0, -2,
     1,  0, -1) / 8.0f;
=======
      end = (azimuth_angles[b + 1] + center) / 2.0;
    }
    angles.push_back(BearingEntry(begin, center, end));
  }

  const float ThetaShift = 1.5*M_PI;
  for ( int r = 0; r < (int)ranges.size(); ++r )
  {
    if ( ranges[r] > rangeMax ) continue;
    for ( int b = 0; b < nBeams; ++b )
    {
      const float range = ranges[r];
      const int intensity = floor(10.0*log(std::abs(P_Beams[nBeams - 1 - b][r])));
      const float begin = angles[b].begin + ThetaShift,
                  end = angles[b].end + ThetaShift;
      const float rad = static_cast<float>(radius) * range/rangeMax;
      // Assume angles are in image frame x-right, y-down
      cv::ellipse(Intensity_image, origin, cv::Size(rad, rad), 0,
                  begin * 180/M_PI, end * 180/M_PI,
                  intensity, binThickness);
    }
  }

  // Normalise and colorize
  cv::normalize(Intensity_image, Intensity_image,
                -255 + this->plotScaler/10*255, 255, cv::NORM_MINMAX);
  cv::Mat Itensity_image_color;
  cv::applyColorMap(Intensity_image, Itensity_image_color, cv::COLORMAP_HOT);

  // Publish final sonar image
  this->sonar_image_msg_.header.frame_id
        = this->frame_name_;
  this->sonar_image_msg_.header.stamp.sec
        = this->depth_sensor_update_time_.sec;
  this->sonar_image_msg_.header.stamp.nsec
        = this->depth_sensor_update_time_.nsec;
  img_bridge = cv_bridge::CvImage(this->sonar_image_msg_.header,
                                  sensor_msgs::image_encodings::BGR8,
                                  Itensity_image_color);
  // from cv_bridge to sensor_msgs::Image
  img_bridge.toImageMsg(this->sonar_image_msg_);

  this->sonar_image_pub_.publish(this->sonar_image_msg_);

  // ---------------------------------------- End of sonar calculation

  // Still publishing the depth and normal image (just because)
  // Depth image
  this->depth_image_msg_.header.frame_id
        = this->frame_name_;
  this->depth_image_msg_.header.stamp.sec
        = this->depth_sensor_update_time_.sec;
  this->depth_image_msg_.header.stamp.nsec
        = this->depth_sensor_update_time_.nsec;
  img_bridge = cv_bridge::CvImage(this->depth_image_msg_.header,
                                  sensor_msgs::image_encodings::TYPE_32FC1,
                                  depth_image);
  // from cv_bridge to sensor_msgs::Image
  img_bridge.toImageMsg(this->depth_image_msg_);
  this->depth_image_pub_.publish(this->depth_image_msg_);

  // Normal image
  this->normal_image_msg_.header.frame_id
        = this->frame_name_;
  this->normal_image_msg_.header.stamp.sec
        = this->depth_sensor_update_time_.sec;
  this->normal_image_msg_.header.stamp.nsec
        = this->depth_sensor_update_time_.nsec;
  cv::Mat normal_image8;
  normal_image.convertTo(normal_image8, CV_8UC3, 255.0);
  img_bridge = cv_bridge::CvImage(this->normal_image_msg_.header,
                                  sensor_msgs::image_encodings::RGB8,
                                  normal_image8);
  img_bridge.toImageMsg(this->normal_image_msg_);
  // from cv_bridge to sensor_msgs::Image
  this->normal_image_pub_.publish(this->normal_image_msg_);

  this->lock_.unlock();
}


void NpsGazeboRosMultibeamSonar::ComputePointCloud(const float *_src)
{
  this->lock_.lock();

  this->point_cloud_msg_.header.frame_id
        = this->frame_name_;
  this->point_cloud_msg_.header.stamp.sec
        = this->depth_sensor_update_time_.sec;
  this->point_cloud_msg_.header.stamp.nsec
        = this->depth_sensor_update_time_.nsec;
  this->point_cloud_msg_.width = this->width;
  this->point_cloud_msg_.height = this->height;
  this->point_cloud_msg_.row_step
        = this->point_cloud_msg_.point_step * this->width;

  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg_);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(this->height * this->width);

  // resize if point cloud image to camera parameters if required
  this->point_cloud_image_.create(this->height, this->width, CV_32FC1);

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg_, "rgb");
  cv::MatIterator_<float> iter_image = this->point_cloud_image_.begin<float>();

  point_cloud_msg_.is_dense = true;

  float* toCopyFrom = const_cast<float*>(_src);
  int index = 0;

  double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
  double fl = static_cast<double>(this->width) / (2.0 * tan(hfov/2.0));

  for (uint32_t j = 0; j < this->height; j++)
  {
    double elevation;
    if (this->height > 1)
      elevation = atan2(static_cast<double>(j) -
                        0.5 * static_cast<double>(this->height), fl);
    else
      elevation = 0.0;

    this->elevation_angles[j] = static_cast<float>(elevation);

    for (uint32_t i = 0; i < this->width;
         i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb, ++iter_image)
    {
      double azimuth;
      if (this->width > 1)
        azimuth = atan2(static_cast<double>(i) -
                        0.5 * static_cast<double>(this->width), fl);
      else
        azimuth = 0.0;

      double depth = toCopyFrom[index++];

      // in optical frame hardcoded rotation
      // rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame
      *iter_x = depth * tan(azimuth);
      *iter_y = depth * tan(elevation);
      if (depth > this->point_cloud_cutoff_)
      {
        *iter_z = depth;
        *iter_image = sqrt(*iter_x * *iter_x +
                           *iter_y * *iter_y +
                           *iter_z * *iter_z);
      }
      else  // point in the unseeable range
      {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        *iter_image = 0.0;
        point_cloud_msg_.is_dense = false;
      }

      // put image color data for each point
      uint8_t*  image_src = static_cast<uint8_t*>(&(this->image_msg_.data[0]));
      if (this->image_msg_.data.size() == this->height * this->width*3)
      {
        // color
        iter_rgb[0] = image_src[i*3+j*this->width*3+0];
        iter_rgb[1] = image_src[i*3+j*this->width*3+1];
        iter_rgb[2] = image_src[i*3+j*this->width*3+2];
      }
      else if (this->image_msg_.data.size() == this->height * this->width)
      {
        // mono (or bayer?  @todo; fix for bayer)
        iter_rgb[0] = image_src[i+j*this->width];
        iter_rgb[1] = image_src[i+j*this->width];
        iter_rgb[2] = image_src[i+j*this->width];
      }
      else
      {
        // no image
        iter_rgb[0] = 0;
        iter_rgb[1] = 0;
        iter_rgb[2] = 0;
      }
    }
  }
  if (this->point_cloud_connect_count_ > 0)
    this->point_cloud_pub_.publish(this->point_cloud_msg_);

  this->lock_.unlock();
}

/////////////////////////////////////////////////
// Precalculation of corrector sonar calculation
void NpsGazeboRosMultibeamSonar::ComputeCorrector()
{
  double hFOV = this->parentSensor->DepthCamera()->HFOV().Radian();
  double hPixelSize = hFOV / this->width;
  double fl = static_cast<double>(width) / (2.0 * tan(hFOV/2.0));
  // Beam culling correction precalculation
  for (size_t beam = 0; beam < nBeams; beam ++)
  {
    float beam_azimuthAngle = atan2(static_cast<double>(beam) -
                        0.5 * static_cast<double>(width), fl);
    for (size_t beam_other = 0; beam_other < nBeams; beam_other ++)
    {
      float beam_azimuthAngle_other = atan2(static_cast<double>(beam_other) -
                        0.5 * static_cast<double>(width), fl);
      float azimuthBeamPattern =
        unnormalized_sinc(M_PI * 0.884 / hPixelSize
        * sin(beam_azimuthAngle-beam_azimuthAngle_other));
      this->beamCorrector[beam][beam_other] = abs(azimuthBeamPattern);
      this->beamCorrectorSum += pow(azimuthBeamPattern, 2);
    }
  }
  this->beamCorrectorSum = sqrt(this->beamCorrectorSum);
}

/////////////////////////////////////////////////
cv::Mat NpsGazeboRosMultibeamSonar::ComputeNormalImage(cv::Mat& depth)
{
  // filters
  cv::Mat_<float> f1 = (cv::Mat_<float>(3, 3) << 1,  2,  1,
                                                 0,  0,  0,
                                                -1, -2, -1) / 8;

  cv::Mat_<float> f2 = (cv::Mat_<float>(3, 3) << 1, 0, -1,
                                                 2, 0, -2,
                                                 1, 0, -1) / 8;
>>>>>>> bde8874 (Remove unused directories from navigator_auv)

  cv::Mat f1m, f2m;
  cv::flip(f1, f1m, 0);
  cv::flip(f2, f2m, 1);

  cv::Mat n1, n2;
  cv::filter2D(depth, n1, -1, f1m, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(depth, n2, -1, f2m, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::Mat no_readings;
  cv::erode(depth == 0, no_readings, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  n1.setTo(0, no_readings);
  n2.setTo(0, no_readings);

  std::vector<cv::Mat> images(3);
<<<<<<< HEAD
  images[0] = n1;
  images[1] = n2;
  images[2] = (1.0 / focal_length_) * depth;
=======

  // NOTE: with different focal lengths, the expression becomes
  // (-dzx*fy, -dzy*fx, fx*fy)
  images.at(0) = n1;    // for green channel
  images.at(1) = n2;    // for red channel
  // FIX 4 (continued): focal_length_ is now set in Load() so this division
  // is safe.  Previously focal_length_ was 0 -> every blue-channel value
  // was +inf, making all normals degenerate after normalisation.
  images.at(2) = 1.0/this->focal_length_*depth;  // for blue channel
>>>>>>> bde8874 (Remove unused directories from navigator_auv)

  cv::Mat normal_image;
  cv::merge(images, normal_image);

  for (int i = 0; i < normal_image.rows; ++i)
<<<<<<< HEAD
    for (int j = 0; j < normal_image.cols; ++j)
    {
      cv::Vec3f & n = normal_image.at<cv::Vec3f>(i, j);
      n = cv::normalize(n);
    }

  return normal_image;
}

}  // namespace nps_uw_multibeam_sonar

// ------------------------------------------------------------------ //
//  Gazebo Harmonic plugin registration
//  (replaces GZ_REGISTER_SENSOR_PLUGIN)
// ------------------------------------------------------------------ //
GZ_ADD_PLUGIN(
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonarRay,
  gz::sim::System,
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonarRay::ISystemConfigure,
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonarRay::ISystemPostUpdate)
=======
  {
    for (int j = 0; j < normal_image.cols; ++j)
    {
      cv::Vec3f& n = normal_image.at<cv::Vec3f>(i, j);
      n = cv::normalize(n);
      // FIX 8: `float& d = depth.at<float>(i, j);` was declared here but
      // never read or written after the declaration — dead code that caused
      // a compiler warning.  Removed.
    }
  }
  return normal_image;
}


/////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonar::PublishCameraInfo()
{
  ROS_DEBUG_NAMED("depth_camera",
    "publishing default camera info, then depth camera info");
  GazeboRosCameraUtils::PublishCameraInfo();

  if (this->depth_info_connect_count_ > 0)
  {
    common::Time sensor_update_time
          = this->parentSensor_->LastMeasurementTime();

    this->sensor_update_time_ = sensor_update_time;
    if (sensor_update_time
          - this->last_depth_image_camera_info_update_time_
          >= this->update_period_)
    {
      this->PublishCameraInfo(this->depth_image_camera_info_pub_);
      this->last_depth_image_camera_info_update_time_ = sensor_update_time;
    }
  }
}

}  // namespace gazebo
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
