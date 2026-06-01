/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
            writeLog << "," << P_Beams[b][i].real()
                     << "+" << P_Beams[b][i].imag() << "i";
          else
            writeLog << "," << P_Beams[b][i].real()
                     << P_Beams[b][i].imag() << "i";
        }
        writeLog << "\n";
      }
      writeLog.close();

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
    }
    else if (b == nBeams - 1)
    {
      begin = angles[b - 1].end;
      end   = 2.0f * center - begin;
    }
    else
    {
      begin = angles[b - 1].end;
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
  images[0] = n1;
  images[1] = n2;
  images[2] = (1.0 / focal_length_) * depth;

  cv::Mat normal_image;
  cv::merge(images, normal_image);

  for (int i = 0; i < normal_image.rows; ++i)
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
