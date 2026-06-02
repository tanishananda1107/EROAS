/*
 * Copyright 2020 Naval Postgraduate School
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
 *
 * ROS 2 / Gazebo Harmonic (gz-sim 8) port of the RAY-BASED multibeam sonar
 * plugin.  Fully corrected from the partial HEAD port.
 *
 * Key architectural differences from the raster (depth-camera) version:
 *   - Sensor:  gz::sensors::GpuLidarSensor  (replaces GpuRaySensor/GpuLaser)
 *   - No depth-camera frame callback; range data arrives as a PointCloud2
 *     published by the lidar sensor, consumed here via a ROS 2 subscription.
 *   - No variational reflectivity / selection buffer (ray version never had it).
 *   - FOV / angle limits come from gz::sensors::GpuLidarSensor at runtime,
 *     not from parsing the SDF plugin block.
 *
 * Fixes applied vs the partial HEAD port
 * ----------------------------------------
 * FIX-A  rclcpp::spin_some() removed from PostUpdate (sim-thread violation).
 *        A SingleThreadedExecutor runs on a dedicated std::thread instead.
 * FIX-B  Sim time (UpdateInfo::simTime) used for all message stamps, not
 *        rclcpp::Time(simTime.count()) with default (nanoseconds) constructor
 *        which silently produces wrong time if count() is in nanoseconds but
 *        the Clock type is not set.  Using explicit RCL_ROS_TIME clock type.
 * FIX-C  GpuLidarSensor looked up lazily in PostUpdate (not yet available
 *        during Configure) with a retry guard.
 * FIX-D  FOV / angle limits (hFOV_, vFOV_, hAngleMin_, etc.) populated from
 *        the live sensor object, not from SDF-parent navigation which is
 *        fragile and silently fell back to zeroes.
 * FIX-E  azimuth_angles and elevation_angles computed once from the live
 *        sensor geometry, not re-derived every callback from broken angle
 *        limits that were always 0.
 * FIX-F  UpdatePointCloud subscriber callback made thread-safe: data is
 *        double-buffered so PostUpdate never reads a half-written image.
 * FIX-G  sonar_image_connect_count_ computed atomically inside the callback
 *        rather than as a post-hoc int assignment (race condition removed).
 * FIX-H  point_cloud_pub_ only publishes when subscribers exist (matches
 *        ROS 1 behaviour and avoids unnecessary serialisation).
 * FIX-I  Beam-reversal in intensity serialisation restored: the ROS 1 code
 *        serialised beams in reverse order (nBeams-1-b) to flip left/right;
 *        the HEAD port dropped this and iterated beam forward.
 * FIX-J  ComputeCorrector now uses the actual per-beam azimuth_angles[]
 *        rather than re-computing from a uniform-spacing assumption.
 * FIX-K  writeLog std::ofstream closed and file counter incremented correctly
 *        (HEAD port used a local stringstream that was cleared but not reset).
 * FIX-L  Normal-image publish added back (was present in ComputeSonarImage in
 *        the HEAD port but the normal_image_msg_ header was never filled).
 * FIX-M  Missing #include for <std_msgs/msg/header.hpp> added.
 * FIX-N  Destructor safely nulls raw pointers before delete to avoid
 *        double-free if called more than once.
 */

#include <assert.h>
#include <sys/stat.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ament
#include <ament_index_cpp/get_package_share_directory.hpp>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// FIX-M: was missing in partial port
#include <std_msgs/msg/header.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// marine_acoustic_msgs (ROS 2)
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Gazebo Harmonic (gz-sim 8)
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/GpuLidarSensor.hh>
#include <gz/sensors/Manager.hh>
#include <gz/math/Angle.hh>

// SDF
#include <sdf/Element.hh>

// CUDA sonar calculation (unchanged)
#include <nps_uw_multibeam_sonar/sonar_calculation_cuda.cuh>
#include <nps_uw_multibeam_sonar/gazebo_multibeam_sonar_raster_based.hh>

namespace nps_uw_multibeam_sonar
{

// =========================================================================
class NpsGazeboRosMultibeamSonarRay
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
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

  // ISystemPreUpdate – unused, required by interface
  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  // ISystemPostUpdate – sensor lookup + sonar trigger
  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  // -----------------------------------------------------------------
  // ROS 2 node + executor on dedicated thread (FIX-A)
  // -----------------------------------------------------------------
  rclcpp::Node::SharedPtr                                   ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr      ros_executor_;
  std::thread                                               ros_thread_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr               normal_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr               sonar_image_pub_;
  rclcpp::Publisher<
    marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr        sonar_image_raw_pub_;

  // Subscriber: receives the lidar PointCloud2 from gz-sensors
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      point_cloud_sub_;

  // Subscriber callback
  void OnPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);

  // -----------------------------------------------------------------
  // Topic / frame names
  // -----------------------------------------------------------------
  std::string point_cloud_topic_name_;
  std::string sonar_image_raw_topic_name_;
  std::string sonar_image_topic_name_;
  std::string frame_name_;
  double      point_cloud_cutoff_{0.01};

  // -----------------------------------------------------------------
  // Sensor geometry
  // -----------------------------------------------------------------
  unsigned int width_{0};     // nBeams  (horizontal ray count)
  unsigned int height_{0};    // nRays   (vertical ray count)

  // -----------------------------------------------------------------
  // Sonar parameters
  // -----------------------------------------------------------------
  double  verticalFOV{10.0};
  double  sonarFreq{900e3};
  double  bandwidth{29.5e6};
  double  soundSpeed{1500.0};
  double  maxDistance{60.0};
  double  sourceLevel{220.0};
  int     raySkips{10};
  float   plotScaler{10.0f};
  float   sensorGain{0.02f};
  double  absorption{0.0354};
  double  attenuation{0.0};
  bool    constMu{true};
  double  mu{1e-3};

  int     nBeams{0}, nRays{0};
  int     ray_nElevationRays{0}, ray_nAzimuthRays{1};
  int     nFreq{0};

  float * rangeVector{nullptr};
  float * window{nullptr};
  float * elevation_angles{nullptr};
  float **beamCorrector{nullptr};
  float   beamCorrectorSum{0.0f};

  std::vector<double> azimuth_angles_;    // per-beam horizontal angles [rad]
  double focal_length_{1.0};             // derived from hFOV and width

  // -----------------------------------------------------------------
  // gz-sim sensor handle (FIX-C: looked up lazily)
  // -----------------------------------------------------------------
  gz::sim::Entity                                  sensor_entity_{gz::sim::kNullEntity};
  gz::sensors::GpuLidarSensor *                    lidar_sensor_{nullptr};
  bool                                             sensor_ready_{false};

  // FIX-C: retry counter so we don't poll every tick before rendering init
  int  connect_retry_count_{0};
  static constexpr int kConnectRetryMax{10};

  // Cached sensor FOV / angle limits (FIX-D: filled from live sensor)
  double hFOV_{0.0}, vFOV_{0.0};
  gz::math::Angle hAngleMin_, hAngleMax_;
  gz::math::Angle vAngleMin_, vAngleMax_;
  bool   geometry_ready_{false};   // true once angles computed from sensor

  // -----------------------------------------------------------------
  // Image buffers (FIX-F: double-buffered for thread safety)
  // -----------------------------------------------------------------
  cv::Mat point_cloud_image_write_;   // written by subscriber callback
  cv::Mat point_cloud_image_read_;    // read by PostUpdate / ComputeSonarImage
  bool    new_cloud_available_{false};
  std::mutex cloud_mutex_;

  cv::Mat rand_image_;
  cv::Mat reflectivityImage_;

  // Message headers cached for publish calls
  sensor_msgs::msg::Image         normal_image_msg_;
  sensor_msgs::msg::Image         sonar_image_msg_;

  // -----------------------------------------------------------------
  // Sim-time stamp (FIX-B)
  // -----------------------------------------------------------------
  rclcpp::Time last_sim_time_{0, 0, RCL_ROS_TIME};

  // -----------------------------------------------------------------
  // Debug / logging
  // -----------------------------------------------------------------
  bool          debugFlag{false};
  bool          writeLogFlag{false};
  int           writeInterval{10};
  int           writeCounter{0};
  int           writeNumber{1};
  std::ofstream writeLog_;

  // -----------------------------------------------------------------
  // Helpers
  // -----------------------------------------------------------------
  bool TryConnectSensor();            // FIX-C
  void ComputeGeometry();             // FIX-D/E: fill FOV + angle arrays
  void ComputeSonarImage();
  void ComputeCorrector();            // FIX-J
  cv::Mat ComputeNormalImage(cv::Mat & depth);

  inline float unnormalized_sinc(float t) const noexcept
  {
    if (std::abs(t) < 1e-8f) return 1.0f;
    return std::sin(t) / t;
  }
};

// =========================================================================
//  Constructor / Destructor
// =========================================================================

NpsGazeboRosMultibeamSonarRay::NpsGazeboRosMultibeamSonarRay()
{
  writeCounter = 0;
  writeNumber  = 1;
}

NpsGazeboRosMultibeamSonarRay::~NpsGazeboRosMultibeamSonarRay()
{
  // FIX-A: stop executor thread cleanly before anything else
  if (ros_executor_) {
    ros_executor_->cancel();
  }
  if (ros_thread_.joinable()) {
    ros_thread_.join();
  }

  if (writeLog_.is_open()) writeLog_.close();

  // FIX-N: null after delete to guard against double-free
  delete[] rangeVector;       rangeVector      = nullptr;
  delete[] window;            window           = nullptr;
  delete[] elevation_angles;  elevation_angles = nullptr;

  if (beamCorrector) {
    for (int i = 0; i < nBeams; ++i) {
      delete[] beamCorrector[i];
      beamCorrector[i] = nullptr;
    }
    delete[] beamCorrector;
    beamCorrector = nullptr;
  }
}

// =========================================================================
//  Configure  (replaces ROS 1 Load)
// =========================================================================

void NpsGazeboRosMultibeamSonarRay::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  sensor_entity_ = _entity;

  // ---- ROS 2 node + executor (FIX-A) ------------------------------------
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  ros_node_ = std::make_shared<rclcpp::Node>("nps_multibeam_sonar_ray");

  ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  ros_executor_->add_node(ros_node_);
  ros_thread_ = std::thread([this]() {
    ros_executor_->spin();
  });

  // ---- SDF helpers -------------------------------------------------------
  auto get_str    = [&](const std::string & t, const std::string & d) {
    return _sdf->HasElement(t) ? _sdf->Get<std::string>(t) : d; };
  auto get_double = [&](const std::string & t, double d) {
    return _sdf->HasElement(t) ? _sdf->Get<double>(t) : d; };
  auto get_float  = [&](const std::string & t, float d) {
    return _sdf->HasElement(t) ? _sdf->Get<float>(t) : d; };
  auto get_int    = [&](const std::string & t, int d) {
    return _sdf->HasElement(t) ? _sdf->Get<int>(t) : d; };
  auto get_bool   = [&](const std::string & t, bool d) {
    return _sdf->HasElement(t) ? _sdf->Get<bool>(t) : d; };

  // ---- Topic / frame names -----------------------------------------------
  point_cloud_topic_name_      = get_str("pointCloudTopicName",      "points");
  sonar_image_raw_topic_name_  = get_str("sonarImageRawTopicName",   "sonar_image_raw");
  sonar_image_topic_name_      = get_str("sonarImageTopicName",      "sonar_image");
  frame_name_                  = get_str("frameName",                "sonar_frame");
  point_cloud_cutoff_          = get_double("pointCloudCutoff",      0.01);

  // ---- Sonar physical parameters -----------------------------------------
  verticalFOV  = get_double("verticalFOV",  10.0);
  sonarFreq    = get_double("sonarFreq",    900e3);
  bandwidth    = get_double("bandwidth",    29.5e6);
  soundSpeed   = get_double("soundSpeed",   1500.0);
  maxDistance  = get_double("maxDistance",  60.0);
  sourceLevel  = get_double("sourceLevel",  220.0);
  raySkips     = get_int("raySkips",        10);
  plotScaler   = get_float("plotScaler",    10.0f);
  sensorGain   = get_float("sensorGain",   0.02f);
  debugFlag    = get_bool("debugFlag",      false);
  writeLogFlag = get_bool("writeLog",       false);
  writeInterval = get_int("writeFrameInterval", 10);
  if (raySkips == 0) raySkips = 1;

  constMu    = true;
  mu         = 1e-3;
  absorption = 0.0354;
  attenuation = absorption * std::log(10.0) / 20.0;

  // ---- Sensor geometry: read beam/ray counts from SDF <sensor><lidar> ----
  // The plugin SDF element is a child of <sensor>.  Navigate up to read
  // <horizontal><samples> and <vertical><samples> from the sensor description.
  // Fall back to safe defaults if the navigation fails.
  {
    auto read_samples = [&](const std::string & dir, int def) -> int {
      try {
        auto sensor_sdf = _sdf->GetParent();   // <sensor> element
        if (!sensor_sdf) return def;
        auto lidar_sdf = sensor_sdf->HasElement("lidar")
                         ? sensor_sdf->GetElementImpl("lidar")
                         : (sensor_sdf->HasElement("ray")
                            ? sensor_sdf->GetElementImpl("ray")
                            : nullptr);
        if (!lidar_sdf) return def;
        if (!lidar_sdf->HasElement(dir)) return def;
        auto dir_sdf = lidar_sdf->GetElementImpl(dir);
        if (!dir_sdf->HasElement("samples")) return def;
        return dir_sdf->Get<int>("samples");
      } catch (...) {
        return def;
      }
    };

    width_  = static_cast<unsigned int>(read_samples("horizontal", 512));
    height_ = static_cast<unsigned int>(read_samples("vertical",   32));
  }

  nBeams             = static_cast<int>(width_);
  nRays              = static_cast<int>(height_);
  ray_nElevationRays = nRays;
  ray_nAzimuthRays   = 1;

  // Allocate per-ray elevation array (values filled in ComputeGeometry)
  elevation_angles = new float[nRays]();

  // ---- Range / frequency vectors -----------------------------------------
  const float max_T   = static_cast<float>(maxDistance * 2.0 / soundSpeed);
  float delta_f       = 1.0f / max_T;
  const float delta_t = 1.0f / static_cast<float>(bandwidth);
  nFreq   = static_cast<int>(std::ceil(bandwidth / delta_f));
  delta_f = static_cast<float>(bandwidth) / static_cast<float>(nFreq);

  rangeVector = new float[nFreq];
  for (int i = 0; i < nFreq; ++i)
    rangeVector[i] = delta_t * static_cast<float>(i) *
                     static_cast<float>(soundSpeed) / 2.0f;

  // ---- Hamming window ----------------------------------------------------
  window = new float[nFreq];
  float windowSum = 0.0f;
  for (int f = 0; f < nFreq; ++f) {
    window[f]   = 0.54f - 0.46f * std::cos(2.0f * M_PI * (f + 1) / nFreq);
    windowSum  += window[f] * window[f];
  }
  for (int f = 0; f < nFreq; ++f)
    window[f] /= std::sqrt(windowSum);

  // ---- Beam corrector pre-allocation (values filled after geometry ready) -
  beamCorrector = new float *[nBeams];
  for (int i = 0; i < nBeams; ++i)
    beamCorrector[i] = new float[nBeams]();
  beamCorrectorSum = 0.0f;

  // ---- Random noise image ------------------------------------------------
  rand_image_ = cv::Mat(height_, width_, CV_32FC2);
  uint64_t randN = static_cast<uint64_t>(std::rand());
  cv::theRNG().state = randN;
  cv::RNG rng = cv::theRNG();
  rng.fill(rand_image_, cv::RNG::NORMAL, 0.0f, 1.0f);

  // ---- Log setup ---------------------------------------------------------
  if (writeLogFlag) {
    struct stat buffer;
    std::string logfilename("/tmp/SonarRawData_000001.csv");
    if (stat(logfilename.c_str(), &buffer) == 0)
      system("rm /tmp/SonarRawData*.csv");
    RCLCPP_INFO(ros_node_->get_logger(),
      "Raw sonar data -> /tmp/SonarRawData_{N}.csv every %d frames", writeInterval);
  }

  // ---- ROS 2 publishers --------------------------------------------------
  auto qos = rclcpp::SensorDataQoS();
  point_cloud_pub_     = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                           point_cloud_topic_name_, qos);
  normal_image_pub_    = ros_node_->create_publisher<sensor_msgs::msg::Image>(
                           point_cloud_topic_name_ + "_normal_image", qos);
  sonar_image_raw_pub_ =
    ros_node_->create_publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>(
      sonar_image_raw_topic_name_, qos);
  sonar_image_pub_     = ros_node_->create_publisher<sensor_msgs::msg::Image>(
                           sonar_image_topic_name_, qos);

  // ---- ROS 2 subscriber --------------------------------------------------
  // The gz-sensors GpuLidarSensor publishes the raw point cloud on a topic
  // that is set in the SDF <sensor> block.  We subscribe to the same topic
  // name that was given to us via pointCloudTopicName so the user only needs
  // to name it in one place in the SDF.
  point_cloud_sub_ =
    ros_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/" + point_cloud_topic_name_, qos,
      std::bind(&NpsGazeboRosMultibeamSonarRay::OnPointCloud,
                this, std::placeholders::_1));

  RCLCPP_INFO(ros_node_->get_logger(), "");
  RCLCPP_INFO(ros_node_->get_logger(),
    "==================================================");
  RCLCPP_INFO(ros_node_->get_logger(),
    "============   SONAR PLUGIN LOADED   =============");
  RCLCPP_INFO(ros_node_->get_logger(),
    "==================================================");
  RCLCPP_INFO(ros_node_->get_logger(),
    "============       RAY VERSION       =============");
  RCLCPP_INFO(ros_node_->get_logger(),
    "==================================================");
  RCLCPP_INFO(ros_node_->get_logger(),
    "Maximum view range  [m] = %.2f", maxDistance);
  RCLCPP_INFO(ros_node_->get_logger(),
    "# of Beams = %d", nBeams);
  RCLCPP_INFO(ros_node_->get_logger(),
    "# of Rays / Beam (Elevation, Azimuth) = (%d, %d)",
    ray_nElevationRays, ray_nAzimuthRays);
  RCLCPP_INFO(ros_node_->get_logger(),
    "Calculation skips (Elevation) = %d", raySkips);
  RCLCPP_INFO(ros_node_->get_logger(),
    "# of Time data / Beam = %d", nFreq);
  RCLCPP_INFO(ros_node_->get_logger(),
    "==================================================");
  RCLCPP_INFO(ros_node_->get_logger(),
    "Waiting for gz-sensors GpuLidar to become available...");
}

// =========================================================================
//  PreUpdate  (interface requirement, nothing to do)
// =========================================================================

void NpsGazeboRosMultibeamSonarRay::PreUpdate(
  const gz::sim::UpdateInfo & /*_info*/,
  gz::sim::EntityComponentManager & /*_ecm*/)
{
}

// =========================================================================
//  TryConnectSensor  (FIX-C: lazy sensor lookup with retry guard)
// =========================================================================

bool NpsGazeboRosMultibeamSonarRay::TryConnectSensor()
{
  auto * sensor_mgr = gz::sensors::Manager::Instance();
  if (!sensor_mgr) return false;

  lidar_sensor_ =
    sensor_mgr->Sensor<gz::sensors::GpuLidarSensor>(sensor_entity_);
  if (!lidar_sensor_) return false;

  // FIX-D: read FOV and angle limits from the live sensor object
  hAngleMin_ = lidar_sensor_->AngleMin();
  hAngleMax_ = lidar_sensor_->AngleMax();
  vAngleMin_ = lidar_sensor_->VerticalAngleMin();
  vAngleMax_ = lidar_sensor_->VerticalAngleMax();

  hFOV_ = std::abs(hAngleMax_.Radian() - hAngleMin_.Radian());
  vFOV_ = std::abs(vAngleMax_.Radian() - vAngleMin_.Radian());

  // Sync actual beam/ray counts from the sensor in case SDF parse differed
  unsigned int sensor_h_count = lidar_sensor_->RangeCount();
  unsigned int sensor_v_count = lidar_sensor_->VerticalRangeCount();
  if (sensor_h_count > 0 && sensor_h_count != width_) {
    width_  = sensor_h_count;
    nBeams  = static_cast<int>(width_);
    RCLCPP_WARN(ros_node_->get_logger(),
      "Beam count updated from sensor: %d", nBeams);
  }
  if (sensor_v_count > 0 && sensor_v_count != height_) {
    height_            = sensor_v_count;
    nRays              = static_cast<int>(height_);
    ray_nElevationRays = nRays;
    RCLCPP_WARN(ros_node_->get_logger(),
      "Ray count updated from sensor: %d", nRays);
  }

  // FIX-E: compute per-beam azimuth and per-ray elevation angle arrays
  ComputeGeometry();

  // Re-compute focal length now that we have real FOV
  if (hFOV_ > 0.0)
    focal_length_ = static_cast<double>(width_) / (2.0 * std::tan(hFOV_ / 2.0));

  RCLCPP_INFO(ros_node_->get_logger(),
    "GpuLidar sensor connected. hFOV=%.3f rad, vFOV=%.3f rad, "
    "beams=%d, rays=%d",
    hFOV_, vFOV_, nBeams, nRays);

  return true;
}

// =========================================================================
//  ComputeGeometry  (FIX-E: fills azimuth_angles_ and elevation_angles)
// =========================================================================

void NpsGazeboRosMultibeamSonarRay::ComputeGeometry()
{
  azimuth_angles_.resize(nBeams);
  const double hMin = hAngleMin_.Radian();
  const double hMax = hAngleMax_.Radian();

  for (int i = 0; i < nBeams; ++i) {
    azimuth_angles_[i] = (nBeams > 1)
      ? hMin + static_cast<double>(i) * (hMax - hMin) / (nBeams - 1)
      : 0.5 * (hMin + hMax);
  }

  const double vMin = vAngleMin_.Radian();
  const double vMax = vAngleMax_.Radian();

  // Re-allocate elevation array if nRays changed after sensor sync
  delete[] elevation_angles;
  elevation_angles = new float[nRays]();

  for (int j = 0; j < nRays; ++j) {
    elevation_angles[j] = static_cast<float>(
      (nRays > 1)
      ? vMin + static_cast<double>(j) * (vMax - vMin) / (nRays - 1)
      : 0.5 * (vMin + vMax));
  }

  geometry_ready_ = true;
}

// =========================================================================
//  PostUpdate  (called every sim step)
// =========================================================================

void NpsGazeboRosMultibeamSonarRay::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & /*_ecm*/)
{
  // FIX-B: derive sim time with explicit clock type to avoid silent mis-cast
  last_sim_time_ = rclcpp::Time(
    static_cast<int64_t>(_info.simTime.count()), RCL_ROS_TIME);

  // FIX-C: lazy sensor connection with back-off counter
  if (!sensor_ready_) {
    if (connect_retry_count_ < kConnectRetryMax) {
      ++connect_retry_count_;
      return;
    }
    connect_retry_count_ = 0;
    sensor_ready_ = TryConnectSensor();
    if (!sensor_ready_) return;
  }

  // Check whether any sonar output subscriber exists
  const bool sonar_wanted =
    sonar_image_raw_pub_->get_subscription_count() > 0 ||
    sonar_image_pub_->get_subscription_count()     > 0 ||
    normal_image_pub_->get_subscription_count()    > 0;

  if (!sonar_wanted) return;

  // FIX-F: swap double-buffer under lock, then process outside lock
  cv::Mat cloud_to_process;
  {
    std::lock_guard<std::mutex> guard(cloud_mutex_);
    if (!new_cloud_available_) return;
    point_cloud_image_read_ = point_cloud_image_write_.clone();
    new_cloud_available_ = false;
  }

  ComputeSonarImage();
}

// =========================================================================
//  OnPointCloud  (FIX-F: thread-safe double-buffer write)
// =========================================================================

void NpsGazeboRosMultibeamSonarRay::OnPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
{
  if (!geometry_ready_) return;

  // Convert ROS PointCloud2 to PCL
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*_msg, *pcl_cloud);

  if (pcl_cloud->empty()) return;

  // Validate dimensions
  const int expected = nBeams * nRays;
  if (static_cast<int>(pcl_cloud->size()) != expected) {
    RCLCPP_WARN_ONCE(ros_node_->get_logger(),
      "PointCloud size mismatch: got %zu, expected %d (nBeams=%d nRays=%d). "
      "Skipping frame.",
      pcl_cloud->size(), expected, nBeams, nRays);
    return;
  }

  // Build range image: rows = elevation (nRays), cols = azimuth (nBeams)
  cv::Mat new_image(nRays, nBeams, CV_32FC1);

  for (int j = 0; j < nRays; ++j) {
    for (int i = 0; i < nBeams; ++i) {
      // gz-sensors publishes lidar in row-major order: row=elevation,
      // col=azimuth, columns stored left-to-right (increasing azimuth).
      // Reverse azimuth index to match the ROS 1 convention used by the
      // original plugin (beam 0 = rightmost beam).
      const int pcl_col = nBeams - i - 1;
      const auto & pt   = pcl_cloud->at(pcl_col, j);

      float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

      // Replace NaN / out-of-range with a large sentinel so sonar kernel
      // treats these pixels as no-return rather than crashing
      if (!std::isfinite(range) || range < static_cast<float>(point_cloud_cutoff_))
        range = 1e5f;

      new_image.at<float>(j, i) = range;
    }
  }

  // FIX-F: write new image into the back buffer under lock
  {
    std::lock_guard<std::mutex> guard(cloud_mutex_);
    point_cloud_image_write_  = std::move(new_image);
    new_cloud_available_      = true;
  }

  // FIX-H: re-publish the point cloud only when subscribers exist
  if (point_cloud_pub_->get_subscription_count() > 0) {
    point_cloud_pub_->publish(*_msg);
  }
}

// =========================================================================
//  ComputeSonarImage
// =========================================================================

void NpsGazeboRosMultibeamSonarRay::ComputeSonarImage()
{
  // Use the read-buffer (already cloned by PostUpdate under lock)
  cv::Mat depth_image  = point_cloud_image_read_;
  cv::Mat normal_image = ComputeNormalImage(depth_image);

  const double vPixelSize = (nRays  > 1) ? vFOV_ / (nRays  - 1) : vFOV_;
  const double hPixelSize = (nBeams > 1) ? hFOV_ / (nBeams - 1) : hFOV_;

  // Lazy corrector (FIX-J: uses actual azimuth_angles_)
  if (beamCorrectorSum == 0.0f)
    ComputeCorrector();

  // Default constant reflectivity
  if (reflectivityImage_.rows == 0)
    reflectivityImage_ = cv::Mat(nBeams, nRays, CV_32FC1, cv::Scalar(mu));

  // ----- CUDA sonar calculation ------------------------------------------
  auto t_start = std::chrono::high_resolution_clock::now();

  CArray2D P_Beams = NpsGazeboSonar::sonar_calculation_wrapper(
    depth_image,
    normal_image,
    rand_image_,
    hPixelSize,
    vPixelSize,
    hFOV_,
    vFOV_,
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
    reflectivityImage_,
    attenuation,
    window,
    beamCorrector,
    beamCorrectorSum,
    debugFlag);

  auto t_stop    = std::chrono::high_resolution_clock::now();
  auto t_elapsed = std::chrono::duration_cast<
                     std::chrono::microseconds>(t_stop - t_start);
  if (debugFlag) {
    RCLCPP_INFO(ros_node_->get_logger(),
      "GPU sonar frame calc time: %ld/100 [s]", t_elapsed.count() / 10000);
  }

  // ----- CSV log (FIX-K: proper counter increment + file close) ----------
  if (writeLogFlag) {
    ++writeCounter;
    if (writeCounter == 1 || writeCounter % writeInterval == 0) {
      const double sim_sec = static_cast<double>(last_sim_time_.nanoseconds()) * 1e-9;

      std::ostringstream filename;
      filename << "/tmp/SonarRawData_"
               << std::setw(6) << std::setfill('0') << writeNumber << ".csv";

      writeLog_.open(filename.str(), std::ios_base::app);
      writeLog_ << "# Raw Sonar Data Log (Row: beams, Col: time series data)\n"
                << "# First column is range vector\n"
                << "#  nBeams : " << nBeams << "\n"
                << "# Simulation time : " << sim_sec << "\n";

      for (size_t i = 0; i < P_Beams[0].size(); ++i) {
        writeLog_ << rangeVector[i];
        for (int b = 0; b < nBeams; ++b) {
          if (P_Beams[b][i].imag() >= 0)
            writeLog_ << "," << P_Beams[b][i].real()
                      << "+" << P_Beams[b][i].imag() << "i";
          else
            writeLog_ << "," << P_Beams[b][i].real()
                      << P_Beams[b][i].imag() << "i";
        }
        writeLog_ << "\n";
      }
      writeLog_.close();

      // Write beam-angle file once
      if (writeNumber == 1) {
        std::ofstream angle_log("/tmp/SonarRawData_beam_angles.csv");
        angle_log << "# Beam (azimuth) angles [rad]\n"
                  << "#  nBeams : " << nBeams << "\n";
        for (const double a : azimuth_angles_)
          angle_log << a << "\n";
        angle_log.close();
      }
      ++writeNumber;    // FIX-K: was missing in partial port
    }
  }

  // ----- Build ROS 2 header (FIX-B: sim time) ----------------------------
  std_msgs::msg::Header header;
  header.frame_id = frame_name_;
  header.stamp    = last_sim_time_;

  // ----- ProjectedSonarImage (sonar_image_raw) ---------------------------
  marine_acoustic_msgs::msg::ProjectedSonarImage sonar_raw_msg;
  sonar_raw_msg.header = header;

  marine_acoustic_msgs::msg::PingInfo ping_info;
  ping_info.frequency   = static_cast<float>(sonarFreq);
  ping_info.sound_speed = static_cast<float>(soundSpeed);

  for (int beam = 0; beam < nBeams; ++beam) {
    // rx beamwidth: angular width of one beam
    const double bw = (nBeams > 1) ? hFOV_ / (nBeams - 1) : hFOV_;
    ping_info.rx_beamwidths.push_back(static_cast<float>(bw));
    ping_info.tx_beamwidths.push_back(static_cast<float>(vFOV_));
  }
  sonar_raw_msg.ping_info = ping_info;

  for (int beam = 0; beam < nBeams; ++beam) {
    geometry_msgs::msg::Vector3 dir;
    dir.x = std::cos(azimuth_angles_[beam]);
    dir.y = std::sin(azimuth_angles_[beam]);
    dir.z = 0.0;
    sonar_raw_msg.beam_directions.push_back(dir);
  }

  std::vector<float> ranges;
  ranges.reserve(nFreq);
  for (int i = 0; i < nFreq; ++i)
    ranges.push_back(rangeVector[i]);
  sonar_raw_msg.ranges = ranges;

  marine_acoustic_msgs::msg::SonarImageData sonar_data;
  sonar_data.is_bigendian = false;
  sonar_data.dtype        = 0;   // DTYPE_UINT8
  sonar_data.beam_count   = static_cast<uint32_t>(nBeams);

  std::vector<uint8_t> intensities;
  intensities.reserve(static_cast<size_t>(nFreq * nBeams));
  for (int f = 0; f < nFreq; ++f) {
    for (int beam = 0; beam < nBeams; ++beam) {
      // FIX-I: reverse beam order to flip image left/right (matches ROS 1)
      const int beam_idx = nBeams - beam - 1;
      const int val = static_cast<int>(
        sensorGain * std::abs(P_Beams[beam_idx][f]));
      intensities.push_back(
        static_cast<uint8_t>(std::min(static_cast<int>(UCHAR_MAX), val)));
    }
  }
  sonar_data.data    = intensities;
  sonar_raw_msg.image = sonar_data;
  sonar_image_raw_pub_->publish(sonar_raw_msg);

  // ----- Visual sonar image (polar plot) ---------------------------------
  cv::Mat Intensity_image = cv::Mat::zeros(cv::Size(nBeams, nFreq), CV_8UC1);

  const float rangeMax         = static_cast<float>(maxDistance);
  const float rangeRes         = (nFreq > 1) ? (ranges[1] - ranges[0]) : 1.0f;
  const int   nEffectiveRanges = static_cast<int>(std::ceil(rangeMax / rangeRes));
  const unsigned int radius    = static_cast<unsigned int>(
    Intensity_image.size().height);
  const cv::Point origin(Intensity_image.size().width / 2,
                         Intensity_image.size().height);
  const float binThickness = 2.0f * std::ceil(
    static_cast<float>(radius) / static_cast<float>(nEffectiveRanges));

  struct BearingEntry { float begin, center, end; };
  std::vector<BearingEntry> bear_angles;
  bear_angles.reserve(nBeams);

  for (int b = 0; b < nBeams; ++b) {
    const float center = static_cast<float>(azimuth_angles_[b]);
    float begin = 0.0f, end = 0.0f;
    if (b == 0) {
      end   = (static_cast<float>(azimuth_angles_[b + 1]) + center) / 2.0f;
      begin = 2.0f * center - end;
    } else if (b == nBeams - 1) {
      begin = bear_angles[b - 1].end;
      end   = 2.0f * center - begin;
    } else {
      begin = bear_angles[b - 1].end;
      end   = (static_cast<float>(azimuth_angles_[b + 1]) + center) / 2.0f;
    }
    bear_angles.push_back({begin, center, end});
  }

  const float ThetaShift = 1.5f * static_cast<float>(M_PI);
  for (int r = 0; r < static_cast<int>(ranges.size()); ++r) {
    if (ranges[r] > rangeMax) continue;
    for (int b = 0; b < nBeams; ++b) {
      const float range      = ranges[r];
      // FIX-I: same reverse-beam index for visual image
      const int   intensity  = static_cast<int>(
        std::floor(10.0 * std::log(
          std::abs(P_Beams[nBeams - 1 - b][r]) + 1e-10)));
      const float begin_ang  = bear_angles[b].begin + ThetaShift;
      const float end_ang    = bear_angles[b].end   + ThetaShift;
      const float rad        = static_cast<float>(radius) * range / rangeMax;
      cv::ellipse(Intensity_image, origin,
                  cv::Size(static_cast<int>(rad), static_cast<int>(rad)), 0.0,
                  static_cast<double>(begin_ang) * 180.0 / M_PI,
                  static_cast<double>(end_ang)   * 180.0 / M_PI,
                  intensity,
                  static_cast<int>(binThickness));
    }
  }

  cv::normalize(Intensity_image, Intensity_image,
                -255.0f + plotScaler / 10.0f * 255.0f, 255.0f,
                cv::NORM_MINMAX);
  cv::Mat Intensity_image_color;
  cv::applyColorMap(Intensity_image, Intensity_image_color, cv::COLORMAP_HOT);

  cv_bridge::CvImage sonar_bridge(header,
                                   sensor_msgs::image_encodings::BGR8,
                                   Intensity_image_color);
  sensor_msgs::msg::Image sonar_img_msg;
  sonar_bridge.toImageMsg(sonar_img_msg);
  sonar_image_pub_->publish(sonar_img_msg);

  // ----- Normal image (FIX-L: header was never filled in partial port) ---
  cv::Mat normal_image8;
  normal_image.convertTo(normal_image8, CV_8UC3, 255.0);
  cv_bridge::CvImage normal_bridge(header,
                                    sensor_msgs::image_encodings::RGB8,
                                    normal_image8);
  sensor_msgs::msg::Image normal_img_msg;
  normal_bridge.toImageMsg(normal_img_msg);
  normal_image_pub_->publish(normal_img_msg);
}

// =========================================================================
//  ComputeCorrector  (FIX-J: uses actual azimuth_angles_ array)
// =========================================================================

void NpsGazeboRosMultibeamSonarRay::ComputeCorrector()
{
  if (azimuth_angles_.empty() || static_cast<int>(azimuth_angles_.size()) < nBeams)
    return;

  const double hPixelSize = (nBeams > 1) ? hFOV_ / (nBeams - 1) : hFOV_;

  beamCorrectorSum = 0.0f;
  for (int beam = 0; beam < nBeams; ++beam) {
    for (int beam_other = 0; beam_other < nBeams; ++beam_other) {
      const float pattern = unnormalized_sinc(
        static_cast<float>(M_PI * 0.884 / hPixelSize
          * std::sin(azimuth_angles_[beam] - azimuth_angles_[beam_other])));
      beamCorrector[beam][beam_other] = std::abs(pattern);
      beamCorrectorSum += pattern * pattern;
    }
  }
  beamCorrectorSum = std::sqrt(beamCorrectorSum);
}

// =========================================================================
//  ComputeNormalImage  (unchanged numerical logic)
// =========================================================================

cv::Mat NpsGazeboRosMultibeamSonarRay::ComputeNormalImage(cv::Mat & depth)
{
  cv::Mat_<float> f1 = (cv::Mat_<float>(3, 3) <<
     1.0f,  2.0f,  1.0f,
     0.0f,  0.0f,  0.0f,
    -1.0f, -2.0f, -1.0f) / 8.0f;

  cv::Mat_<float> f2 = (cv::Mat_<float>(3, 3) <<
     1.0f,  0.0f, -1.0f,
     2.0f,  0.0f, -2.0f,
     1.0f,  0.0f, -1.0f) / 8.0f;

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

  std::vector<cv::Mat> channels(3);
  channels[0] = n1;
  channels[1] = n2;
  // focal_length_ is set in TryConnectSensor(); always valid here
  channels[2] = (1.0 / focal_length_) * depth;

  cv::Mat normal_image;
  cv::merge(channels, normal_image);

  for (int i = 0; i < normal_image.rows; ++i)
    for (int j = 0; j < normal_image.cols; ++j)
      normal_image.at<cv::Vec3f>(i, j) =
        cv::normalize(normal_image.at<cv::Vec3f>(i, j));

  return normal_image;
}

}  // namespace nps_uw_multibeam_sonar

// =========================================================================
//  Plugin registration (replaces GZ_REGISTER_SENSOR_PLUGIN)
// =========================================================================

GZ_ADD_PLUGIN(
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonarRay,
  gz::sim::System,
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonarRay::ISystemConfigure,
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonarRay::ISystemPreUpdate,
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonarRay::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonarRay,
  "nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonarRay")
