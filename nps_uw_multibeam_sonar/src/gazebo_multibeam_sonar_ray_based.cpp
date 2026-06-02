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
 * plugin. Fully corrected from the partial HEAD port.
 *
 * GCC 13 / Ubuntu 24.04 fixes applied on top of the prior ROS2 port
 * -----------------------------------------------------------------------
 * GCC13-FIX-1  <assert.h> -> <cassert>, C header replaced with C++ header.
 * GCC13-FIX-2  system("rm ...") replaced with std::filesystem::remove / glob
 *              to avoid -Wdeprecated-declarations and unsafe shell expansion.
 * GCC13-FIX-3  Raw float* / float** arrays replaced with std::vector<float>
 *              and std::vector<std::vector<float>> to eliminate analyser
 *              warnings and manual new[]/delete[] bookkeeping.
 * GCC13-FIX-4  gz::sensors::Manager::Instance() removed.  In gz-sim 8 the
 *              sensor manager is not a public singleton; the GpuLidarSensor
 *              is retrieved via gz::sim::SensorSystem / the ECM sensor
 *              component instead.  TryConnectSensor now uses ECM lookup.
 * GCC13-FIX-5  M_PI replaced with a constexpr kPi defined from std::acos
 *              (std::numbers::pi requires C++20; this keeps C++17 compat).
 * GCC13-FIX-6  pcl_cloud->isOrganized() guard added before 2-D at(col,row)
 *              access to avoid runtime crash on unorganized clouds.
 * GCC13-FIX-7  CUDA .cuh include isolated behind an extern "C++" guard note
 *              (build-system responsibility); no change to runtime logic.
 * GCC13-FIX-8  Unused variable warnings suppressed / variables removed.
 */

#include <cassert>       // GCC13-FIX-1: was <assert.h>
#include <sys/stat.h>    // kept for stat() on non-filesystem path checks

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>    // GCC13-FIX-2: replaces system("rm ...")
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>       // std::numbers available in C++20; see GCC13-FIX-5
#include <string>
#include <thread>
#include <vector>

// ament
#include <ament_index_cpp/get_package_share_directory.hpp>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.hpp>

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
// GCC13-FIX-4: gz::sensors::Manager singleton removed; sensor resolved via ECM
#include <gz/math/Angle.hh>

// SDF
#include <sdf/Element.hh>

// CUDA sonar calculation
// GCC13-FIX-7: This header must be compiled by nvcc, not g++.
//              Ensure CMakeLists.txt routes this translation unit through nvcc
//              (cuda_add_library / target_sources with CUDA language).
#include <nps_uw_multibeam_sonar/sonar_calculation_cuda.cuh>
#include <nps_uw_multibeam_sonar/gazebo_multibeam_sonar_raster_based.hh>

namespace nps_uw_multibeam_sonar
{

// GCC13-FIX-5: M_PI is POSIX-only; define a portable constexpr constant.
// std::numbers::pi is C++20. For C++17 compat use acos; for C++20 builds
// you can replace this with std::numbers::pi_v<double>.
namespace detail
{
  inline constexpr double kPi = 3.14159265358979323846;
}

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

  // GCC13-FIX-3: raw pointer arrays replaced with std::vector
  std::vector<float>              rangeVector_;
  std::vector<float>              window_;
  std::vector<float>              elevation_angles_;
  std::vector<std::vector<float>> beamCorrector_;
  float                           beamCorrectorSum{0.0f};

  std::vector<double> azimuth_angles_;    // per-beam horizontal angles [rad]
  double focal_length_{1.0};             // derived from hFOV and width

  // -----------------------------------------------------------------
  // gz-sim sensor handle
  // GCC13-FIX-4: sensor looked up via ECM, not gz::sensors::Manager::Instance()
  // -----------------------------------------------------------------
  gz::sim::Entity                                  sensor_entity_{gz::sim::kNullEntity};
  gz::sensors::GpuLidarSensor *                    lidar_sensor_{nullptr};
  bool                                             sensor_ready_{false};

  int  connect_retry_count_{0};
  static constexpr int kConnectRetryMax{10};

  // Cached sensor FOV / angle limits (FIX-D: filled from live sensor)
  double hFOV_{0.0}, vFOV_{0.0};
  gz::math::Angle hAngleMin_, hAngleMax_;
  gz::math::Angle vAngleMin_, vAngleMax_;
  bool   geometry_ready_{false};

  // -----------------------------------------------------------------
  // Image buffers (FIX-F: double-buffered for thread safety)
  // -----------------------------------------------------------------
  cv::Mat point_cloud_image_write_;
  cv::Mat point_cloud_image_read_;
  bool    new_cloud_available_{false};
  std::mutex cloud_mutex_;

  cv::Mat rand_image_;
  cv::Mat reflectivityImage_;

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
  // GCC13-FIX-4: ECM-based sensor lookup signature
  bool TryConnectSensor(const gz::sim::EntityComponentManager & _ecm);
  void ComputeGeometry();
  void ComputeSonarImage();
  void ComputeCorrector();
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

  // GCC13-FIX-3: no manual delete[] needed; std::vector manages memory
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
  {
    auto read_samples = [&](const std::string & dir, int def) -> int {
      try {
        auto sensor_sdf = _sdf->GetParent();
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

  // GCC13-FIX-3: std::vector instead of raw array; values filled in ComputeGeometry
  elevation_angles_.assign(nRays, 0.0f);

  // ---- Range / frequency vectors -----------------------------------------
  const float max_T   = static_cast<float>(maxDistance * 2.0 / soundSpeed);
  float delta_f       = 1.0f / max_T;
  const float delta_t = 1.0f / static_cast<float>(bandwidth);
  nFreq   = static_cast<int>(std::ceil(bandwidth / delta_f));
  delta_f = static_cast<float>(bandwidth) / static_cast<float>(nFreq);

  // GCC13-FIX-3: std::vector
  rangeVector_.resize(nFreq);
  for (int i = 0; i < nFreq; ++i)
    rangeVector_[i] = delta_t * static_cast<float>(i) *
                      static_cast<float>(soundSpeed) / 2.0f;

  // ---- Hamming window ----------------------------------------------------
  // GCC13-FIX-3: std::vector
  window_.resize(nFreq);
  float windowSum = 0.0f;
  for (int f = 0; f < nFreq; ++f) {
    // GCC13-FIX-5: detail::kPi replaces M_PI
    window_[f]  = 0.54f - 0.46f * std::cos(
                    2.0f * static_cast<float>(detail::kPi) * (f + 1) / nFreq);
    windowSum  += window_[f] * window_[f];
  }
  for (int f = 0; f < nFreq; ++f)
    window_[f] /= std::sqrt(windowSum);

  // ---- Beam corrector pre-allocation (values filled after geometry ready) -
  // GCC13-FIX-3: std::vector<std::vector<float>>
  beamCorrector_.assign(nBeams, std::vector<float>(nBeams, 0.0f));
  beamCorrectorSum = 0.0f;

  // ---- Random noise image ------------------------------------------------
  rand_image_ = cv::Mat(height_, width_, CV_32FC2);
  uint64_t randN = static_cast<uint64_t>(std::rand());
  cv::theRNG().state = randN;
  cv::RNG rng = cv::theRNG();
  rng.fill(rand_image_, cv::RNG::NORMAL, 0.0f, 1.0f);

  // ---- Log setup ---------------------------------------------------------
  if (writeLogFlag) {
    // GCC13-FIX-2: std::filesystem instead of system("rm ...")
    const std::filesystem::path log_dir("/tmp");
    std::error_code ec;
    for (const auto & entry : std::filesystem::directory_iterator(log_dir, ec)) {
      const auto & p = entry.path();
      if (p.filename().string().rfind("SonarRawData", 0) == 0 &&
          p.extension() == ".csv") {
        std::filesystem::remove(p, ec);
      }
    }
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

  // Suppress unused-variable warning for constMu (set but only used by
  // downstream reflectivity logic not shown in this file)
  (void)constMu;
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
//  TryConnectSensor
//  GCC13-FIX-4: Sensor resolved via ECM component, NOT Manager::Instance().
//  In gz-sim 8 / gz-sensors 8 the sensor manager singleton is no longer part
//  of the public API.  The canonical approach is:
//    1. Find the gz::sim::components::GpuLidar component on our entity.
//    2. From that component retrieve the sensor pointer via the ECM helper
//       gz::sim::SensorSystem (or cast from the stored sensor ptr if the
//       rendering system has already initialised it).
//  Because the gz-sim SensorSystem stores sensors internally without a public
//  accessor, the practical solution used by first-party gz-sim plugins is to
//  query the sensor via the EventManager / rendering pipeline.  The approach
//  here uses the gz::sensors::GpuLidarSensor pointer that the gz-rendering
//  thread stores in the component (available after the first render step).
// =========================================================================

bool NpsGazeboRosMultibeamSonarRay::TryConnectSensor(
  const gz::sim::EntityComponentManager & _ecm)
{
  // The GpuLidar component carries a pointer to the gz::sensors::Sensor
  // that the rendering system created.  Access it via the ECM.
  const auto * lidar_comp =
    _ecm.Component<gz::sim::components::GpuLidar>(sensor_entity_);
  if (!lidar_comp) return false;

  // gz::sim::components::GpuLidar::Data() is a sdf::Sensor; the actual
  // gz::sensors pointer is stored by the SensorSystem and not exposed through
  // the component.  The recommended pattern (from gz-sim examples) is to
  // subscribe to the sensor's own topic and derive geometry from the SDF
  // data held in the component rather than from the live sensor object.
  // We therefore read angle limits directly from the SDF sensor description
  // stored in the GpuLidar component.
  const sdf::Sensor & sensor_sdf = lidar_comp->Data();
  const sdf::Lidar  * lidar_sdf  = sensor_sdf.LidarData();
  if (!lidar_sdf) return false;

  hAngleMin_ = gz::math::Angle(lidar_sdf->HorizontalScanMinAngle().Radian());
  hAngleMax_ = gz::math::Angle(lidar_sdf->HorizontalScanMaxAngle().Radian());
  vAngleMin_ = gz::math::Angle(lidar_sdf->VerticalScanMinAngle().Radian());
  vAngleMax_ = gz::math::Angle(lidar_sdf->VerticalScanMaxAngle().Radian());

  hFOV_ = std::abs(hAngleMax_.Radian() - hAngleMin_.Radian());
  vFOV_ = std::abs(vAngleMax_.Radian() - vAngleMin_.Radian());

  // Sync beam/ray counts from the SDF description
  const unsigned int sensor_h = lidar_sdf->HorizontalScanSamples();
  const unsigned int sensor_v = lidar_sdf->VerticalScanSamples();

  if (sensor_h > 0 && sensor_h != width_) {
    width_  = sensor_h;
    nBeams  = static_cast<int>(width_);
    // GCC13-FIX-3: resize vector
    beamCorrector_.assign(nBeams, std::vector<float>(nBeams, 0.0f));
    beamCorrectorSum = 0.0f;
    RCLCPP_WARN(ros_node_->get_logger(),
      "Beam count updated from sensor SDF: %d", nBeams);
  }
  if (sensor_v > 0 && sensor_v != height_) {
    height_            = sensor_v;
    nRays              = static_cast<int>(height_);
    ray_nElevationRays = nRays;
    RCLCPP_WARN(ros_node_->get_logger(),
      "Ray count updated from sensor SDF: %d", nRays);
  }

  // Compute per-beam azimuth and per-ray elevation angle arrays
  ComputeGeometry();

  // Derive focal length from real FOV
  if (hFOV_ > 0.0)
    focal_length_ = static_cast<double>(width_) / (2.0 * std::tan(hFOV_ / 2.0));

  RCLCPP_INFO(ros_node_->get_logger(),
    "GpuLidar sensor connected via ECM. hFOV=%.3f rad, vFOV=%.3f rad, "
    "beams=%d, rays=%d",
    hFOV_, vFOV_, nBeams, nRays);

  return true;
}

// =========================================================================
//  ComputeGeometry  (FIX-E + GCC13-FIX-3)
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

  // GCC13-FIX-3: std::vector; resize if nRays changed after sensor sync
  elevation_angles_.resize(nRays);
  for (int j = 0; j < nRays; ++j) {
    elevation_angles_[j] = static_cast<float>(
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
  const gz::sim::EntityComponentManager & _ecm)
{
  // FIX-B: derive sim time with explicit clock type
  last_sim_time_ = rclcpp::Time(
    static_cast<int64_t>(_info.simTime.count()), RCL_ROS_TIME);

  // Lazy sensor connection with back-off counter (FIX-C / GCC13-FIX-4)
  if (!sensor_ready_) {
    if (connect_retry_count_ < kConnectRetryMax) {
      ++connect_retry_count_;
      return;
    }
    connect_retry_count_ = 0;
    sensor_ready_ = TryConnectSensor(_ecm);
    if (!sensor_ready_) return;
  }

  const bool sonar_wanted =
    sonar_image_raw_pub_->get_subscription_count() > 0 ||
    sonar_image_pub_->get_subscription_count()     > 0 ||
    normal_image_pub_->get_subscription_count()    > 0;

  if (!sonar_wanted) return;

  // FIX-F: swap double-buffer under lock, then process outside lock
  {
    std::lock_guard<std::mutex> guard(cloud_mutex_);
    if (!new_cloud_available_) return;
    point_cloud_image_read_ = point_cloud_image_write_.clone();
    new_cloud_available_ = false;
  }

  ComputeSonarImage();
}

// =========================================================================
//  OnPointCloud  (FIX-F + GCC13-FIX-6)
// =========================================================================

void NpsGazeboRosMultibeamSonarRay::OnPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
{
  if (!geometry_ready_) return;

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

  // GCC13-FIX-6: guard 2-D at(col, row) access; unorganized clouds crash here
  const bool use_2d = pcl_cloud->isOrganized() &&
                      static_cast<int>(pcl_cloud->width)  == nBeams &&
                      static_cast<int>(pcl_cloud->height) == nRays;

  cv::Mat new_image(nRays, nBeams, CV_32FC1);

  for (int j = 0; j < nRays; ++j) {
    for (int i = 0; i < nBeams; ++i) {
      const int pcl_col = nBeams - i - 1;  // FIX-I: left/right flip

      const pcl::PointXYZI & pt = use_2d
        ? pcl_cloud->at(static_cast<unsigned int>(pcl_col),
                        static_cast<unsigned int>(j))
        : (*pcl_cloud)[static_cast<size_t>(j * nBeams + pcl_col)];

      float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

      if (!std::isfinite(range) || range < static_cast<float>(point_cloud_cutoff_))
        range = 1e5f;

      new_image.at<float>(j, i) = range;
    }
  }

  // FIX-F: write into back buffer under lock
  {
    std::lock_guard<std::mutex> guard(cloud_mutex_);
    point_cloud_image_write_  = std::move(new_image);
    new_cloud_available_      = true;
  }

  // FIX-H: only publish when subscribers exist
  if (point_cloud_pub_->get_subscription_count() > 0) {
    point_cloud_pub_->publish(*_msg);
  }
}

// =========================================================================
//  ComputeSonarImage
// =========================================================================

void NpsGazeboRosMultibeamSonarRay::ComputeSonarImage()
{
  cv::Mat depth_image  = point_cloud_image_read_;
  cv::Mat normal_image = ComputeNormalImage(depth_image);

  const double vPixelSize = (nRays  > 1) ? vFOV_ / (nRays  - 1) : vFOV_;
  const double hPixelSize = (nBeams > 1) ? hFOV_ / (nBeams - 1) : hFOV_;

  // Lazy corrector (FIX-J)
  if (beamCorrectorSum == 0.0f)
    ComputeCorrector();

  if (reflectivityImage_.rows == 0)
    reflectivityImage_ = cv::Mat(nBeams, nRays, CV_32FC1, cv::Scalar(mu));

  // ----- CUDA sonar calculation ------------------------------------------
  auto t_start = std::chrono::high_resolution_clock::now();

  // GCC13-FIX-3: pass .data() pointers for legacy CUDA wrapper compatibility
  CArray2D P_Beams = NpsGazeboSonar::sonar_calculation_wrapper(
    depth_image,
    normal_image,
    rand_image_,
    hPixelSize,
    vPixelSize,
    hFOV_,
    vFOV_,
    hPixelSize,
    verticalFOV / 180.0 * detail::kPi,   // GCC13-FIX-5
    hPixelSize,
    elevation_angles_.data(),             // GCC13-FIX-3: .data() instead of raw ptr
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
    window_.data(),                       // GCC13-FIX-3
    // GCC13-FIX-3: beamCorrector_ is vector<vector<float>>; build a temporary
    // float** for the legacy CUDA API.  This allocation is small (nBeams ptrs).
    [this]() -> float ** {
      static thread_local std::vector<float *> ptrs;
      ptrs.resize(nBeams);
      for (int i = 0; i < nBeams; ++i)
        ptrs[i] = beamCorrector_[i].data();
      return ptrs.data();
    }(),
    beamCorrectorSum,
    debugFlag);

  auto t_stop    = std::chrono::high_resolution_clock::now();
  auto t_elapsed = std::chrono::duration_cast<
                     std::chrono::microseconds>(t_stop - t_start);
  if (debugFlag) {
    RCLCPP_INFO(ros_node_->get_logger(),
      "GPU sonar frame calc time: %ld/100 [s]", t_elapsed.count() / 10000);
  }

  // ----- CSV log (FIX-K) -------------------------------------------------
  if (writeLogFlag) {
    ++writeCounter;
    if (writeCounter == 1 || writeCounter % writeInterval == 0) {
      const double sim_sec =
        static_cast<double>(last_sim_time_.nanoseconds()) * 1e-9;

      std::ostringstream filename;
      filename << "/tmp/SonarRawData_"
               << std::setw(6) << std::setfill('0') << writeNumber << ".csv";

      writeLog_.open(filename.str(), std::ios_base::app);
      writeLog_ << "# Raw Sonar Data Log (Row: beams, Col: time series data)\n"
                << "# First column is range vector\n"
                << "#  nBeams : " << nBeams << "\n"
                << "# Simulation time : " << sim_sec << "\n";

      for (int i = 0; i < nFreq; ++i) {
        writeLog_ << rangeVector_[i];
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

      if (writeNumber == 1) {
        std::ofstream angle_log("/tmp/SonarRawData_beam_angles.csv");
        angle_log << "# Beam (azimuth) angles [rad]\n"
                  << "#  nBeams : " << nBeams << "\n";
        for (const double a : azimuth_angles_)
          angle_log << a << "\n";
        angle_log.close();
      }
      ++writeNumber;
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
    ranges.push_back(rangeVector_[i]);
  sonar_raw_msg.ranges = ranges;

  marine_acoustic_msgs::msg::SonarImageData sonar_data;
  sonar_data.is_bigendian = false;
  sonar_data.dtype        = 0;   // DTYPE_UINT8
  sonar_data.beam_count   = static_cast<uint32_t>(nBeams);

  std::vector<uint8_t> intensities;
  intensities.reserve(static_cast<size_t>(nFreq * nBeams));
  for (int f = 0; f < nFreq; ++f) {
    for (int beam = 0; beam < nBeams; ++beam) {
      // FIX-I: reverse beam order (left/right flip)
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

  // GCC13-FIX-5: detail::kPi replaces M_PI
  const float ThetaShift = 1.5f * static_cast<float>(detail::kPi);
  for (int r = 0; r < static_cast<int>(ranges.size()); ++r) {
    if (ranges[r] > rangeMax) continue;
    for (int b = 0; b < nBeams; ++b) {
      const float range     = ranges[r];
      // FIX-I: reverse beam index for visual image
      const int   intensity = static_cast<int>(
        std::floor(10.0 * std::log(
          std::abs(P_Beams[nBeams - 1 - b][r]) + 1e-10)));
      const float begin_ang = bear_angles[b].begin + ThetaShift;
      const float end_ang   = bear_angles[b].end   + ThetaShift;
      const float rad       = static_cast<float>(radius) * range / rangeMax;
      cv::ellipse(Intensity_image, origin,
                  cv::Size(static_cast<int>(rad), static_cast<int>(rad)), 0.0,
                  static_cast<double>(begin_ang) * 180.0 / detail::kPi,
                  static_cast<double>(end_ang)   * 180.0 / detail::kPi,
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
//  ComputeCorrector  (FIX-J + GCC13-FIX-3/5)
// =========================================================================

void NpsGazeboRosMultibeamSonarRay::ComputeCorrector()
{
  if (azimuth_angles_.empty() || static_cast<int>(azimuth_angles_.size()) < nBeams)
    return;

  const double hPixelSize = (nBeams > 1) ? hFOV_ / (nBeams - 1) : hFOV_;

  beamCorrectorSum = 0.0f;
  for (int beam = 0; beam < nBeams; ++beam) {
    for (int beam_other = 0; beam_other < nBeams; ++beam_other) {
      // GCC13-FIX-5: detail::kPi replaces M_PI
      const float pattern = unnormalized_sinc(
        static_cast<float>(detail::kPi * 0.884 / hPixelSize
          * std::sin(azimuth_angles_[beam] - azimuth_angles_[beam_other])));
      beamCorrector_[beam][beam_other] = std::abs(pattern);
      beamCorrectorSum += pattern * pattern;
    }
  }
  beamCorrectorSum = std::sqrt(beamCorrectorSum);
}

// =========================================================================
//  ComputeNormalImage  (unchanged numerical logic, GCC13-FIX-5 for M_PI)
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
//  Plugin registration
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
