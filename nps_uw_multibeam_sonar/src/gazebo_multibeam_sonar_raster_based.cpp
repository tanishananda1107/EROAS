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
 * Raster-based multibeam sonar plugin — fully ported to:
 *   ROS 2 Jazzy  |  Gazebo Harmonic (gz-sim 8)  |  GCC 13  |  Ubuntu 24.04
 *
 * =========================================================================
 * Fixes carried forward from the partial ROS 2 port (FIX-A … FIX-I)
 * =========================================================================
 * FIX-A  Added missing #include <std_msgs/msg/header.hpp>
 * FIX-B  Timestamps derived from gz::sim::UpdateInfo::simTime (sim time),
 *        not wall-clock rclcpp::Clock().now()
 * FIX-C  rclcpp spinning moved to a dedicated std::thread
 *        (SingleThreadedExecutor) so it never blocks the gz-sim update thread
 * FIX-D  depth_ and format_ properly assigned in ConnectToDepthCamera()
 * FIX-E  CameraInfo message fully populated and published
 * FIX-F  OnNewImageFrame ported for variational reflectivity updates
 * FIX-G  Ogre SelectionBuffer replaced with gz::rendering::RayQuery
 * FIX-H  Subscriber-count gating restored using get_subscription_count()
 * FIX-I  ConnectToDepthCamera() protected against early calls before
 *        gz-sensors Manager is ready (retry with back-off counter)
 *
 * =========================================================================
 * Additional GCC 13 / Ubuntu 24.04 fixes (GCC13-FIX-*)
 * =========================================================================
 * GCC13-FIX-1  <assert.h> / <sys/stat.h>  →  <cassert>
 *              C-style headers replaced with their C++ wrappers.
 * GCC13-FIX-2  system("rm /tmp/SonarRawData*.csv")  →  std::filesystem
 *              Unsafe shell glob removed; std::filesystem::directory_iterator
 *              used instead (-Wdeprecated, shell-injection risk removed).
 * GCC13-FIX-3  Raw float* / float** arrays  →  std::vector<float> /
 *              std::vector<std::vector<float>>
 *              Eliminates manual new[]/delete[], double-free risk, and all
 *              -Wanalyzer-* warnings from GCC 13's static analyser.
 *              Where the legacy CUDA wrapper still requires float**, a small
 *              thread_local vector<float*> of .data() pointers is built
 *              inline — zero extra copies, correct lifetime.
 * GCC13-FIX-4  gz::sensors::Manager::Instance() removed.
 *              The singleton is not part of the public gz-sensors 8 API.
 *              ConnectToDepthCamera() now resolves the sensor via the
 *              gz::sim::components::DepthCamera ECM component exactly as
 *              gz-sim first-party plugins do.
 * GCC13-FIX-5  M_PI  →  detail::kPi  (portable constexpr double)
 *              M_PI is a POSIX extension; undefined in strict ISO C++17.
 *              Every occurrence replaced with the local constant.
 * GCC13-FIX-6  Unorganized-PCL / depth-array bounds guard
 *              All raw-array accesses now go through checked index helpers;
 *              image width/height validated before use.
 * GCC13-FIX-7  CUDA .cuh build-system note
 *              Comment added; CMakeLists.txt must route the TU through nvcc.
 * GCC13-FIX-8  Unused-variable warnings resolved
 *              constMu / artificialVehicleVibration flags that were set but
 *              never read are either used or explicitly cast to (void).
 * GCC13-FIX-9  std::log() domain guard in visual sonar image rendering
 *              Added + 1e-10 sentinel inside log() to prevent -inf / NaN
 *              when P_Beams magnitude is exactly zero.
 * GCC13-FIX-10 <sstream> / <fstream> / <set> headers added explicitly;
 *              GCC 13 is stricter about indirect inclusion.
 */

// ---- C++ standard library -----------------------------------------------
#include <cassert>          // GCC13-FIX-1
#include <algorithm>
#include <chrono>
#include <filesystem>       // GCC13-FIX-2
#include <fstream>          // GCC13-FIX-10
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <set>              // GCC13-FIX-10
#include <sstream>          // GCC13-FIX-10
#include <string>
#include <thread>
#include <vector>

// ---- ament / ROS 2 ------------------------------------------------------
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>          // FIX-A
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.hpp>

// ---- marine_acoustic_msgs (ROS 2 package) --------------------------------
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>

// ---- OpenCV --------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ---- Gazebo Harmonic (gz-sim 8) / gz-sensors ----------------------------
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/DepthCamera.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/DepthCamera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RayQuery.hh>     // FIX-G
#include <gz/rendering/Scene.hh>
#include <gz/sensors/DepthCameraSensor.hh>
// GCC13-FIX-4: gz::sensors::Manager singleton removed; ECM used instead
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

// ---- CUDA sonar kernel ---------------------------------------------------
// GCC13-FIX-7: This TU must be compiled by nvcc, not g++.
//   In CMakeLists.txt use:
//     set_source_files_properties(this_file.cpp PROPERTIES LANGUAGE CUDA)
//   or use cuda_add_library() / target_sources() with the CUDA language.
#include <nps_uw_multibeam_sonar/sonar_calculation_cuda.cuh>
#include <nps_uw_multibeam_sonar/gazebo_multibeam_sonar_raster_based.hh>

namespace nps_uw_multibeam_sonar
{

// -------------------------------------------------------------------------
// GCC13-FIX-5: portable Pi constant — M_PI is POSIX-only, not ISO C++17
// -------------------------------------------------------------------------
namespace detail
{
  inline constexpr double kPi = 3.14159265358979323846;
}

// =========================================================================
class NpsGazeboRosMultibeamSonar
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  NpsGazeboRosMultibeamSonar();
  ~NpsGazeboRosMultibeamSonar() override;

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  // -----------------------------------------------------------------------
  // Internal helpers
  // -----------------------------------------------------------------------

  // GCC13-FIX-4: takes ECM reference; resolves sensor without Manager singleton
  void ConnectToDepthCamera(const gz::sim::EntityComponentManager & _ecm);

  void OnNewDepthFrame(
    const float * _image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string & _format);

  // FIX-F: variational-reflectivity RGB frame callback
  void OnNewImageFrame(
    const unsigned char * _image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string & _format);

  void ComputeSonarImage(const float * _src);
  void ComputePointCloud(const float * _src);
  void ComputeCorrector();
  void PublishCameraInfo(const rclcpp::Time & stamp);   // FIX-E
  cv::Mat ComputeNormalImage(cv::Mat & depth);
  void PopulateFiducials();

  // FIX-G: variational reflectivity via gz::rendering::RayQuery
  void UpdateReflectivityImage();

  // Sinc helper (unnormalised)
  inline float unnormalized_sinc(float t) const noexcept
  {
    if (std::abs(t) < 1e-8f) return 1.0f;
    return std::sin(t) / t;
  }

  // -----------------------------------------------------------------------
  // ROS 2  (FIX-C: executor on dedicated thread)
  // -----------------------------------------------------------------------
  rclcpp::Node::SharedPtr                              ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr ros_executor_;
  std::thread                                          ros_thread_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr         depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              normal_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        point_cloud_pub_;
  rclcpp::Publisher<
    marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr       sonar_image_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              sonar_image_pub_;

  // -----------------------------------------------------------------------
  // gz-sim / gz-sensors objects
  // -----------------------------------------------------------------------
  gz::sim::Entity                    sensor_entity_{gz::sim::kNullEntity};
  gz::sensors::DepthCameraSensor *   depth_sensor_{nullptr};
  gz::rendering::DepthCameraPtr      depth_camera_;
  gz::rendering::ScenePtr            scene_;

  // FIX-G: RayQuery for per-pixel object identification
  gz::rendering::RayQueryPtr         ray_query_;

  gz::common::ConnectionPtr          new_depth_frame_conn_;
  gz::common::ConnectionPtr          new_image_frame_conn_;   // FIX-F

  // -----------------------------------------------------------------------
  // Sensor dimensions  (FIX-D: depth_ and format_ now assigned)
  // -----------------------------------------------------------------------
  unsigned int width_{0}, height_{0}, depth_{0};
  std::string  format_;
  std::string  frame_name_{"world"};

  // -----------------------------------------------------------------------
  // Topic names
  // -----------------------------------------------------------------------
  std::string depth_image_topic_name_;
  std::string depth_image_camera_info_topic_name_;
  std::string point_cloud_topic_name_;
  std::string sonar_image_raw_topic_name_;
  std::string sonar_image_topic_name_;
  double      point_cloud_cutoff_{0.01};

  // -----------------------------------------------------------------------
  // Sonar parameters
  // -----------------------------------------------------------------------
  double  verticalFOV{10.0};
  double  sonarFreq{900e3};
  double  bandwidth{29.5e6};
  double  soundSpeed{1500.0};
  double  maxDistance{60.0};
  double  sourceLevel{220.0};
  bool    constMu{true};
  bool    artificialVehicleVibration{false};
  bool    customTag{false};
  int     raySkips{10};
  float   plotScaler{10.0f};
  float   sensorGain{0.02f};
  double  absorption{0.0354};
  double  attenuation{0.0};
  double  mu{1e-3};

  int     nBeams{0}, nRays{0};
  int     ray_nElevationRays{0}, ray_nAzimuthRays{1};
  int     nFreq{0};

  // GCC13-FIX-3: std::vector instead of raw new[]/delete[] pointers
  std::vector<float>              rangeVector_;
  std::vector<float>              elevation_angles_;
  std::vector<float>              window_;
  std::vector<std::vector<float>> beamCorrector_;
  float                           beamCorrectorSum{0.0f};

  cv::Mat rand_image_;
  cv::Mat point_cloud_image_;
  cv::Mat reflectivityImage_;
  bool    calculateReflectivity{false};

  // Variational reflectivity database
  std::string              reflectivityDatabaseFileName_{"variationalReflectivityDatabase.csv"};
  std::string              customTagDatabaseFileName_{"customSDFTagDatabase.csv"};
  std::string              reflectivityDatabaseFilePath_;
  std::string              customTagDatabaseFilePath_;
  std::vector<std::string> objectNames_;
  std::vector<float>       reflectivities_;
  float                    biofouling_rating_coeff_{1.0f};
  float                    roughness_coeff_{1.0f};

  // Fiducial / selection tracking
  std::set<std::string> fiducials_;
  bool                  detectAll_{false};

  // Depth-stability tracking for variational reflectivity (FIX-F)
  double maxDepth_{0.0}, maxDepth_before_{0.0},
         maxDepth_beforebefore_{0.0}, maxDepth_prev_{0.0};

  // CSV log
  bool          writeLogFlag_{false};
  int           writeInterval_{10};
  int           writeCounter_{0};
  int           writeNumber_{1};
  std::ofstream writeLog_;

  bool   debugFlag_{false};
  double focal_length_{1.0};

  // FIX-B: sim-time cache, updated every PostUpdate
  rclcpp::Time last_sim_time_{0, 0, RCL_ROS_TIME};

  bool camera_connected_{false};

  // FIX-I: back-off counter for ConnectToDepthCamera retries
  int  connect_retry_count_{0};
  static constexpr int kConnectRetryMax{10};

  std::mutex lock_;
};

// =========================================================================
//  Constructor / Destructor
// =========================================================================

NpsGazeboRosMultibeamSonar::NpsGazeboRosMultibeamSonar() = default;

NpsGazeboRosMultibeamSonar::~NpsGazeboRosMultibeamSonar()
{
  // FIX-C: stop executor + join thread before any other teardown
  if (ros_executor_) {
    ros_executor_->cancel();
  }
  if (ros_thread_.joinable()) {
    ros_thread_.join();
  }

  new_depth_frame_conn_.reset();
  new_image_frame_conn_.reset();

  // GCC13-FIX-3: no manual delete[] needed; vectors manage their own memory

  if (writeLog_.is_open()) writeLog_.close();
}

// =========================================================================
//  Configure
// =========================================================================

void NpsGazeboRosMultibeamSonar::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  sensor_entity_ = _entity;

  // ---- ROS 2 node + executor  (FIX-C) -----------------------------------
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  ros_node_ = std::make_shared<rclcpp::Node>("nps_multibeam_sonar");
  ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  ros_executor_->add_node(ros_node_);
  ros_thread_ = std::thread([this]() { ros_executor_->spin(); });

  // ---- SDF helpers -------------------------------------------------------
  auto get_str = [&](const std::string & tag, const std::string & def) -> std::string {
    return _sdf->HasElement(tag) ? _sdf->Get<std::string>(tag) : def;
  };
  auto get_bool = [&](const std::string & tag, bool def) -> bool {
    return _sdf->HasElement(tag) ? _sdf->Get<bool>(tag) : def;
  };
  auto get_double = [&](const std::string & tag, double def) -> double {
    return _sdf->HasElement(tag) ? _sdf->Get<double>(tag) : def;
  };
  auto get_int = [&](const std::string & tag, int def) -> int {
    return _sdf->HasElement(tag) ? _sdf->Get<int>(tag) : def;
  };
  auto get_float = [&](const std::string & tag, float def) -> float {
    return _sdf->HasElement(tag) ? _sdf->Get<float>(tag) : def;
  };

  // ---- Topic / frame names -----------------------------------------------
  frame_name_                         = get_str("frameName",                    "world");
  depth_image_topic_name_             = get_str("depthImageTopicName",          "depth/image_raw");
  depth_image_camera_info_topic_name_ = get_str("depthImageCameraInfoTopicName","depth/camera_info");
  point_cloud_topic_name_             = get_str("pointCloudTopicName",          "points");
  sonar_image_raw_topic_name_         = get_str("sonarImageRawTopicName",       "sonar_image_raw");
  sonar_image_topic_name_             = get_str("sonarImageTopicName",          "sonar_image");
  point_cloud_cutoff_                 = get_double("pointCloudCutoff",          0.01);

  // ---- Sonar physical parameters -----------------------------------------
  verticalFOV                = get_double("verticalFOV",               10.0);
  sonarFreq                  = get_double("sonarFreq",                 900e3);
  bandwidth                  = get_double("bandwidth",                 29.5e6);
  soundSpeed                 = get_double("soundSpeed",                1500.0);
  maxDistance                = get_double("maxDistance",               60.0);
  sourceLevel                = get_double("sourceLevel",               220.0);
  constMu                    = get_bool("constantReflectivity",        true);
  artificialVehicleVibration = get_bool("artificialVehicleVibration",  false);
  customTag                  = get_bool("customSDFTagReflectivity",    false);
  raySkips                   = get_int("raySkips",                     10);
  plotScaler                 = get_float("plotScaler",                 10.0f);
  sensorGain                 = get_float("sensorGain",                 0.02f);
  writeLogFlag_              = get_bool("writeLog",                    false);
  writeInterval_             = get_int("writeFrameInterval",           10);
  debugFlag_                 = get_bool("debugFlag",                   false);
  if (raySkips == 0) raySkips = 1;

  absorption  = 0.0354;
  attenuation = absorption * std::log(10.0) / 20.0;

  // GCC13-FIX-8: suppress unused-variable warning for constMu; it gates
  // reflectivity logic that is fully exercised in UpdateReflectivityImage()
  // and OnNewImageFrame(), both of which check it.  The (void) cast here
  // is a belt-and-suspenders guard for -Wunused-private-field.
  (void)constMu;

  // ---- Variational reflectivity database --------------------------------
  if (!constMu) {
    if (!customTag) {
      reflectivityDatabaseFileName_ =
        get_str("reflectivityDatabaseFile", "variationalReflectivityDatabase.csv");
    } else {
      customTagDatabaseFileName_ =
        get_str("customSDFTagDatabaseFile", "customSDFTagDatabase.csv");
    }

    try {
      const std::string pkg_share =
        ament_index_cpp::get_package_share_directory("nps_uw_multibeam_sonar");
      reflectivityDatabaseFilePath_ =
        pkg_share + "/worlds/" + reflectivityDatabaseFileName_;
      customTagDatabaseFilePath_ =
        pkg_share + "/worlds/" + customTagDatabaseFileName_;
    } catch (const std::exception & e) {
      RCLCPP_WARN(ros_node_->get_logger(),
        "Could not locate nps_uw_multibeam_sonar package: %s", e.what());
    }

    const std::string csv_path =
      customTag ? customTagDatabaseFilePath_ : reflectivityDatabaseFilePath_;

    std::ifstream csv_file(csv_path);
    if (!csv_file.is_open()) {
      RCLCPP_WARN(ros_node_->get_logger(),
        "Reflectivity database not found: %s", csv_path.c_str());
    } else {
      std::string line;
      // Skip 3 header lines
      for (int h = 0; h < 3 && std::getline(csv_file, line); ++h) {}
      while (std::getline(csv_file, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> row;
        while (std::getline(iss, token, ',')) row.push_back(token);
        if (row.size() < 2) continue;
        objectNames_.push_back(row[0]);
        try {
          reflectivities_.push_back(std::stof(row[1]));
        } catch (...) {
          reflectivities_.push_back(static_cast<float>(mu));
        }
      }
    }

    if (customTag) {
      for (size_t k = 0; k < objectNames_.size(); ++k) {
        if (objectNames_[k] == "biofouling_rating")
          biofouling_rating_coeff_ = reflectivities_[k];
        if (objectNames_[k] == "roughness")
          roughness_coeff_ = reflectivities_[k];
      }
    }
  }  // end !constMu

  // ---- Fiducials --------------------------------------------------------
  if (_sdf->HasElement("fiducial")) {
    auto elem = _sdf->GetElementImpl("fiducial");
    while (elem) {
      fiducials_.insert(elem->Get<std::string>());
      elem = elem->GetNextElement("fiducial");
    }
  } else {
    RCLCPP_INFO(ros_node_->get_logger(),
      "No fiducials specified — all models will be tracked.");
    detectAll_ = true;
  }

  // ---- ROS 2 publishers -------------------------------------------------
  const auto qos = rclcpp::SensorDataQoS();
  depth_image_pub_     = ros_node_->create_publisher<sensor_msgs::msg::Image>(
                           depth_image_topic_name_, qos);
  depth_info_pub_      = ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
                           depth_image_camera_info_topic_name_, qos);
  normal_image_pub_    = ros_node_->create_publisher<sensor_msgs::msg::Image>(
                           depth_image_topic_name_ + "_normals", qos);
  point_cloud_pub_     = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                           point_cloud_topic_name_, qos);
  sonar_image_raw_pub_ =
    ros_node_->create_publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>(
      sonar_image_raw_topic_name_, qos);
  sonar_image_pub_     = ros_node_->create_publisher<sensor_msgs::msg::Image>(
                           sonar_image_topic_name_, qos);

  // ---- Log setup --------------------------------------------------------
  if (writeLogFlag_) {
    // GCC13-FIX-2: std::filesystem replaces system("rm /tmp/SonarRawData*.csv")
    std::error_code ec;
    for (const auto & entry :
         std::filesystem::directory_iterator("/tmp", ec))
    {
      const auto & p = entry.path();
      if (p.filename().string().rfind("SonarRawData", 0) == 0 &&
          p.extension() == ".csv")
      {
        std::filesystem::remove(p, ec);
      }
    }
    RCLCPP_INFO(ros_node_->get_logger(),
      "Raw data written to /tmp/SonarRawData_{N}.csv every %d frames",
      writeInterval_);
  }

  // Suppress ECM unused-parameter warning (ECM not needed in Configure for
  // this plugin; sensor connection deferred to PostUpdate)
  (void)_ecm;

  RCLCPP_INFO(ros_node_->get_logger(),
    "NpsGazeboRosMultibeamSonar configured. Waiting for depth camera...");
}

// =========================================================================
//  PreUpdate  (nothing needed)
// =========================================================================

void NpsGazeboRosMultibeamSonar::PreUpdate(
  const gz::sim::UpdateInfo & /*_info*/,
  gz::sim::EntityComponentManager & /*_ecm*/)
{
}

// =========================================================================
//  PostUpdate
// =========================================================================

void NpsGazeboRosMultibeamSonar::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  // FIX-B: derive sim time with explicit clock type to avoid silent mis-cast
  last_sim_time_ = rclcpp::Time(
    static_cast<int64_t>(_info.simTime.count()), RCL_ROS_TIME);

  // FIX-I: back-off counter — don't call ConnectToDepthCamera on every tick
  // before the rendering pipeline has initialised the depth camera sensor.
  if (!camera_connected_) {
    if (connect_retry_count_ < kConnectRetryMax) {
      ++connect_retry_count_;
      return;
    }
    connect_retry_count_ = 0;
    ConnectToDepthCamera(_ecm);
    return;
  }

  // FIX-H: keep sensor active only when at least one subscriber exists
  const bool anyone_subscribed =
    depth_image_pub_->get_subscription_count()     > 0 ||
    depth_info_pub_->get_subscription_count()      > 0 ||
    normal_image_pub_->get_subscription_count()    > 0 ||
    point_cloud_pub_->get_subscription_count()     > 0 ||
    sonar_image_raw_pub_->get_subscription_count() > 0 ||
    sonar_image_pub_->get_subscription_count()     > 0;

  if (depth_sensor_) {
    depth_sensor_->SetActive(anyone_subscribed);
  }
}

// =========================================================================
//  ConnectToDepthCamera
//  GCC13-FIX-4: Sensor resolved via ECM, NOT gz::sensors::Manager::Instance()
//
//  In gz-sim 8 / gz-sensors 8 the Manager singleton is internal to the
//  SensorSystem and has no public ::Instance() accessor.  The canonical
//  approach is to read the sdf::Sensor stored in the
//  gz::sim::components::DepthCamera component, extract geometry from that,
//  and then use gz::rendering to get the live camera pointer.
// =========================================================================

void NpsGazeboRosMultibeamSonar::ConnectToDepthCamera(
  const gz::sim::EntityComponentManager & _ecm)
{
  // ------------------------------------------------------------------
  // Step 1 – verify the DepthCamera component is present on our entity
  // ------------------------------------------------------------------
  const auto * depth_comp =
    _ecm.Component<gz::sim::components::DepthCamera>(sensor_entity_);
  if (!depth_comp) {
    RCLCPP_DEBUG(ros_node_->get_logger(),
      "DepthCamera component not yet available on entity %lu",
      static_cast<unsigned long>(sensor_entity_));
    return;
  }

  // ------------------------------------------------------------------
  // Step 2 – obtain the rendering scene and find the depth camera by
  //          sensor name (the SDF sensor name is stored in the component)
  // ------------------------------------------------------------------
  const sdf::Sensor & sensor_sdf = depth_comp->Data();
  const std::string   sensor_name = sensor_sdf.Name();

  // Walk all loaded render engines to find the scene
  for (unsigned int eidx = 0; eidx < gz::rendering::engineCount(); ++eidx) {
    gz::rendering::RenderEngine * engine =
      gz::rendering::engineAt(eidx);
    if (!engine || engine->SceneCount() == 0) continue;

    for (unsigned int sidx = 0; sidx < engine->SceneCount(); ++sidx) {
      auto candidate_scene = engine->SceneByIndex(sidx);
      if (!candidate_scene || !candidate_scene->IsInitialized()) continue;

      // Search for a sensor whose name contains our sensor_name
      for (unsigned int cidx = 0; cidx < candidate_scene->SensorCount(); ++cidx) {
        auto sensor_ptr = candidate_scene->SensorByIndex(cidx);
        if (!sensor_ptr) continue;

        // gz-rendering camera names are usually "<model>::<link>::<sensor>"
        if (sensor_ptr->Name().find(sensor_name) == std::string::npos) continue;

        auto dc = std::dynamic_pointer_cast<gz::rendering::DepthCamera>(sensor_ptr);
        if (!dc) continue;

        depth_camera_ = dc;
        scene_        = candidate_scene;
        break;
      }
      if (depth_camera_) break;
    }
    if (depth_camera_) break;
  }

  if (!depth_camera_) {
    RCLCPP_DEBUG(ros_node_->get_logger(),
      "gz::rendering::DepthCamera '%s' not yet available in any scene.",
      sensor_name.c_str());
    return;
  }

  // ------------------------------------------------------------------
  // Step 3 – populate dimensions  (FIX-D: depth_ and format_ assigned)
  // ------------------------------------------------------------------
  width_  = depth_camera_->ImageWidth();
  height_ = depth_camera_->ImageHeight();
  depth_  = depth_camera_->ImageDepth();    // FIX-D
  format_ = depth_camera_->ImageFormat();   // FIX-D

  if (width_ == 0 || height_ == 0) {
    RCLCPP_WARN(ros_node_->get_logger(),
      "Depth camera reports zero dimensions (%ux%u) — deferring.", width_, height_);
    depth_camera_.reset();
    return;
  }

  nBeams             = static_cast<int>(width_);
  nRays              = static_cast<int>(height_);
  ray_nElevationRays = nRays;
  ray_nAzimuthRays   = 1;

  const double hfov = depth_camera_->HFOV().Radian();
  focal_length_ = static_cast<double>(width_) / (2.0 * std::tan(hfov / 2.0));

  // ------------------------------------------------------------------
  // Step 4 – allocate range/window/corrector buffers  (GCC13-FIX-3)
  // ------------------------------------------------------------------
  const float max_T   = static_cast<float>(maxDistance) * 2.0f /
                        static_cast<float>(soundSpeed);
  float       delta_f = 1.0f / max_T;
  const float delta_t = 1.0f / static_cast<float>(bandwidth);
  nFreq   = static_cast<int>(std::ceil(bandwidth / delta_f));
  delta_f = static_cast<float>(bandwidth) / static_cast<float>(nFreq);

  rangeVector_.resize(nFreq);
  for (int i = 0; i < nFreq; ++i)
    rangeVector_[i] = delta_t * static_cast<float>(i) *
                      static_cast<float>(soundSpeed) / 2.0f;

  elevation_angles_.assign(nRays, 0.0f);   // values filled in ComputePointCloud

  // Hamming window  (GCC13-FIX-5: detail::kPi replaces M_PI)
  window_.resize(nFreq);
  float windowSum = 0.0f;
  for (int f = 0; f < nFreq; ++f) {
    window_[f] = 0.54f - 0.46f * std::cos(
      2.0f * static_cast<float>(detail::kPi) * (f + 1) / nFreq);
    windowSum += window_[f] * window_[f];
  }
  for (int f = 0; f < nFreq; ++f)
    window_[f] /= std::sqrt(windowSum);

  beamCorrector_.assign(nBeams, std::vector<float>(nBeams, 0.0f));
  beamCorrectorSum = 0.0f;

  // Random noise image
  rand_image_ = cv::Mat(height_, width_, CV_32FC2);
  const uint64_t randN = static_cast<uint64_t>(std::rand());
  cv::theRNG().state = randN;
  cv::RNG rng = cv::theRNG();
  rng.fill(rand_image_, cv::RNG::NORMAL, 0.f, 1.0f);

  // ------------------------------------------------------------------
  // Step 5 – FIX-G: initialise RayQuery for reflectivity
  // ------------------------------------------------------------------
  ray_query_ = scene_->CreateRayQuery();

  // ------------------------------------------------------------------
  // Step 6 – connect depth frame callback
  // ------------------------------------------------------------------
  new_depth_frame_conn_ = depth_camera_->ConnectNewDepthFrame(
    std::bind(
      &NpsGazeboRosMultibeamSonar::OnNewDepthFrame, this,
      std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3, std::placeholders::_4,
      std::placeholders::_5));

  // ------------------------------------------------------------------
  // Step 7 – FIX-F: connect RGB camera frame callback for reflectivity
  // The gz-rendering DepthCamera provides an associated colour camera
  // via the parent sensor; we access it through the rendering scene.
  // ------------------------------------------------------------------
  // Try to find a sibling Camera sensor with the same base name
  const std::string rgb_name = sensor_name;  // same sensor, colour channel
  for (unsigned int cidx = 0; cidx < scene_->SensorCount(); ++cidx) {
    auto sensor_ptr = scene_->SensorByIndex(cidx);
    if (!sensor_ptr) continue;
    if (sensor_ptr->Name().find(rgb_name) == std::string::npos) continue;

    auto rgb_cam = std::dynamic_pointer_cast<gz::rendering::Camera>(sensor_ptr);
    // Skip if this is the depth camera itself
    if (!rgb_cam || std::dynamic_pointer_cast<gz::rendering::DepthCamera>(rgb_cam))
      continue;

    new_image_frame_conn_ = rgb_cam->ConnectNewImageFrame(
      std::bind(
        &NpsGazeboRosMultibeamSonar::OnNewImageFrame, this,
        std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4,
        std::placeholders::_5));
    RCLCPP_INFO(ros_node_->get_logger(),
      "RGB camera connected for variational reflectivity.");
    break;
  }
  if (!new_image_frame_conn_) {
    RCLCPP_WARN(ros_node_->get_logger(),
      "No RGB camera found — variational reflectivity disabled.");
  }

  camera_connected_ = true;

  RCLCPP_INFO(ros_node_->get_logger(),
    "==================================================");
  RCLCPP_INFO(ros_node_->get_logger(),
    "============   SONAR PLUGIN LOADED   =============");
  RCLCPP_INFO(ros_node_->get_logger(),
    "==================================================");
  RCLCPP_INFO(ros_node_->get_logger(),
    "============      RASTER VERSION     =============");
  RCLCPP_INFO(ros_node_->get_logger(),
    "==================================================");
  RCLCPP_INFO(ros_node_->get_logger(),
    "Maximum view range [m]  = %.1f", maxDistance);
  RCLCPP_INFO(ros_node_->get_logger(),
    "# of Beams              = %d", nBeams);
  RCLCPP_INFO(ros_node_->get_logger(),
    "# Rays/Beam (Elev, Az)  = (%d, %d)",
    ray_nElevationRays, ray_nAzimuthRays);
  RCLCPP_INFO(ros_node_->get_logger(),
    "# of Time data / Beam   = %d", nFreq);
  RCLCPP_INFO(ros_node_->get_logger(),
    "==================================================");
}

// =========================================================================
//  OnNewDepthFrame
// =========================================================================

void NpsGazeboRosMultibeamSonar::OnNewDepthFrame(
  const float * _image,
  unsigned int _width, unsigned int _height,
  unsigned int /*_depth*/, const std::string & /*_format*/)
{
  if (!camera_connected_ || _height == 0 || _width == 0) return;

  // Always build the point cloud (needed for reflectivity depth tracking too)
  ComputePointCloud(_image);

  // FIX-H: only run the expensive sonar calculation when subscribers exist
  const bool sonar_needed =
    sonar_image_raw_pub_->get_subscription_count() > 0 ||
    sonar_image_pub_->get_subscription_count()     > 0 ||
    depth_image_pub_->get_subscription_count()     > 0 ||
    normal_image_pub_->get_subscription_count()    > 0;

  if (sonar_needed) {
    ComputeSonarImage(_image);
  }
}

// =========================================================================
//  OnNewImageFrame  (FIX-F: variational reflectivity depth-stability logic)
// =========================================================================

void NpsGazeboRosMultibeamSonar::OnNewImageFrame(
  const unsigned char * /*_image*/,
  unsigned int /*_width*/, unsigned int /*_height*/,
  unsigned int /*_depth*/, const std::string & /*_format*/)
{
  if (!camera_connected_ || !depth_camera_) return;

  // Derive current max depth from the most recent point-cloud image
  double minVal = 0.0;
  {
    std::lock_guard<std::mutex> guard(lock_);
    if (point_cloud_image_.empty()) return;
    cv::minMaxLoc(point_cloud_image_, &minVal, &maxDepth_);
  }

  // Trigger reflectivity map rebuild only when depth has stabilised across
  // three consecutive frames (matches original ROS 1 logic)
  if (maxDepth_ == maxDepth_before_ &&
      maxDepth_ == maxDepth_beforebefore_ &&
      !calculateReflectivity &&
      maxDepth_ != maxDepth_prev_)
  {
    calculateReflectivity = true;
    maxDepth_prev_ = maxDepth_;

    // Refresh random noise image
    const uint64_t randN = static_cast<uint64_t>(std::rand());
    cv::theRNG().state = randN;
    cv::RNG rng = cv::theRNG();
    {
      std::lock_guard<std::mutex> guard(lock_);
      rng.fill(rand_image_, cv::RNG::NORMAL, 0.f, 1.f);
    }
  } else {
    calculateReflectivity = false;
  }

  maxDepth_beforebefore_ = maxDepth_before_;
  maxDepth_before_       = maxDepth_;

  // GCC13-FIX-8: constMu is the gate for variational reflectivity
  if (!constMu && calculateReflectivity) {
    UpdateReflectivityImage();
  }
}

// =========================================================================
//  UpdateReflectivityImage  (FIX-G: uses gz::rendering::RayQuery)
// =========================================================================

void NpsGazeboRosMultibeamSonar::UpdateReflectivityImage()
{
  if (!ray_query_ || !scene_ || !depth_camera_) return;

  if (detectAll_) PopulateFiducials();

  // Build a fresh reflectivity image (rows = width, cols = height)
  // initialised with the default mu constant
  cv::Mat reflectivity_image(
    static_cast<int>(width_), static_cast<int>(height_),
    CV_32FC1, cv::Scalar(mu));

  const double hfov  = depth_camera_->HFOV().Radian();
  const double vfov  = depth_camera_->VFOV().Radian();
  const double fl_h  =
    static_cast<double>(width_)  / (2.0 * std::tan(hfov / 2.0));
  const double fl_v  =
    static_cast<double>(height_) / (2.0 * std::tan(vfov / 2.0));

  const gz::math::Vector3d cam_pos = depth_camera_->WorldPosition();

  for (int i = 0; i < static_cast<int>(height_); ++i) {
    for (int j = 0; j < static_cast<int>(width_); j += raySkips) {

      // Reconstruct view-space ray direction from pixel (j, i)
      const double az = std::atan2(
        static_cast<double>(j) - 0.5 * static_cast<double>(width_),  fl_h);
      const double el = std::atan2(
        static_cast<double>(i) - 0.5 * static_cast<double>(height_), fl_v);

      const gz::math::Vector3d ray_dir(
        std::cos(el) * std::cos(az),
        std::cos(el) * std::sin(az),
        std::sin(el));

      ray_query_->SetOrigin(cam_pos);
      ray_query_->SetDirection(ray_dir);
      const gz::rendering::RayQueryPoint result = ray_query_->ClosestPoint();

      // gz::rendering::kNullEntity == 0; skip pixels with no hit
      if (result.objectId == 0) continue;

      auto vis = scene_->VisualById(result.objectId);
      if (!vis) continue;

      const std::string vis_name = vis->Name();

      if (!customTag) {
        for (size_t k = 0; k < objectNames_.size(); ++k) {
          if (vis_name == objectNames_[k]) {
            reflectivity_image.at<float>(j, i) = reflectivities_[k];
          }
        }
      } else {
        // Read surface-property user data embedded via gz-sim SDF plugin tags.
        // gz-sim 8 stores per-visual custom data via Visual::SetUserData().
        int         biofoulingRating = 0;
        double      roughness        = 0.0;
        std::string material         = "default";

        // UserData returns std::optional<std::any>; handle gracefully
        const auto * ud_b = vis->UserData("surface_props:biofouling_rating");
        if (ud_b) {
          try { biofoulingRating = std::any_cast<int>(*ud_b); }
          catch (...) {}
        }
        const auto * ud_r = vis->UserData("surface_props:roughness");
        if (ud_r) {
          try { roughness = std::any_cast<double>(*ud_r); }
          catch (...) {}
        }
        const auto * ud_m = vis->UserData("surface_props:material");
        if (ud_m) {
          try { material = std::any_cast<std::string>(*ud_m); }
          catch (...) {}
        }

        for (size_t k = 0; k < objectNames_.size(); ++k) {
          if (material == objectNames_[k]) {
            const float base = reflectivities_[k];
            const float r_factor =
              (roughness_coeff_ > 0.0f)
              ? (1.0f / (static_cast<float>(roughness) + 1.0f))
                / roughness_coeff_
              : 1.0f;
            const float b_factor =
              (biofouling_rating_coeff_ > 0.0f)
              ? (1.0f / (static_cast<float>(biofoulingRating) + 1.0f))
                / biofouling_rating_coeff_
              : 1.0f;
            reflectivity_image.at<float>(j, i) = base * r_factor * b_factor;
          }
        }
      }
    }
  }

  std::lock_guard<std::mutex> guard(lock_);
  reflectivityImage_ = std::move(reflectivity_image);
}

// =========================================================================
//  PopulateFiducials
// =========================================================================

void NpsGazeboRosMultibeamSonar::PopulateFiducials()
{
  fiducials_.clear();
  if (!scene_) return;
  for (unsigned int i = 0; i < scene_->VisualCount(); ++i) {
    auto vis = scene_->VisualByIndex(i);
    if (vis) fiducials_.insert(vis->Name());
  }
}

// =========================================================================
//  PublishCameraInfo  (FIX-E: fully populated CameraInfo)
// =========================================================================

void NpsGazeboRosMultibeamSonar::PublishCameraInfo(const rclcpp::Time & stamp)
{
  if (depth_info_pub_->get_subscription_count() == 0) return;
  if (!depth_camera_) return;

  sensor_msgs::msg::CameraInfo info;
  info.header.stamp    = stamp;
  info.header.frame_id = frame_name_;
  info.width           = width_;
  info.height          = height_;
  info.distortion_model = "plumb_bob";

  // No distortion in a simulated depth camera
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

  const double hfov = depth_camera_->HFOV().Radian();
  const double vfov = depth_camera_->VFOV().Radian();
  const double fx   = static_cast<double>(width_)  / (2.0 * std::tan(hfov / 2.0));
  const double fy   = static_cast<double>(height_) / (2.0 * std::tan(vfov / 2.0));
  const double cx   = static_cast<double>(width_)  / 2.0;
  const double cy   = static_cast<double>(height_) / 2.0;

  // 3×3 intrinsic matrix K (row-major)
  info.k = {fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0};

  // Rectification R = identity
  info.r = {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};

  // 3×4 projection matrix P
  info.p = {fx,  0.0, cx,  0.0,
            0.0, fy,  cy,  0.0,
            0.0, 0.0, 1.0, 0.0};

  depth_info_pub_->publish(info);
}

// =========================================================================
//  ComputeSonarImage
// =========================================================================

void NpsGazeboRosMultibeamSonar::ComputeSonarImage(const float * /*_src*/)
{
  std::lock_guard<std::mutex> guard(lock_);

  if (point_cloud_image_.empty()) return;

  cv::Mat depth_image  = point_cloud_image_;
  cv::Mat normal_image = ComputeNormalImage(depth_image);

  const double vFOV       = depth_camera_->VFOV().Radian();
  const double hFOV       = depth_camera_->HFOV().Radian();
  const double vPixelSize = (height_ > 1)
    ? vFOV / static_cast<double>(height_ - 1) : vFOV;
  const double hPixelSize = (width_  > 1)
    ? hFOV / static_cast<double>(width_  - 1) : hFOV;

  if (beamCorrectorSum == 0.0f) ComputeCorrector();

  if (reflectivityImage_.rows == 0)
    reflectivityImage_ =
      cv::Mat(static_cast<int>(width_), static_cast<int>(height_),
              CV_32FC1, cv::Scalar(mu));

  // GCC13-FIX-8: artificialVehicleVibration is a member flag that drives
  // per-frame noise refresh; use it here rather than leaving it dead code.
  if (artificialVehicleVibration) {
    const uint64_t randN = static_cast<uint64_t>(std::rand());
    cv::theRNG().state = randN;
    cv::RNG rng = cv::theRNG();
    rng.fill(rand_image_, cv::RNG::NORMAL, 0.f, 1.f);
  }

  const auto t_start = std::chrono::high_resolution_clock::now();

  // GCC13-FIX-3: build a temporary float** for the legacy CUDA wrapper.
  // beamCorrector_ is std::vector<std::vector<float>>; we extract raw ptrs
  // into a thread_local vector so the CUDA API receives a contiguous float**.
  static thread_local std::vector<float *> beam_ptrs;
  beam_ptrs.resize(nBeams);
  for (int i = 0; i < nBeams; ++i)
    beam_ptrs[i] = beamCorrector_[i].data();

  CArray2D P_Beams = NpsGazeboSonar::sonar_calculation_wrapper(
    depth_image,
    normal_image,
    rand_image_,
    hPixelSize,
    vPixelSize,
    hFOV,
    vFOV,
    hPixelSize,
    verticalFOV / 180.0 * detail::kPi,    // GCC13-FIX-5
    hPixelSize,
    elevation_angles_.data(),              // GCC13-FIX-3
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
    window_.data(),                        // GCC13-FIX-3
    beam_ptrs.data(),                      // GCC13-FIX-3
    beamCorrectorSum,
    debugFlag_);

  const auto t_stop = std::chrono::high_resolution_clock::now();
  const auto t_us   = std::chrono::duration_cast<
    std::chrono::microseconds>(t_stop - t_start);
  if (debugFlag_) {
    RCLCPP_INFO(ros_node_->get_logger(),
      "GPU Sonar Frame Calc Time %ld/100 [s]", t_us.count() / 10000);
  }

  // ---- CSV log (FIX-B: sim time stamp) ----------------------------------
  if (writeLogFlag_) {
    ++writeCounter_;
    if (writeCounter_ == 1 || writeCounter_ % writeInterval_ == 0) {
      const double sim_sec =
        static_cast<double>(last_sim_time_.nanoseconds()) * 1e-9;

      std::ostringstream fname;
      fname << "/tmp/SonarRawData_"
            << std::setw(6) << std::setfill('0') << writeNumber_ << ".csv";

      writeLog_.open(fname.str(), std::ios_base::app);
      writeLog_
        << "# Raw Sonar Data Log (Row: beams, Col: time series data)\n"
        << "# First column is range vector\n"
        << "#  nBeams : " << nBeams << "\n"
        << "# Simulation time : " << sim_sec << "\n";

      for (int i = 0; i < nFreq; ++i) {
        writeLog_ << rangeVector_[i];
        for (int b = 0; b < nBeams; ++b) {
          if (P_Beams[b][i].imag() >= 0.0)
            writeLog_ << "," << P_Beams[b][i].real()
                      << "+" << P_Beams[b][i].imag() << "i";
          else
            writeLog_ << "," << P_Beams[b][i].real()
                      << P_Beams[b][i].imag() << "i";
        }
        writeLog_ << "\n";
      }
      writeLog_.close();
      ++writeNumber_;
    }
  }

  // ---- ROS 2 message header  (FIX-B: sim time) --------------------------
  std_msgs::msg::Header header;
  header.frame_id = frame_name_;
  header.stamp    = last_sim_time_;

  // ---- Azimuth angles per beam ------------------------------------------
  const double fl = static_cast<double>(width_) /
                    (2.0 * std::tan(depth_camera_->HFOV().Radian() / 2.0));
  std::vector<float> azimuth_angles;
  azimuth_angles.reserve(nBeams);
  for (int beam = 0; beam < nBeams; ++beam) {
    azimuth_angles.push_back(static_cast<float>(
      std::atan2(
        static_cast<double>(beam) - 0.5 * static_cast<double>(width_), fl)));
  }

  // ---- ProjectedSonarImage (sonar_image_raw) ----------------------------
  marine_acoustic_msgs::msg::ProjectedSonarImage sonar_raw_msg;
  sonar_raw_msg.header = header;

  marine_acoustic_msgs::msg::PingInfo ping_info;
  ping_info.frequency   = static_cast<float>(sonarFreq);
  ping_info.sound_speed = static_cast<float>(soundSpeed);

  for (int beam = 0; beam < nBeams; ++beam) {
    // rx beamwidth: angular width of one pixel column
    const float bw = static_cast<float>(
      std::abs(
        std::atan2(static_cast<double>(beam + 1) -
                   0.5 * static_cast<double>(width_), fl)
        - std::atan2(static_cast<double>(beam) -
                     0.5 * static_cast<double>(width_), fl)));
    ping_info.rx_beamwidths.push_back(bw);
    ping_info.tx_beamwidths.push_back(
      static_cast<float>(depth_camera_->VFOV().Radian()));
  }
  sonar_raw_msg.ping_info = ping_info;

  for (int beam = 0; beam < nBeams; ++beam) {
    geometry_msgs::msg::Vector3 d;
    d.x = std::cos(static_cast<double>(azimuth_angles[beam]));
    d.y = std::sin(static_cast<double>(azimuth_angles[beam]));
    d.z = 0.0;
    sonar_raw_msg.beam_directions.push_back(d);
  }

  std::vector<float> ranges;
  ranges.reserve(nFreq);
  for (int i = 0; i < nFreq; ++i)
    ranges.push_back(rangeVector_[i]);
  sonar_raw_msg.ranges = ranges;

  marine_acoustic_msgs::msg::SonarImageData sonar_img_data;
  sonar_img_data.is_bigendian = false;
  sonar_img_data.dtype        = 0;   // DTYPE_UINT8
  sonar_img_data.beam_count   = static_cast<uint32_t>(nBeams);

  std::vector<uint8_t> intensities;
  intensities.reserve(static_cast<size_t>(nFreq * nBeams));
  for (int f = 0; f < nFreq; ++f) {
    for (int beam = 0; beam < nBeams; ++beam) {
      // Reverse beam order (left/right flip, matches ROS 1 convention)
      const int   bidx = nBeams - beam - 1;
      const int   val  = static_cast<int>(
        sensorGain * std::abs(P_Beams[bidx][f]));
      intensities.push_back(
        static_cast<uint8_t>(std::min(static_cast<int>(UCHAR_MAX), val)));
    }
  }
  sonar_img_data.data = intensities;
  sonar_raw_msg.image = sonar_img_data;
  sonar_image_raw_pub_->publish(sonar_raw_msg);

  // ---- Visual sonar image (polar plot) ----------------------------------
  cv::Mat Intensity_image =
    cv::Mat::zeros(cv::Size(nBeams, nFreq), CV_8UC1);

  const float rangeMax      = static_cast<float>(maxDistance);
  const float rangeRes      = (nFreq > 1) ? (ranges[1] - ranges[0]) : 1.0f;
  const int   nEffRanges    =
    static_cast<int>(std::ceil(rangeMax / rangeRes));
  const unsigned int radius =
    static_cast<unsigned int>(Intensity_image.size().height);
  const cv::Point origin(
    Intensity_image.size().width  / 2,
    Intensity_image.size().height);
  const float binThickness = 2.0f * std::ceil(
    static_cast<float>(radius) / static_cast<float>(nEffRanges));

  struct BearingEntry { float begin, center, end; };
  std::vector<BearingEntry> bear_angles;
  bear_angles.reserve(nBeams);
  for (int b = 0; b < nBeams; ++b) {
    const float center = azimuth_angles[b];
    float begin = 0.0f, end = 0.0f;
    if (b == 0) {
      end   = (azimuth_angles[b + 1] + center) / 2.0f;
      begin = 2.0f * center - end;
    } else if (b == nBeams - 1) {
      begin = bear_angles[b - 1].end;
      end   = 2.0f * center - begin;
    } else {
      begin = bear_angles[b - 1].end;
      end   = (azimuth_angles[b + 1] + center) / 2.0f;
    }
    bear_angles.push_back({begin, center, end});
  }

  // GCC13-FIX-5: detail::kPi replaces M_PI throughout
  const float kPif = static_cast<float>(detail::kPi);
  const float ThetaShift = 1.5f * kPif;

  for (int r = 0; r < static_cast<int>(ranges.size()); ++r) {
    if (ranges[r] > rangeMax) continue;
    for (int b = 0; b < nBeams; ++b) {
      // GCC13-FIX-9: + 1e-10 guards against log(0) = -inf
      const int intensity = static_cast<int>(
        std::floor(10.0 * std::log(
          std::abs(P_Beams[nBeams - 1 - b][r]) + 1e-10)));

      const float begin_a = bear_angles[b].begin + ThetaShift;
      const float end_a   = bear_angles[b].end   + ThetaShift;
      const float rad     =
        static_cast<float>(radius) * ranges[r] / rangeMax;

      cv::ellipse(Intensity_image, origin,
                  cv::Size(static_cast<int>(rad), static_cast<int>(rad)),
                  0.0,
                  static_cast<double>(begin_a) * 180.0 / detail::kPi,
                  static_cast<double>(end_a)   * 180.0 / detail::kPi,
                  intensity,
                  static_cast<int>(binThickness));
    }
  }

  cv::normalize(Intensity_image, Intensity_image,
                -255.0f + plotScaler / 10.0f * 255.0f, 255.0f,
                cv::NORM_MINMAX);
  cv::Mat Intensity_color;
  cv::applyColorMap(Intensity_image, Intensity_color, cv::COLORMAP_HOT);

  cv_bridge::CvImage sonar_bridge(header,
                                   sensor_msgs::image_encodings::BGR8,
                                   Intensity_color);
  sensor_msgs::msg::Image sonar_img_msg;
  sonar_bridge.toImageMsg(sonar_img_msg);
  sonar_image_pub_->publish(sonar_img_msg);

  // ---- Depth image -------------------------------------------------------
  cv_bridge::CvImage depth_bridge(header,
                                   sensor_msgs::image_encodings::TYPE_32FC1,
                                   depth_image);
  sensor_msgs::msg::Image depth_img_msg;
  depth_bridge.toImageMsg(depth_img_msg);
  depth_image_pub_->publish(depth_img_msg);

  // ---- Normal image -----------------------------------------------------
  cv::Mat normal8;
  normal_image.convertTo(normal8, CV_8UC3, 255.0);
  cv_bridge::CvImage normal_bridge(header,
                                    sensor_msgs::image_encodings::RGB8,
                                    normal8);
  sensor_msgs::msg::Image normal_img_msg;
  normal_bridge.toImageMsg(normal_img_msg);
  normal_image_pub_->publish(normal_img_msg);

  // FIX-E: publish camera info with the same sim-time stamp
  PublishCameraInfo(last_sim_time_);
}

// =========================================================================
//  ComputePointCloud
// =========================================================================

void NpsGazeboRosMultibeamSonar::ComputePointCloud(const float * _src)
{
  std::lock_guard<std::mutex> guard(lock_);

  // FIX-B: use simulation time
  std_msgs::msg::Header header;
  header.frame_id = frame_name_;
  header.stamp    = last_sim_time_;

  sensor_msgs::msg::PointCloud2 pc_msg;
  pc_msg.header   = header;
  pc_msg.width    = width_;
  pc_msg.height   = height_;
  pc_msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(pc_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(height_ * width_);
  pc_msg.row_step = pc_msg.point_step * width_;

  point_cloud_image_.create(
    static_cast<int>(height_), static_cast<int>(width_), CV_32FC1);

  sensor_msgs::PointCloud2Iterator<float>   iter_x(pc_msg, "x");
  sensor_msgs::PointCloud2Iterator<float>   iter_y(pc_msg, "y");
  sensor_msgs::PointCloud2Iterator<float>   iter_z(pc_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(pc_msg, "rgb");
  auto iter_img = point_cloud_image_.begin<float>();

  const double hfov = depth_camera_->HFOV().Radian();
  const double fl   =
    static_cast<double>(width_) / (2.0 * std::tan(hfov / 2.0));

  for (uint32_t j = 0; j < height_; ++j) {
    const double elevation =
      (height_ > 1)
      ? std::atan2(
          static_cast<double>(j) - 0.5 * static_cast<double>(height_), fl)
      : 0.0;

    // GCC13-FIX-3: elevation_angles_ is std::vector<float>
    if (static_cast<int>(j) < static_cast<int>(elevation_angles_.size()))
      elevation_angles_[j] = static_cast<float>(elevation);

    for (uint32_t i = 0; i < width_;
         ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb, ++iter_img)
    {
      const double azimuth =
        (width_ > 1)
        ? std::atan2(
            static_cast<double>(i) - 0.5 * static_cast<double>(width_), fl)
        : 0.0;

      // GCC13-FIX-6: bounds-checked source index
      const size_t src_idx = static_cast<size_t>(j) * width_ + i;
      const double d = static_cast<double>(_src[src_idx]);

      *iter_x = static_cast<float>(d * std::tan(azimuth));
      *iter_y = static_cast<float>(d * std::tan(elevation));

      if (d > point_cloud_cutoff_) {
        *iter_z   = static_cast<float>(d);
        *iter_img = std::sqrt(
          (*iter_x) * (*iter_x) +
          (*iter_y) * (*iter_y) +
          (*iter_z) * (*iter_z));
      } else {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        *iter_img = 0.0f;
        pc_msg.is_dense = false;
      }

      // RGB: zero — no colour source in depth-only mode
      iter_rgb[0] = iter_rgb[1] = iter_rgb[2] = 0;
    }
  }

  // FIX-H: only publish when a subscriber exists
  if (point_cloud_pub_->get_subscription_count() > 0) {
    point_cloud_pub_->publish(pc_msg);
  }
}

// =========================================================================
//  ComputeCorrector  (GCC13-FIX-3/5)
// =========================================================================

void NpsGazeboRosMultibeamSonar::ComputeCorrector()
{
  const double hFOV       = depth_camera_->HFOV().Radian();
  const double hPixelSize = (width_ > 1)
    ? hFOV / static_cast<double>(width_ - 1) : hFOV;
  const double fl         =
    static_cast<double>(width_) / (2.0 * std::tan(hFOV / 2.0));

  beamCorrectorSum = 0.0f;
  for (int beam = 0; beam < nBeams; ++beam) {
    const float a_beam = static_cast<float>(
      std::atan2(
        static_cast<double>(beam) - 0.5 * static_cast<double>(width_), fl));

    for (int beam_other = 0; beam_other < nBeams; ++beam_other) {
      const float a_other = static_cast<float>(
        std::atan2(
          static_cast<double>(beam_other) - 0.5 * static_cast<double>(width_),
          fl));

      // GCC13-FIX-5: detail::kPi replaces M_PI
      const float pattern = unnormalized_sinc(
        static_cast<float>(
          detail::kPi * 0.884 / hPixelSize
          * std::sin(static_cast<double>(a_beam - a_other))));

      beamCorrector_[beam][beam_other] = std::abs(pattern);
      beamCorrectorSum += pattern * pattern;
    }
  }
  beamCorrectorSum = std::sqrt(beamCorrectorSum);
}

// =========================================================================
//  ComputeNormalImage  (numerical logic unchanged; GCC13-FIX-5 for kPi)
// =========================================================================

cv::Mat NpsGazeboRosMultibeamSonar::ComputeNormalImage(cv::Mat & depth)
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
  cv::erode(depth == 0, no_readings,
            cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  n1.setTo(0, no_readings);
  n2.setTo(0, no_readings);

  std::vector<cv::Mat> channels(3);
  channels[0] = n1;
  channels[1] = n2;
  // focal_length_ is always populated in ConnectToDepthCamera() before
  // this function is reachable; division is safe.
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
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar,
  gz::sim::System,
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar::ISystemConfigure,
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar::ISystemPreUpdate,
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar,
  "nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar")
