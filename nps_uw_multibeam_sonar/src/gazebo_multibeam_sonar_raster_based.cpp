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
 * ROS2 / Gazebo Harmonic (gz-sim 8) port — fully fixed
 *
 * Fixes applied vs the partial port:
 *  FIX-A  Added missing #include <std_msgs/msg/header.hpp>
 *  FIX-B  Timestamps now derived from gz::sim::UpdateInfo::simTime,
 *         not wall-clock rclcpp::Clock().now()
 *  FIX-C  rclcpp spinning moved to a dedicated std::thread (SingleThreadedExecutor)
 *         so it never blocks the gz-sim update thread
 *  FIX-D  depth_ and format_ properly assigned in ConnectToDepthCamera()
 *  FIX-E  CameraInfo message fully populated and published
 *  FIX-F  OnNewImageFrame ported for variational reflectivity updates
 *  FIX-G  Ogre SelectionBuffer replaced with gz::rendering::RayQuery
 *  FIX-H  Subscriber-count gating restored using get_subscription_count()
 *  FIX-I  ConnectToDepthCamera() protected against early calls before
 *         the gz-sensors Manager is ready
 */

#include <assert.h>
#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ament / ROS 2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// FIX-A: was missing
#include <std_msgs/msg/header.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

// Marine acoustic messages (ROS 2 package)
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Gazebo Harmonic (gz-sim 8) / gz-sensors
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
#include <gz/rendering/RayQuery.hh>      // FIX-G: replaces Ogre SelectionBuffer
#include <gz/rendering/Scene.hh>
#include <gz/sensors/DepthCameraSensor.hh>
#include <gz/sensors/Manager.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

// CUDA sonar calculation
#include <nps_uw_multibeam_sonar/sonar_calculation_cuda.cuh>
#include <nps_uw_multibeam_sonar/gazebo_multibeam_sonar_raster_based.hh>

namespace nps_uw_multibeam_sonar
{

//////////////////////////////////////////////////
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
  // ---- internal helpers ------------------------------------------------
  void ConnectToDepthCamera();

  void OnNewDepthFrame(
    const float * _image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string & _format);

  // FIX-F: ported from ROS 1 for variational reflectivity
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

  // FIX-G: replaces SelectionBuffer Ogre picking
  void UpdateReflectivityImage();

  // FIX-C: ROS 2 executor on a dedicated thread
  rclcpp::Node::SharedPtr                          ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr ros_executor_;
  std::thread                                      ros_thread_;

  // ---- publishers ------------------------------------------------------
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr         depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              normal_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        point_cloud_pub_;
  rclcpp::Publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr
                                                                     sonar_image_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              sonar_image_pub_;

  // ---- gz-sim / gz-sensors objects ------------------------------------
  gz::sim::Entity                          sensor_entity_{gz::sim::kNullEntity};
  gz::sensors::DepthCameraSensor *         depth_sensor_{nullptr};
  gz::rendering::DepthCameraPtr            depth_camera_;
  gz::rendering::ScenePtr                  scene_;

  // FIX-G: ray query object for per-pixel object identification
  gz::rendering::RayQueryPtr               ray_query_;

  gz::common::ConnectionPtr new_depth_frame_conn_;
  gz::common::ConnectionPtr new_image_frame_conn_;   // FIX-F

  // ---- sensor dimensions -----------------------------------------------
  unsigned int width_{0}, height_{0}, depth_{0};   // FIX-D: depth_ now assigned
  std::string  format_;                             // FIX-D: format_ now assigned
  std::string  frame_name_{"world"};

  // ---- topic names -------------------------------------------------------
  std::string depth_image_topic_name_;
  std::string depth_image_camera_info_topic_name_;
  std::string point_cloud_topic_name_;
  std::string sonar_image_raw_topic_name_;
  std::string sonar_image_topic_name_;
  double      point_cloud_cutoff_{0.01};

  // ---- sonar parameters --------------------------------------------------
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

  float * rangeVector{nullptr};
  float * elevation_angles{nullptr};
  float * window{nullptr};
  float **beamCorrector{nullptr};
  float   beamCorrectorSum{0.0f};

  cv::Mat rand_image_;
  cv::Mat point_cloud_image_;
  cv::Mat reflectivityImage_;
  bool    calculateReflectivity{false};

  // variational reflectivity
  std::string              reflectivityDatabaseFileName_{"variationalReflectivityDatabase.csv"};
  std::string              customTagDatabaseFileName_{"customSDFTagDatabase.csv"};
  std::string              reflectivityDatabaseFilePath_;
  std::string              customTagDatabaseFilePath_;
  std::vector<std::string> objectNames_;
  std::vector<float>       reflectivities_;
  float                    biofouling_rating_coeff_{1.0f};
  float                    roughness_coeff_{1.0f};

  // fiducial / selection
  std::set<std::string> fiducials_;
  bool                  detectAll_{false};

  // depth tracking for variational reflectivity (FIX-F)
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

  // FIX-B: sim time cache updated every PostUpdate
  rclcpp::Time  last_sim_time_{0, 0, RCL_ROS_TIME};
  bool          camera_connected_{false};

  // guard for ConnectToDepthCamera retry (FIX-I)
  int           connect_retry_count_{0};
  static constexpr int kConnectRetryMax{10};

  // thread safety
  std::mutex lock_;
};

//////////////////////////////////////////////////
NpsGazeboRosMultibeamSonar::NpsGazeboRosMultibeamSonar()
{
}

//////////////////////////////////////////////////
NpsGazeboRosMultibeamSonar::~NpsGazeboRosMultibeamSonar()
{
  // FIX-C: stop executor thread cleanly
  if (ros_executor_) {
    ros_executor_->cancel();
  }
  if (ros_thread_.joinable()) {
    ros_thread_.join();
  }

  new_depth_frame_conn_.reset();
  new_image_frame_conn_.reset();

  if (rangeVector)       { delete[] rangeVector;       rangeVector = nullptr; }
  if (elevation_angles)  { delete[] elevation_angles;  elevation_angles = nullptr; }
  if (window)            { delete[] window;             window = nullptr; }
  if (beamCorrector) {
    for (int i = 0; i < nBeams; ++i) delete[] beamCorrector[i];
    delete[] beamCorrector;
    beamCorrector = nullptr;
  }
  if (writeLog_.is_open()) writeLog_.close();
}

//////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonar::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  sensor_entity_ = _entity;

  // ---- ROS 2 node -------------------------------------------------------
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  ros_node_ = std::make_shared<rclcpp::Node>("nps_multibeam_sonar");

  // FIX-C: create executor and spin on dedicated thread
  ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  ros_executor_->add_node(ros_node_);
  ros_thread_ = std::thread([this]() {
    ros_executor_->spin();
  });

  // ---- SDF helpers -------------------------------------------------------
  auto get_str = [&](const std::string & tag, const std::string & def) -> std::string {
    if (_sdf->HasElement(tag)) return _sdf->Get<std::string>(tag);
    return def;
  };
  auto get_bool = [&](const std::string & tag, bool def) -> bool {
    if (_sdf->HasElement(tag)) return _sdf->Get<bool>(tag);
    return def;
  };
  auto get_double = [&](const std::string & tag, double def) -> double {
    if (_sdf->HasElement(tag)) return _sdf->Get<double>(tag);
    return def;
  };
  auto get_int = [&](const std::string & tag, int def) -> int {
    if (_sdf->HasElement(tag)) return _sdf->Get<int>(tag);
    return def;
  };
  auto get_float = [&](const std::string & tag, float def) -> float {
    if (_sdf->HasElement(tag)) return _sdf->Get<float>(tag);
    return def;
  };

  frame_name_                         = get_str("frameName",                    "world");
  depth_image_topic_name_             = get_str("depthImageTopicName",          "depth/image_raw");
  depth_image_camera_info_topic_name_ = get_str("depthImageCameraInfoTopicName","depth/camera_info");
  point_cloud_topic_name_             = get_str("pointCloudTopicName",          "points");
  sonar_image_raw_topic_name_         = get_str("sonarImageRawTopicName",       "sonar_image_raw");
  sonar_image_topic_name_             = get_str("sonarImageTopicName",          "sonar_image");
  point_cloud_cutoff_                 = get_double("pointCloudCutoff",          0.01);

  verticalFOV               = get_double("verticalFOV",            10.0);
  sonarFreq                 = get_double("sonarFreq",              900e3);
  bandwidth                 = get_double("bandwidth",              29.5e6);
  soundSpeed                = get_double("soundSpeed",             1500.0);
  maxDistance               = get_double("maxDistance",            60.0);
  sourceLevel               = get_double("sourceLevel",            220.0);
  constMu                   = get_bool("constantReflectivity",     true);
  artificialVehicleVibration = get_bool("artificialVehicleVibration", false);
  customTag                 = get_bool("customSDFTagReflectivity", false);
  raySkips                  = get_int("raySkips",                  10);
  plotScaler                = get_float("plotScaler",              10.0f);
  sensorGain                = get_float("sensorGain",              0.02f);
  writeLogFlag_             = get_bool("writeLog",                 false);
  writeInterval_            = get_int("writeFrameInterval",        10);
  debugFlag_                = get_bool("debugFlag",                false);
  if (raySkips == 0) raySkips = 1;

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
      std::string pkg_share =
        ament_index_cpp::get_package_share_directory("nps_uw_multibeam_sonar");
      reflectivityDatabaseFilePath_ = pkg_share + "/worlds/" + reflectivityDatabaseFileName_;
      customTagDatabaseFilePath_    = pkg_share + "/worlds/" + customTagDatabaseFileName_;
    } catch (const std::exception & e) {
      RCLCPP_WARN(ros_node_->get_logger(),
        "Could not find nps_uw_multibeam_sonar package: %s", e.what());
    }

    // Read CSV database
    std::string csvPath = customTag ? customTagDatabaseFilePath_ : reflectivityDatabaseFilePath_;
    std::ifstream csvFile(csvPath);
    std::string line;
    // skip 3 header lines
    for (int h = 0; h < 3 && std::getline(csvFile, line); ++h) {}
    while (std::getline(csvFile, line)) {
      if (line.empty()) continue;
      std::istringstream iss(line);
      std::string token;
      std::vector<std::string> row;
      while (std::getline(iss, token, ',')) row.push_back(token);
      if (row.size() < 2) continue;
      objectNames_.push_back(row[0]);
      reflectivities_.push_back(std::stof(row[1]));
    }

    if (customTag) {
      for (size_t k = 0; k < objectNames_.size(); ++k) {
        if (objectNames_[k] == "biofouling_rating") biofouling_rating_coeff_ = reflectivities_[k];
        if (objectNames_[k] == "roughness")         roughness_coeff_         = reflectivities_[k];
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
      "No fiducials specified – all models will be tracked.");
    detectAll_ = true;
  }

  // ---- Transmission attenuation ----------------------------------------
  absorption  = 0.0354;
  attenuation = absorption * std::log(10.0) / 20.0;

  // ---- ROS 2 publishers -------------------------------------------------
  auto qos = rclcpp::SensorDataQoS();
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
    struct stat buffer;
    std::string logfilename("/tmp/SonarRawData_000001.csv");
    if (stat(logfilename.c_str(), &buffer) == 0)
      system("rm /tmp/SonarRawData*.csv");
    RCLCPP_INFO(ros_node_->get_logger(),
      "Raw data written to /tmp/SonarRawData_{number}.csv every %d frames",
      writeInterval_);
  }

  RCLCPP_INFO(ros_node_->get_logger(),
    "NpsGazeboRosMultibeamSonar configured (camera not yet connected).");
}

//////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonar::PreUpdate(
  const gz::sim::UpdateInfo & /*_info*/,
  gz::sim::EntityComponentManager & /*_ecm*/)
{
  // nothing needed in pre-update for this plugin
}

//////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonar::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & /*_ecm*/)
{
  // FIX-B: convert gz nanosecond duration to rclcpp::Time
  last_sim_time_ = rclcpp::Time(
    static_cast<int64_t>(_info.simTime.count()), RCL_ROS_TIME);

  // FIX-I: retry with a back-off counter so we don't hammer the sensor
  // manager before rendering is initialised
  if (!camera_connected_) {
    if (connect_retry_count_ < kConnectRetryMax) {
      ++connect_retry_count_;
    } else {
      ConnectToDepthCamera();
      connect_retry_count_ = 0;   // reset so we retry again if it still fails
    }
    return;
  }

  // FIX-H: only keep sensor active when someone is subscribed
  bool anyone_subscribed =
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

//////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonar::ConnectToDepthCamera()
{
  auto * sensor_mgr = gz::sensors::Manager::Instance();
  if (!sensor_mgr) return;

  depth_sensor_ = sensor_mgr->Sensor<gz::sensors::DepthCameraSensor>(sensor_entity_);
  if (!depth_sensor_) return;

  depth_camera_ = depth_sensor_->DepthCamera();
  if (!depth_camera_) return;

  scene_ = depth_camera_->Scene();
  if (!scene_) return;

  width_  = depth_camera_->ImageWidth();
  height_ = depth_camera_->ImageHeight();

  // FIX-D: assign depth_ and format_ that were missing in the partial port
  depth_  = depth_camera_->ImageDepth();
  format_ = depth_camera_->ImageFormat();

  nBeams             = static_cast<int>(width_);
  nRays              = static_cast<int>(height_);
  ray_nElevationRays = static_cast<int>(height_);
  ray_nAzimuthRays   = 1;

  double hfov = depth_camera_->HFOV().Radian();
  focal_length_ = static_cast<double>(width_) / (2.0 * std::tan(hfov / 2.0));

  // ---- Range vector -----------------------------------------------------
  const float max_T   = static_cast<float>(maxDistance) * 2.0f /
                        static_cast<float>(soundSpeed);
  float       delta_f = 1.0f / max_T;
  const float delta_t = 1.0f / static_cast<float>(bandwidth);
  nFreq   = static_cast<int>(std::ceil(bandwidth / delta_f));
  delta_f = static_cast<float>(bandwidth) / static_cast<float>(nFreq);

  rangeVector = new float[nFreq];
  for (int i = 0; i < nFreq; ++i)
    rangeVector[i] = delta_t * static_cast<float>(i) *
                     static_cast<float>(soundSpeed) / 2.0f;

  // ---- Elevation angles -------------------------------------------------
  elevation_angles = new float[nRays];

  // ---- Hamming window ---------------------------------------------------
  window = new float[nFreq];
  float windowSum = 0.0f;
  for (int f = 0; f < nFreq; ++f) {
    window[f]  = 0.54f - 0.46f * std::cos(2.0f * M_PI * (f + 1) / nFreq);
    windowSum += window[f] * window[f];
  }
  for (int f = 0; f < nFreq; ++f) window[f] /= std::sqrt(windowSum);

  // ---- Beam corrector ---------------------------------------------------
  beamCorrector = new float *[nBeams];
  for (int i = 0; i < nBeams; ++i) beamCorrector[i] = new float[nBeams];
  beamCorrectorSum = 0.0f;

  // ---- Random image for noise ------------------------------------------
  rand_image_ = cv::Mat(height_, width_, CV_32FC2);
  uint64_t randN = static_cast<uint64_t>(std::rand());
  cv::theRNG().state = randN;
  cv::RNG rng = cv::theRNG();
  rng.fill(rand_image_, cv::RNG::NORMAL, 0.f, 1.0f);

  // FIX-G: initialise RayQuery for per-pixel object identification
  ray_query_ = scene_->CreateRayQuery();

  // ---- Connect callbacks -----------------------------------------------
  new_depth_frame_conn_ = depth_camera_->ConnectNewDepthFrame(
    std::bind(
      &NpsGazeboRosMultibeamSonar::OnNewDepthFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  // FIX-F: connect the RGB camera frame for reflectivity updates
  // The gz-sensors DepthCameraSensor exposes the RGB camera via Camera()
  auto rgb_camera = depth_sensor_->Camera();
  if (rgb_camera) {
    new_image_frame_conn_ = rgb_camera->ConnectNewImageFrame(
      std::bind(
        &NpsGazeboRosMultibeamSonar::OnNewImageFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));
  } else {
    RCLCPP_WARN(ros_node_->get_logger(),
      "No RGB camera on depth sensor — variational reflectivity disabled.");
  }

  camera_connected_ = true;

  RCLCPP_INFO(ros_node_->get_logger(),
    "==================================================\n"
    "============   SONAR PLUGIN LOADED   =============\n"
    "==================================================\n"
    "============      RASTER VERSION     =============\n"
    "==================================================\n"
    "Maximum view range  [m] = %.1f\n"
    "# of Beams = %d\n"
    "# of Rays / Beam (Elev, Az) = (%d, %d)\n"
    "# of Time data / Beam = %d",
    maxDistance, nBeams, ray_nElevationRays, ray_nAzimuthRays, nFreq);
}

//////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonar::OnNewDepthFrame(
  const float * _image,
  unsigned int _width, unsigned int _height,
  unsigned int /*_depth*/, const std::string & /*_format*/)
{
  if (!camera_connected_ || _height == 0 || _width == 0) return;

  ComputePointCloud(_image);

  // FIX-H: only compute sonar image when a subscriber exists
  if (sonar_image_raw_pub_->get_subscription_count() > 0 ||
      sonar_image_pub_->get_subscription_count()     > 0 ||
      depth_image_pub_->get_subscription_count()     > 0 ||
      normal_image_pub_->get_subscription_count()    > 0)
  {
    ComputeSonarImage(_image);
  }
}

//////////////////////////////////////////////////
// FIX-F: ported from ROS 1 OnNewImageFrame — drives variational reflectivity
void NpsGazeboRosMultibeamSonar::OnNewImageFrame(
  const unsigned char * /*_image*/,
  unsigned int /*_width*/, unsigned int /*_height*/,
  unsigned int /*_depth*/, const std::string & /*_format*/)
{
  if (!camera_connected_ || !depth_camera_) return;

  // Compute maxDepth from the current point cloud image
  double minVal;
  {
    std::lock_guard<std::mutex> guard(lock_);
    if (point_cloud_image_.empty()) return;
    cv::minMaxLoc(point_cloud_image_, &minVal, &maxDepth_);
  }

  // Trigger reflectivity update only when depth has stabilised
  if (maxDepth_ == maxDepth_before_ &&
      maxDepth_ == maxDepth_beforebefore_ &&
      !calculateReflectivity &&
      maxDepth_ != maxDepth_prev_)
  {
    calculateReflectivity = true;
    maxDepth_prev_ = maxDepth_;

    // Regenerate random noise image
    uint64_t randN = static_cast<uint64_t>(std::rand());
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

  if (!constMu && calculateReflectivity) {
    UpdateReflectivityImage();
  }
}

//////////////////////////////////////////////////
// FIX-G: variational reflectivity using gz::rendering::RayQuery
//        (replaces the Ogre SelectionBuffer that no longer exists in gz-rendering 8)
void NpsGazeboRosMultibeamSonar::UpdateReflectivityImage()
{
  if (!ray_query_ || !scene_) return;

  if (detectAll_) PopulateFiducials();

  cv::Mat reflectivity_image(width_, height_, CV_32FC1, cv::Scalar(mu));

  double hfov = depth_camera_->HFOV().Radian();
  double vfov = depth_camera_->VFOV().Radian();
  double fl_h = static_cast<double>(width_)  / (2.0 * std::tan(hfov / 2.0));
  double fl_v = static_cast<double>(height_) / (2.0 * std::tan(vfov / 2.0));

  for (int i = 0; i < static_cast<int>(reflectivity_image.rows); ++i) {
    for (int j = 0; j < static_cast<int>(reflectivity_image.cols); j += raySkips) {

      // Reconstruct view-space ray direction from pixel coordinates
      double az_rad = std::atan2(static_cast<double>(j) -
                                 0.5 * static_cast<double>(width_),  fl_h);
      double el_rad = std::atan2(static_cast<double>(i) -
                                 0.5 * static_cast<double>(height_), fl_v);

      gz::math::Vector3d ray_dir(
        std::cos(el_rad) * std::cos(az_rad),
        std::cos(el_rad) * std::sin(az_rad),
        std::sin(el_rad));

      gz::rendering::RayQueryPoint result;
      ray_query_->SetOrigin(depth_camera_->WorldPosition());
      ray_query_->SetDirection(ray_dir);
      result = ray_query_->ClosestPoint();

      if (result.objectId == gz::rendering::kNullEntity) continue;

      // Find the visual name from the scene
      auto vis = scene_->VisualById(result.objectId);
      if (!vis) continue;

      std::string vis_name = vis->Name();

      if (!customTag) {
        // Match by object name against the database
        for (size_t k = 0; k < objectNames_.size(); ++k) {
          if (vis_name == objectNames_[k]) {
            reflectivity_image.at<float>(j, i) = reflectivities_[k];
          }
        }
      } else {
        // Use surface property SDF tags embedded in the visual's user data
        // gz-sim 8 stores per-model SDF as user data on the visual
        int    biofoulingRating = 0;
        double roughness        = 0.0;
        std::string material    = "default";

        // Attempt to read gz-sim custom user data (set via SDF plugin tags)
        auto * user_data = vis->UserData("surface_props:biofouling_rating");
        if (user_data) biofoulingRating = std::get<int>(*user_data);

        auto * user_data_r = vis->UserData("surface_props:roughness");
        if (user_data_r) roughness = std::get<double>(*user_data_r);

        auto * user_data_m = vis->UserData("surface_props:material");
        if (user_data_m) material = std::get<std::string>(*user_data_m);

        for (size_t k = 0; k < objectNames_.size(); ++k) {
          if (material == objectNames_[k]) {
            reflectivity_image.at<float>(j, i) =
              reflectivities_[k]
              * (1.0 / (roughness + 1.0))        / roughness_coeff_
              * (1.0 / (biofoulingRating + 1.0)) / biofouling_rating_coeff_;
          }
        }
      }
    }
  }

  std::lock_guard<std::mutex> guard(lock_);
  reflectivityImage_ = reflectivity_image;
}

//////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonar::PopulateFiducials()
{
  fiducials_.clear();
  if (!scene_) return;
  for (unsigned int i = 0; i < scene_->VisualCount(); ++i) {
    auto vis = scene_->VisualByIndex(i);
    if (vis) fiducials_.insert(vis->Name());
  }
}

//////////////////////////////////////////////////
// FIX-E: populate and publish CameraInfo properly
void NpsGazeboRosMultibeamSonar::PublishCameraInfo(const rclcpp::Time & stamp)
{
  if (depth_info_pub_->get_subscription_count() == 0) return;
  if (!depth_camera_) return;

  sensor_msgs::msg::CameraInfo info_msg;
  info_msg.header.stamp    = stamp;
  info_msg.header.frame_id = frame_name_;
  info_msg.width           = width_;
  info_msg.height          = height_;
  info_msg.distortion_model = "plumb_bob";

  // Zero distortion (depth camera in simulation has none)
  info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};

  double hfov = depth_camera_->HFOV().Radian();
  double vfov = depth_camera_->VFOV().Radian();
  double fx    = static_cast<double>(width_)  / (2.0 * std::tan(hfov / 2.0));
  double fy    = static_cast<double>(height_) / (2.0 * std::tan(vfov / 2.0));
  double cx    = static_cast<double>(width_)  / 2.0;
  double cy    = static_cast<double>(height_) / 2.0;

  // 3x3 intrinsic matrix K (row-major)
  info_msg.k = {fx,  0.0, cx,
                0.0, fy,  cy,
                0.0, 0.0, 1.0};

  // Rectification matrix R = identity
  info_msg.r = {1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0};

  // 3x4 projection matrix P
  info_msg.p = {fx,  0.0, cx,  0.0,
                0.0, fy,  cy,  0.0,
                0.0, 0.0, 1.0, 0.0};

  depth_info_pub_->publish(info_msg);
}

//////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonar::ComputeSonarImage(const float * /*_src*/)
{
  std::lock_guard<std::mutex> guard(lock_);

  cv::Mat depth_image  = point_cloud_image_;
  cv::Mat normal_image = ComputeNormalImage(depth_image);

  double vFOV       = depth_camera_->VFOV().Radian();
  double hFOV       = depth_camera_->HFOV().Radian();
  double vPixelSize = vFOV / height_;
  double hPixelSize = hFOV / width_;

  if (beamCorrectorSum == 0.0f) ComputeCorrector();

  if (reflectivityImage_.rows == 0)
    reflectivityImage_ = cv::Mat(width_, height_, CV_32FC1, cv::Scalar(mu));

  if (artificialVehicleVibration) {
    uint64_t randN = static_cast<uint64_t>(std::rand());
    cv::theRNG().state = randN;
    cv::RNG rng = cv::theRNG();
    rng.fill(rand_image_, cv::RNG::NORMAL, 0.f, 1.f);
  }

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
    reflectivityImage_,
    attenuation,
    window,
    beamCorrector,
    beamCorrectorSum,
    debugFlag_);

  auto stop     = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  if (debugFlag_)
    RCLCPP_INFO(ros_node_->get_logger(),
      "GPU Sonar Frame Calc Time %ld/100 [s]", duration.count() / 10000);

  // ---- CSV log ----------------------------------------------------------
  if (writeLogFlag_) {
    writeCounter_++;
    if (writeCounter_ == 1 || writeCounter_ % writeInterval_ == 0) {
      // FIX-B: use simulation time, not wall clock
      double sim_time = static_cast<double>(last_sim_time_.nanoseconds()) * 1e-9;
      std::ostringstream filename;
      filename << "/tmp/SonarRawData_"
               << std::setw(6) << std::setfill('0') << writeNumber_ << ".csv";
      writeLog_.open(filename.str(), std::ios_base::app);
      writeLog_ << "# Raw Sonar Data Log (Row: beams, Col: time series data)\n"
                << "# First column is range vector\n"
                << "#  nBeams : " << nBeams << "\n"
                << "# Simulation time : " << sim_time << "\n";
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
      writeNumber_++;
    }
  }

  // FIX-B: use sim time for all message stamps
  std_msgs::msg::Header header;
  header.frame_id = frame_name_;
  header.stamp    = last_sim_time_;

  // ---- Azimuth angles --------------------------------------------------
  double fl = static_cast<double>(width_) / (2.0 * std::tan(hFOV / 2.0));
  std::vector<float> azimuth_angles;
  azimuth_angles.reserve(nBeams);
  for (int beam = 0; beam < nBeams; ++beam)
    azimuth_angles.push_back(static_cast<float>(
      std::atan2(static_cast<double>(beam) - 0.5 * static_cast<double>(width_), fl)));

  // ---- sonar_image_raw (ProjectedSonarImage) ---------------------------
  marine_acoustic_msgs::msg::ProjectedSonarImage sonar_raw_msg;
  sonar_raw_msg.header = header;

  marine_acoustic_msgs::msg::PingInfo ping_info;
  ping_info.frequency   = static_cast<float>(sonarFreq);
  ping_info.sound_speed = static_cast<float>(soundSpeed);
  for (int beam = 0; beam < nBeams; ++beam) {
    ping_info.rx_beamwidths.push_back(static_cast<float>(
      std::abs(
        std::atan2(static_cast<double>(beam) - 1.0 * static_cast<double>(width_), fl)
        - std::atan2(static_cast<double>(beam), fl))));
    ping_info.tx_beamwidths.push_back(static_cast<float>(vFOV));
  }
  sonar_raw_msg.ping_info = ping_info;

  std::vector<geometry_msgs::msg::Vector3> beam_dirs;
  beam_dirs.reserve(nBeams);
  for (int beam = 0; beam < nBeams; ++beam) {
    geometry_msgs::msg::Vector3 d;
    d.x = std::cos(azimuth_angles[beam]);
    d.y = std::sin(azimuth_angles[beam]);
    d.z = 0.0;
    beam_dirs.push_back(d);
  }
  sonar_raw_msg.beam_directions = beam_dirs;

  std::vector<float> ranges;
  ranges.reserve(nFreq);
  for (int i = 0; i < nFreq; ++i) ranges.push_back(rangeVector[i]);
  sonar_raw_msg.ranges = ranges;

  marine_acoustic_msgs::msg::SonarImageData sonar_img_data;
  sonar_img_data.is_bigendian = false;
  sonar_img_data.dtype        = 0;  // DTYPE_UINT8
  sonar_img_data.beam_count   = nBeams;

  std::vector<uint8_t> intensities;
  intensities.reserve(static_cast<size_t>(nFreq * nBeams));
  for (int f = 0; f < nFreq; ++f) {
    for (int beam = 0; beam < nBeams; ++beam) {
      const int beam_idx = nBeams - beam - 1;
      int intensity = static_cast<int>(sensorGain * std::abs(P_Beams[beam_idx][f]));
      intensities.push_back(static_cast<uint8_t>(std::min(255, intensity)));
    }
  }
  sonar_img_data.data      = intensities;
  sonar_raw_msg.image      = sonar_img_data;
  sonar_image_raw_pub_->publish(sonar_raw_msg);

  // ---- Visual sonar image (polar plot) ---------------------------------
  cv::Mat Intensity_image = cv::Mat::zeros(cv::Size(nBeams, nFreq), CV_8UC1);
  const float rangeMax      = static_cast<float>(maxDistance);
  const float rangeRes      = (nFreq > 1) ? (ranges[1] - ranges[0]) : 1.0f;
  const int   nEffRanges    = static_cast<int>(std::ceil(rangeMax / rangeRes));
  const unsigned int radius = static_cast<unsigned int>(Intensity_image.size().height);
  const cv::Point    origin(Intensity_image.size().width / 2,
                             Intensity_image.size().height);
  const float binThickness  = 2.0f * std::ceil(static_cast<float>(radius) / nEffRanges);

  struct BearingEntry { float begin, center, end; };
  std::vector<BearingEntry> angles;
  angles.reserve(nBeams);
  for (int b = 0; b < nBeams; ++b) {
    float center = azimuth_angles[b];
    float begin = 0.0f, end = 0.0f;
    if (b == 0) {
      end   = (azimuth_angles[b + 1] + center) / 2.0f;
      begin = 2.0f * center - end;
    } else if (b == nBeams - 1) {
      begin = angles[b - 1].end;
      end   = 2.0f * center - begin;
    } else {
      begin = angles[b - 1].end;
      end   = (azimuth_angles[b + 1] + center) / 2.0f;
    }
    angles.push_back({begin, center, end});
  }

  const float ThetaShift = 1.5f * static_cast<float>(M_PI);
  for (int r = 0; r < static_cast<int>(ranges.size()); ++r) {
    if (ranges[r] > rangeMax) continue;
    for (int b = 0; b < nBeams; ++b) {
      float range     = ranges[r];
      int   intensity = static_cast<int>(
        std::floor(10.0 * std::log(std::abs(P_Beams[nBeams - 1 - b][r]))));
      float begin_a   = angles[b].begin + ThetaShift;
      float end_a     = angles[b].end   + ThetaShift;
      float rad       = static_cast<float>(radius) * range / rangeMax;
      cv::ellipse(Intensity_image, origin,
                  cv::Size(static_cast<int>(rad), static_cast<int>(rad)), 0,
                  static_cast<double>(begin_a) * 180.0 / M_PI,
                  static_cast<double>(end_a)   * 180.0 / M_PI,
                  intensity, static_cast<int>(binThickness));
    }
  }

  cv::normalize(Intensity_image, Intensity_image,
                -255 + plotScaler / 10.0f * 255.0f, 255.0f, cv::NORM_MINMAX);
  cv::Mat Intensity_image_color;
  cv::applyColorMap(Intensity_image, Intensity_image_color, cv::COLORMAP_HOT);

  cv_bridge::CvImage img_bridge(header,
                                 sensor_msgs::image_encodings::BGR8,
                                 Intensity_image_color);
  sensor_msgs::msg::Image sonar_img_msg;
  img_bridge.toImageMsg(sonar_img_msg);
  sonar_image_pub_->publish(sonar_img_msg);

  // ---- Depth image -------------------------------------------------------
  cv_bridge::CvImage depth_bridge(header,
                                   sensor_msgs::image_encodings::TYPE_32FC1,
                                   depth_image);
  sensor_msgs::msg::Image depth_img_msg;
  depth_bridge.toImageMsg(depth_img_msg);
  depth_image_pub_->publish(depth_img_msg);

  // ---- Normal image -----------------------------------------------------
  cv::Mat normal_image8;
  normal_image.convertTo(normal_image8, CV_8UC3, 255.0);
  cv_bridge::CvImage normal_bridge(header,
                                    sensor_msgs::image_encodings::RGB8,
                                    normal_image8);
  sensor_msgs::msg::Image normal_img_msg;
  normal_bridge.toImageMsg(normal_img_msg);
  normal_image_pub_->publish(normal_img_msg);

  // FIX-E: publish camera info with the same stamp
  PublishCameraInfo(last_sim_time_);
}

//////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonar::ComputePointCloud(const float * _src)
{
  std::lock_guard<std::mutex> guard(lock_);

  // FIX-B: use simulation time
  std_msgs::msg::Header header;
  header.frame_id = frame_name_;
  header.stamp    = last_sim_time_;

  sensor_msgs::msg::PointCloud2 point_cloud_msg;
  point_cloud_msg.header   = header;
  point_cloud_msg.width    = width_;
  point_cloud_msg.height   = height_;
  point_cloud_msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(height_ * width_);
  point_cloud_msg.row_step = point_cloud_msg.point_step * width_;

  point_cloud_image_.create(height_, width_, CV_32FC1);

  sensor_msgs::PointCloud2Iterator<float>   iter_x(point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float>   iter_y(point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float>   iter_z(point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg, "rgb");
  cv::MatIterator_<float> iter_img = point_cloud_image_.begin<float>();

  const float * toCopy = _src;
  int index = 0;

  double hfov = depth_camera_->HFOV().Radian();
  double fl   = static_cast<double>(width_) / (2.0 * std::tan(hfov / 2.0));

  for (uint32_t j = 0; j < height_; ++j) {
    double elevation = (height_ > 1)
      ? std::atan2(static_cast<double>(j) - 0.5 * static_cast<double>(height_), fl)
      : 0.0;
    elevation_angles[j] = static_cast<float>(elevation);

    for (uint32_t i = 0; i < width_; ++i,
         ++iter_x, ++iter_y, ++iter_z, ++iter_rgb, ++iter_img)
    {
      double azimuth = (width_ > 1)
        ? std::atan2(static_cast<double>(i) - 0.5 * static_cast<double>(width_), fl)
        : 0.0;

      double d = static_cast<double>(toCopy[index++]);

      *iter_x = static_cast<float>(d * std::tan(azimuth));
      *iter_y = static_cast<float>(d * std::tan(elevation));

      if (d > point_cloud_cutoff_) {
        *iter_z   = static_cast<float>(d);
        *iter_img = std::sqrt((*iter_x) * (*iter_x)
                            + (*iter_y) * (*iter_y)
                            + (*iter_z) * (*iter_z));
      } else {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        *iter_img = 0.0f;
        point_cloud_msg.is_dense = false;
      }

      // RGB: zeroed — no colour camera source connected in depth-only mode
      iter_rgb[0] = iter_rgb[1] = iter_rgb[2] = 0;
    }
  }

  // FIX-H: only publish when someone is listening
  if (point_cloud_pub_->get_subscription_count() > 0) {
    point_cloud_pub_->publish(point_cloud_msg);
  }
}

//////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonar::ComputeCorrector()
{
  double hFOV       = depth_camera_->HFOV().Radian();
  double hPixelSize = hFOV / width_;
  double fl         = static_cast<double>(width_) / (2.0 * std::tan(hFOV / 2.0));

  beamCorrectorSum = 0.0f;
  for (int beam = 0; beam < nBeams; ++beam) {
    float a_beam = static_cast<float>(
      std::atan2(static_cast<double>(beam) - 0.5 * static_cast<double>(width_), fl));
    for (int beam_other = 0; beam_other < nBeams; ++beam_other) {
      float a_other = static_cast<float>(
        std::atan2(static_cast<double>(beam_other) -
                   0.5 * static_cast<double>(width_), fl));
      float azimuthBeamPattern = static_cast<float>(
        unnormalized_sinc(M_PI * 0.884 / hPixelSize
                          * std::sin(static_cast<double>(a_beam - a_other))));
      beamCorrector[beam][beam_other] = std::abs(azimuthBeamPattern);
      beamCorrectorSum += azimuthBeamPattern * azimuthBeamPattern;
    }
  }
  beamCorrectorSum = std::sqrt(beamCorrectorSum);
}

//////////////////////////////////////////////////
cv::Mat NpsGazeboRosMultibeamSonar::ComputeNormalImage(cv::Mat & depth)
{
  cv::Mat_<float> f1 = (cv::Mat_<float>(3, 3) <<  1,  2,  1,
                                                    0,  0,  0,
                                                   -1, -2, -1) / 8.0f;
  cv::Mat_<float> f2 = (cv::Mat_<float>(3, 3) <<  1,  0, -1,
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
  // focal_length_ is always assigned in ConnectToDepthCamera() before this
  // is called, so the division is safe (no divide-by-zero risk)
  images[2] = (1.0 / focal_length_) * depth;

  cv::Mat normal_image;
  cv::merge(images, normal_image);

  for (int i = 0; i < normal_image.rows; ++i) {
    for (int j = 0; j < normal_image.cols; ++j) {
      cv::Vec3f & n = normal_image.at<cv::Vec3f>(i, j);
      n = cv::normalize(n);
    }
  }
  return normal_image;
}

}  // namespace nps_uw_multibeam_sonar

// Register as gz-sim plugin
GZ_ADD_PLUGIN(
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar,
  gz::sim::System,
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar::ISystemConfigure,
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar::ISystemPreUpdate,
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar,
  "nps_uw_multibeam_sonar::NpsGazeboRosMultibeamSonar")
