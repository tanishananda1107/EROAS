/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_ROS_MULTIBEAM_SONAR_RAY_HH
#define GAZEBO_ROS_MULTIBEAM_SONAR_RAY_HH

// ROS 2 Core Headers
#include <rclcpp/rclcpp.hpp>

// ROS 2 Message Types
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// Marine Acoustic Message Definitions (ROS 2 Versions)
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>

// Gazebo Harmonic (Gz Sim 8) Headers
#include <gz/sim/System.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>

// Gazebo Math & Rendering Headers
#include <gz/math/Vector2.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

// OpenCV Core
#include <opencv2/core.hpp>

// Standard C++ Implementations
#include <string>
#include <complex>
#include <valarray>
#include <sstream>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <fstream>
#include <vector>

namespace gazebo_plugins
{
  typedef std::complex<float> Complex;
  typedef std::valarray<Complex> CArray;
  typedef std::valarray<CArray> CArray2D;

  typedef std::valarray<float> Array;
  typedef std::valarray<Array> Array2D;

  /// \brief Gazebo Harmonic System Plugin mapping a Multibeam Sonar Ray sensor over ROS 2
  class NpsGazeboRosMultibeamSonarRay : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor
    public: NpsGazeboRosMultibeamSonarRay();

    /// \brief Destructor
    public: virtual ~NpsGazeboRosMultibeamSonarRay();

    /// \brief Configures plugin data components on entity loading (Replaces legacy Load)
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    /// \brief Post-update hook triggered every calculation cycle step
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Native subscriber processing new incoming Gazebo GPU sensor frame messages
    private: void OnNewLaserFrame(const gz::msgs::LaserScan &_msg);

    // Image & Array structural layout sizes
    protected: unsigned int width, height, depth;
    protected: std::string format;

    // Gazebo Transport Node and Entity bindings
    private: gz::sim::Entity sensorEntity;
    private: gz::transport::Node gzNode;
    
    // ROS 2 Multithreading Thread-Execution infrastructure 
    private: rclcpp::Node::SharedPtr ros_node_;
    private: rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    private: std::thread thread_executor_;

    // ROS 2 Publishers
    private: rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr normal_image_pub_;
    private: rclcpp::Publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_image_raw_pub_;
    private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sonar_image_pub_;

    // ROS 2 Subscribers
    private: rclcpp::Subscriber<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_gpu_laser_point_cloud_sub_;

    // Connectivity logic structures
    private: std::mutex mutex_;
    private: int point_cloud_connect_count_;
    private: int sonar_image_connect_count_;
    private: void PointCloudConnect();
    private: void PointCloudDisconnect();
    private: void SonarImageConnect();
    private: void SonarImageDisconnect();

    /// \brief Calculations and image updates
    private: void UpdatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);
    private: void ComputeSonarImage();
    private: cv::Mat ComputeNormalImage(cv::Mat& depth_mat);
    private: double point_cloud_cutoff_;

    private: void ComputeCorrector();
    private: cv::Mat rand_image;

    /// \brief Parameters for sonar acoustic modeling
    private: double sonarFreq;
    private: double bandwidth;
    private: double soundSpeed;
    private: double maxDistance;
    private: double sourceLevel;
    private: bool constMu;
    private: double absorption;
    private: double attenuation;
    private: double verticalFOV;
    private: double mu;
    private: bool calculateReflectivity;
    private: cv::Mat reflectivityImage;
    private: std::vector<float> azimuth_angles;
    private: float* elevation_angles;
    private: float* rangeVector;
    private: float* window;
    private: float** beamCorrector;
    private: float beamCorrectorSum;
    private: int nFreq;
    private: double df;
    private: int nBeams;
    private: int nRays;
    private: int beamSkips;
    private: int raySkips;
    private: int ray_nAzimuthRays;
    private: int ray_nElevationRays;
    private: float plotScaler;
    private: float sensorGain;
    protected: bool debugFlag;

    // Local cached variables used to populate topics
    private: sensor_msgs::msg::PointCloud2 point_cloud_msg_;
    private: sensor_msgs::msg::Image normal_image_msg_;
    private: marine_acoustic_msgs::msg::ProjectedSonarImage sonar_image_raw_msg_;
    private: sensor_msgs::msg::Image sonar_image_msg_;
    private: sensor_msgs::msg::Image sonar_image_mono_msg_;
    private: cv::Mat point_cloud_image_;
    private: cv::Mat point_cloud_normal_image_;

    private: std::string point_cloud_topic_name_;
    private: std::string sonar_image_raw_topic_name_;
    private: std::string sonar_image_topic_name_;

    /// \brief Logger parameters
    protected: std::ofstream writeLog;
    protected: uint64_t writeCounter;
    protected: uint64_t writeNumber;
    protected: uint64_t writeInterval;
    protected: bool writeLogFlag;
  };

  ///////////////////////////////////////////
  inline double unnormalized_sinc(double t)
  {
    if (std::abs(t) < 1e-9)
    {
      return 1.0;
    }
    return std::sin(t) / t;
  }
}

#endif // GAZEBO_ROS_MULTIBEAM_SONAR_RAY_HH
