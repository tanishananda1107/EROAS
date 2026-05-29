// Copyright (C) 2014 Open Source Robotics Foundation
// Modifications Copyright 2018 Nils Bore (nbore@kth.se)
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <mutex>
#include <random>
#include <string>
#include <vector>
#include <memory>

// Gazebo Harmonic
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sensors/DepthCameraSensor.hh>
#include <gz/rendering/DepthCamera.hh>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// OpenCV
#include <opencv2/core/core.hpp>

namespace gz::sim::systems
{

class GazeboRosImageSonar :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  GazeboRosImageSonar();
  ~GazeboRosImageSonar() override;

  // gz-sim8 System interface
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override;

private:
  // ── Sensor frame callbacks ─────────────────────────────────────────────────
  void OnNewDepthFrame(
    const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);

  void OnNewRGBPointCloud(
    const float *_pcd,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);

  void OnNewImageFrame(
    const unsigned char *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);

  // ── Point cloud helpers ────────────────────────────────────────────────────
  void FillPointCloud(const float *_src);

  bool FillPointCloudHelper(
    sensor_msgs::msg::PointCloud2 &cloud_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void *data_arg);

  bool FillDepthImageHelper(
    sensor_msgs::msg::Image &image_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void *data_arg);

  // ── Sonar pipeline ─────────────────────────────────────────────────────────
  void    ComputeSonarImage(const float *_src);
  cv::Mat ComputeNormalImage(cv::Mat &depth);
  cv::Mat ConstructSonarImage(cv::Mat &depth, cv::Mat &normals);
  cv::Mat ConstructScanImage(cv::Mat &depth, cv::Mat &SNR);
  cv::Mat ConstructVisualScanImage(cv::Mat &raw_scan);
  void    ApplySpeckleNoise(cv::Mat &scan, float fov);
  void    ApplySmoothing(cv::Mat &scan, float fov);
  void    ApplyMedianFilter(cv::Mat &scan);

  // ── Camera info ────────────────────────────────────────────────────────────
  void PublishCameraInfo(const rclcpp::Time &stamp);

  // ── Gazebo handles ─────────────────────────────────────────────────────────
  gz::sim::Entity sensorEntity_{gz::sim::kNullEntity};
  std::shared_ptr<gz::sensors::DepthCameraSensor> depthSensor_;
  gz::rendering::DepthCameraPtr                   depthCamera_;

  gz::common::ConnectionPtr newDepthFrameConn_;
  gz::common::ConnectionPtr newRGBPointCloudConn_;
  gz::common::ConnectionPtr newImageFrameConn_;

  std::chrono::steady_clock::duration depth_sensor_update_time_{0};
  std::chrono::steady_clock::duration
    last_depth_image_camera_info_update_time_{0};

  // ── ROS 2 handles ──────────────────────────────────────────────────────────
  std::shared_ptr<rclcpp::Node> rosNode_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr        depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr        normal_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr        multibeam_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr        sonar_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr        raw_sonar_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr        image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr   depth_image_camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr   camera_info_pub_;

  // ── State ──────────────────────────────────────────────────────────────────
  bool        initialized_{false};
  int         width_{0}, height_{0};
  double      focal_length_{0.0};
  float       cx_{0.f}, cy_{0.f};
  double      point_cloud_cutoff_{0.4};
  std::string frame_name_;
  std::string point_cloud_topic_name_;
  std::string depth_image_topic_name_;
  std::string depth_image_camera_info_topic_name_;
  std::string image_topic_name_;
  std::string camera_info_topic_name_;

  int point_cloud_connect_count_{0};
  int depth_info_connect_count_{0};
  int depth_image_connect_count_{0};

  std::mutex lock_;

  sensor_msgs::msg::Image       image_msg_;
  cv::Mat                       dist_matrix_;

  std::vector<std::vector<int>> angle_range_indices_;
  std::vector<int>              angle_nbr_indices_;

  std::default_random_engine generator_;
};

} // namespace gz::sim::systems
