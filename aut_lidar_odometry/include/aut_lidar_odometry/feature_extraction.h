// This code is inspired by:
// Shan, Tixiao and Englot, Brendan. LeGO-LOAM. https://github.com/RobustFieldAutonomyLab/LeGO-LOAM (Under BSD-3 License)

#ifndef AUT_LIDAR_ODOMETRY_FEATURE_EXTRACTION_H_
#define AUT_LIDAR_ODOMETRY_FEATURE_EXTRACTION_H_

#include <mutex>
#include <memory>
#include <deque>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include "aut_lidar_odometry/msg/features.hpp"

struct PointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
  (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
  (uint16_t, ring, ring) (float, time, time)
) 

namespace aut_lidar_odometry {

struct smoothness_t {
  float value;
  size_t ind;
};

class FeatureExtraction : public rclcpp::Node {

 public:
  explicit FeatureExtraction(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());  // TODO: See if we need to put "= default" or "= delete"

 private:
  void ResetState();

  void KOdomCallBack(const nav_msgs::msg::Odometry::SharedPtr k_odom_msg);  // TODO: Remove this function and replace by tf2

  void VOdomCallBack(const nav_msgs::msg::Odometry::SharedPtr v_odom_msg);  // TODO: Remove this function and replace by tf2

  void PointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg);

  bool CanProcessPointCloud();

  void PreprocessPointCloud();

  void ImageProjection();

  void DeskewPoint(pcl::PointXYZI & p, float point_time);

  void OrganizeCloud();

  void ComputeSmoothness();

  void MarkUnreliable();

  void ExctractFeatures();

  void PublishFeatures();

  // Parameters
  int max_cache_size_;
  float max_lidar_range_;
  float min_box_x_;
  float max_box_x_;

  float min_box_y_;
  float max_box_y_;

  float min_box_z_;
  float max_box_z_;

  float threshold_edge_;
  float threshold_flat_;

  Eigen::Affine3f affine_body_tform_velodyne_; // TODO: Remove

  // Current
  sensor_msgs::msg::PointCloud2 current_point_cloud_msg_;

  std_msgs::msg::Header current_header_;

  float current_point_start_time_;
  float current_point_end_time_;

  long current_start_time_cloud_;
  long current_end_time_cloud_;

  pcl::PointCloud<PointXYZIRT>::Ptr current_cloud_;

  geometry_msgs::msg::TransformStamped transform_start_;
  geometry_msgs::msg::TransformStamped transform_end_;

  Eigen::Quaternionf rotation_start_;
  Eigen::Quaternionf rotation_end_;

  Eigen::Vector3f translation_start_;
  Eigen::Vector3f translation_end_;

  Eigen::Affine3f affine_start_;
  Eigen::Affine3f affine_end_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr image_cloud_;

  Eigen::MatrixXf range_matrix_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr organized_cloud_;

  std::vector<int> start_ring_index_;
  std::vector<int> end_ring_index_;

  std::vector<int> cloud_column_id_;
  std::vector<float> cloud_range_;

  std::vector<float> cloud_curvature_;
  std::vector<smoothness_t> cloud_smoothness_;
  std::vector<int> cloud_label_;
  std::vector<int> cloud_neighbor_picked_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr flat_points_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr flat_points_scan_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr flat_points_scan_ds_;

  // Cache
  std::deque<sensor_msgs::msg::PointCloud2> cache_point_cloud_msg_;

  // tf buffer
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // tf2 Listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr v_odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr k_odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr v_odom_callback_group_;
  rclcpp::CallbackGroup::SharedPtr callback_group_odom_;

  // Publisher
  rclcpp::Publisher<aut_lidar_odometry::msg::Features>::SharedPtr features_publisher_;

  // Mutexes
  std::mutex tf_buffer_mtx_;

};

}  // namespace aut_lidar_odometry

#endif  // AUT_LIDAR_ODOMETRY_FEATURE_EXTRACTION_H_
