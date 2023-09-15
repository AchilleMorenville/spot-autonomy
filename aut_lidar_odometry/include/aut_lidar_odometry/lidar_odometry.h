#ifndef AUT_LIDAR_ODOMETRY_LIDAR_ODOMETRY_H_
#define AUT_LIDAR_ODOMETRY_LIDAR_ODOMETRY_H_

#include <deque>
#include <memory>
#include <chrono>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/transform.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>

#include "aut_lidar_odometry/msg/features.hpp"
#include "aut_msgs/msg/point_cloud_with_pose.hpp"

class Timer {
public:
  Timer() {
    start_time_point = std::chrono::high_resolution_clock::now();
  }
  ~Timer() {
    Stop();
  }

  void Stop() {

    auto end_time_point = std::chrono::high_resolution_clock::now();

    auto start = std::chrono::time_point_cast<std::chrono::milliseconds>(start_time_point).time_since_epoch().count();
    auto end = std::chrono::time_point_cast<std::chrono::milliseconds>(end_time_point).time_since_epoch().count();

    auto duration = end - start;

    std::cout << duration << " ms\n";
  }
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_point;
};

namespace aut_lidar_odometry {

class LidarOdometry : public rclcpp::Node {

 public:
  explicit LidarOdometry(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());  // TODO: See if we need to put "= default" or "= delete"

 private:

  void FeaturesCallBack(const aut_lidar_odometry::msg::Features::SharedPtr features_msg);

  void GetInput(const aut_lidar_odometry::msg::Features::SharedPtr features_msg);

  void ComputeLocalMap();

  void OptimizeInput();

  bool NeedSave();

  void SaveFrame();

  void PublishOdom();

  void LocalMapBuilder();

  // Parameters
  float edge_leaf_size_;
  float flat_leaf_size_;

  float dist_new_key_frame_;
  float rad_new_key_frame_;

  int max_deque_size_;

  // Input
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_all_points_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_edge_points_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_flat_points_;

  sensor_msgs::msg::PointCloud2 input_all_points_msg_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_edge_points_ds_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_flat_points_ds_;

  std_msgs::msg::Header input_header_;

  Eigen::Matrix4f input_initial_guess_;

  // Last
  Eigen::Matrix4f last_odom_estimation_;
  Eigen::Matrix4f last_odom_optimized_;

  Eigen::Matrix4f last_saved_odom_optimized_;

  // Optimized
  Eigen::Matrix4f odom_optimized_;

  // KeyFrames
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> deque_edge_points_;
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> deque_flat_points_;

  // Local Map
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_edge_points_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_flat_points_;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_local_map_edge_points_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_local_map_flat_points_;

  // Voxel Grids
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_edge_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_flat_;

  // Subscription
  rclcpp::Subscription<aut_lidar_odometry::msg::Features>::SharedPtr features_subscription_;
  
  // Callback groups
  rclcpp::CallbackGroup::SharedPtr callback_group_features_;

  // Publisher
  rclcpp::Publisher<aut_msgs::msg::PointCloudWithPose>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_publisher_;

  // tf2 Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Mutex
  std::mutex state_mtx_;

};

}  // namespace aut_lidar_odometry

#endif  // AUT_LIDAR_ODOMETRY_LIDAR_ODOMETRY_H_