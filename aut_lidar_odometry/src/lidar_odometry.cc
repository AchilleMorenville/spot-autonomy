#include "aut_lidar_odometry/lidar_odometry.h"
#include "aut_lidar_odometry/optimization.h"

#include <chrono>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "aut_utils/utils.h"

#include "aut_lidar_odometry/msg/features.hpp"
#include "aut_msgs/msg/point_cloud_with_pose.hpp"

namespace aut_lidar_odometry {

LidarOdometry::LidarOdometry(const rclcpp::NodeOptions& options)
    : Node("lidar_odometry", options) {

  // Parameters
  this->declare_parameter("edge_leaf_size", 0.05f);
  edge_leaf_size_ = this->get_parameter("edge_leaf_size").get_parameter_value().get<float>();

  this->declare_parameter("flat_leaf_size", 0.1f);
  flat_leaf_size_ = this->get_parameter("flat_leaf_size").get_parameter_value().get<float>();

  this->declare_parameter("rad_new_key_frame", 0.17f);
  rad_new_key_frame_ = this->get_parameter("rad_new_key_frame").get_parameter_value().get<float>();

  this->declare_parameter("dist_new_key_frame", 0.2f);
  dist_new_key_frame_ = this->get_parameter("dist_new_key_frame").get_parameter_value().get<float>();

  this->declare_parameter("max_deque_size", 50);
  max_deque_size_ = this->get_parameter("max_deque_size").get_parameter_value().get<int>();
  
  callback_group_features_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );

  rclcpp::SubscriptionOptions features_options = rclcpp::SubscriptionOptions();
  features_options.callback_group = callback_group_features_;

  features_subscription_ = this->create_subscription<aut_lidar_odometry::msg::Features>(
    "aut_lidar_odometry/features", 10, 
    std::bind(&LidarOdometry::FeaturesCallBack, this, std::placeholders::_1),
    features_options
  );

  odom_publisher_ = this->create_publisher<aut_msgs::msg::PointCloudWithPose>("aut_lidar_odometry/point_cloud_with_pose", 10);

  local_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aut_lidar_odometry/local_map", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&LidarOdometry::LocalMapBuilder, this));

  // Memory allocation
  input_all_points_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  input_edge_points_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  input_flat_points_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  input_edge_points_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  input_flat_points_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  input_initial_guess_ = Eigen::Matrix4f::Identity();

  local_map_edge_points_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  local_map_flat_points_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  kdtree_local_map_edge_points_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  kdtree_local_map_flat_points_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);

  // Voxel Grids
  voxel_grid_edge_.setLeafSize(edge_leaf_size_, edge_leaf_size_, edge_leaf_size_);
  voxel_grid_flat_.setLeafSize(flat_leaf_size_, flat_leaf_size_, flat_leaf_size_);

  // Last
  last_odom_estimation_ = Eigen::Matrix4f::Identity();
  last_odom_optimized_ = Eigen::Matrix4f::Identity();

  last_saved_odom_optimized_ = Eigen::Matrix4f::Identity();
  
  // Optimized
  odom_optimized_ = Eigen::Matrix4f::Identity();

}

void LidarOdometry::FeaturesCallBack(const aut_lidar_odometry::msg::Features::SharedPtr features_msg) {
  RCLCPP_INFO(this->get_logger(), "Received features");

  {
    Timer timer;
    std::lock_guard<std::mutex> lock(state_mtx_);
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_point = std::chrono::high_resolution_clock::now();
    GetInput(features_msg);
    ComputeLocalMap();
    OptimizeInput();
    SaveFrame();
    PublishOdom();
    auto end_time_point = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::time_point_cast<std::chrono::milliseconds>(start_time_point).time_since_epoch().count();
    auto end = std::chrono::time_point_cast<std::chrono::milliseconds>(end_time_point).time_since_epoch().count();
    auto duration = end - start;
    RCLCPP_INFO(this->get_logger(), "Execution time compute local map : %ld", duration);
  }
}

void LidarOdometry::GetInput(const aut_lidar_odometry::msg::Features::SharedPtr features_msg) {

  input_all_points_msg_ = features_msg->cloud;

  pcl::fromROSMsg(features_msg->cloud, *input_all_points_);
  pcl::fromROSMsg(features_msg->cloud_edge, *input_edge_points_);
  pcl::fromROSMsg(features_msg->cloud_flat, *input_flat_points_);

  input_header_ = features_msg->header;

  input_initial_guess_ = aut_utils::TransformToMatrix(features_msg->initial_guess);
}

void LidarOdometry::ComputeLocalMap() {
  return;

  if (deque_edge_points_.empty()) {
    return;
  }

  local_map_edge_points_->clear();
  local_map_flat_points_->clear();

  const std::size_t deque_size = deque_edge_points_.size();
  for (std::size_t i = 0; i < deque_size; ++i) {
    *local_map_edge_points_ += *deque_edge_points_[i];
    *local_map_flat_points_ += *deque_flat_points_[i];
  }

  voxel_grid_edge_.setInputCloud(local_map_edge_points_);
  voxel_grid_edge_.filter(*local_map_edge_points_);

  voxel_grid_flat_.setInputCloud(local_map_flat_points_);
  voxel_grid_flat_.filter(*local_map_flat_points_);

  kdtree_local_map_edge_points_->setInputCloud(local_map_edge_points_);
  kdtree_local_map_flat_points_->setInputCloud(local_map_flat_points_);
}

void LidarOdometry::OptimizeInput() {
  if (deque_edge_points_.empty()) {
    return;
  }

  voxel_grid_edge_.setInputCloud(input_edge_points_);
  voxel_grid_edge_.filter(*input_edge_points_ds_);

  voxel_grid_flat_.setInputCloud(input_flat_points_);
  voxel_grid_flat_.filter(*input_flat_points_ds_);

  Eigen::Matrix4f odom_estimation = last_odom_optimized_ * aut_utils::GetDifferenceTransformation(last_odom_estimation_, input_initial_guess_);

  odom_optimized_ = Optimize(
    odom_estimation,
    input_edge_points_ds_,
    input_flat_points_ds_,
    local_map_edge_points_,
    local_map_flat_points_,
    kdtree_local_map_edge_points_,
    kdtree_local_map_flat_points_,
    0.1
  );

}

bool LidarOdometry::NeedSave() {

  if (deque_edge_points_.empty()) {
    return true;
  }

  // Compute the displacement
  Eigen::Matrix4f displacement = aut_utils::GetDifferenceTransformation(last_saved_odom_optimized_, odom_optimized_);

  Eigen::Affine3f affine_displacement;
  affine_displacement.matrix() = displacement;

  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(affine_displacement, x, y, z, roll, pitch, yaw);

  return std::abs(yaw) >= rad_new_key_frame_ || 
         std::abs(pitch) >= rad_new_key_frame_ || 
         std::abs(roll) >= rad_new_key_frame_ || 
         x * x + y * y + z * z >= dist_new_key_frame_ * dist_new_key_frame_;
}

void LidarOdometry::SaveFrame() {
  last_odom_optimized_ = odom_optimized_;
  last_odom_estimation_ = input_initial_guess_;

  if (!NeedSave()) {
    return;
  }

  last_saved_odom_optimized_ = odom_optimized_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_input_edge_points(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_input_flat_points(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::transformPointCloud(*input_edge_points_, *transformed_input_edge_points, odom_optimized_);
  pcl::transformPointCloud(*input_flat_points_, *transformed_input_flat_points, odom_optimized_);

  deque_edge_points_.push_back(transformed_input_edge_points);
  deque_flat_points_.push_back(transformed_input_flat_points);

  if ((int) deque_edge_points_.size() > max_deque_size_) { // TODO: Change this conversion
    deque_edge_points_.pop_front();
    deque_flat_points_.pop_front();
  }

  if ((int) deque_edge_points_.size() == 1) {
    local_map_edge_points_->clear();
    local_map_flat_points_->clear();

    const std::size_t deque_size = deque_edge_points_.size();
    for (std::size_t i = 0; i < deque_size; ++i) {
      *local_map_edge_points_ += *deque_edge_points_[i];
      *local_map_flat_points_ += *deque_flat_points_[i];
    }

    voxel_grid_edge_.setInputCloud(local_map_edge_points_);
    voxel_grid_edge_.filter(*local_map_edge_points_);

    voxel_grid_flat_.setInputCloud(local_map_flat_points_);
    voxel_grid_flat_.filter(*local_map_flat_points_);

    kdtree_local_map_edge_points_->setInputCloud(local_map_edge_points_);
    kdtree_local_map_flat_points_->setInputCloud(local_map_flat_points_);
  }
}

void LidarOdometry::PublishOdom() {

  //  Publish point cloud with pose

  aut_msgs::msg::PointCloudWithPose point_cloud_with_pose_msg;
  point_cloud_with_pose_msg.header = input_header_;
  input_all_points_msg_.header.frame_id = "base_link";
  point_cloud_with_pose_msg.point_cloud = input_all_points_msg_;

  Eigen::Quaternionf quat(odom_optimized_.block<3, 3>(0, 0));

  point_cloud_with_pose_msg.pose.translation.x = odom_optimized_(0, 3);
  point_cloud_with_pose_msg.pose.translation.y = odom_optimized_(1, 3);
  point_cloud_with_pose_msg.pose.translation.z = odom_optimized_(2, 3);
  point_cloud_with_pose_msg.pose.rotation.w = quat.w();
  point_cloud_with_pose_msg.pose.rotation.x = quat.x();
  point_cloud_with_pose_msg.pose.rotation.y = quat.y();
  point_cloud_with_pose_msg.pose.rotation.z = quat.z();

  odom_publisher_->publish(point_cloud_with_pose_msg);

  //  Publish local map

  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZI>());
  *local_map += *local_map_edge_points_;
  *local_map += *local_map_flat_points_;

  sensor_msgs::msg::PointCloud2 point_cloud_msg;
  pcl::toROSMsg(*local_map, point_cloud_msg);
  point_cloud_msg.header.frame_id = "l_odom";

  local_map_publisher_->publish(point_cloud_msg); 

  // Publish odometry to tf2

  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = input_header_.stamp;
  t.header.frame_id = "l_odom";
  t.child_frame_id = "v_odom";

  RCLCPP_INFO(this->get_logger(), "Published odometry with time: %d.%d", input_header_.stamp.sec, input_header_.stamp.nanosec);

  Eigen::Matrix4f m_l_odom_to_v_odom = odom_optimized_ * aut_utils::InverseTransformation(input_initial_guess_);

  t.transform.translation.x = m_l_odom_to_v_odom(0, 3);
  t.transform.translation.y = m_l_odom_to_v_odom(1, 3);
  t.transform.translation.z = m_l_odom_to_v_odom(2, 3);

  Eigen::Quaternionf quat_l_odom_to_v_odom(m_l_odom_to_v_odom.block<3, 3>(0, 0));
  t.transform.rotation.w = quat_l_odom_to_v_odom.w();
  t.transform.rotation.x = quat_l_odom_to_v_odom.x();
  t.transform.rotation.y = quat_l_odom_to_v_odom.y();
  t.transform.rotation.z = quat_l_odom_to_v_odom.z();

  tf_broadcaster_->sendTransform(t);

  RCLCPP_INFO(this->get_logger(), "Published odometry");
}

void LidarOdometry::LocalMapBuilder() {
  std::lock_guard<std::mutex> lock(state_mtx_);
  if (deque_edge_points_.empty()) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Update local map");

  local_map_edge_points_->clear();
  local_map_flat_points_->clear();

  const std::size_t deque_size = deque_edge_points_.size();
  for (std::size_t i = 0; i < deque_size; ++i) {
    *local_map_edge_points_ += *deque_edge_points_[i];
    *local_map_flat_points_ += *deque_flat_points_[i];
  }

  voxel_grid_edge_.setInputCloud(local_map_edge_points_);
  voxel_grid_edge_.filter(*local_map_edge_points_);

  voxel_grid_flat_.setInputCloud(local_map_flat_points_);
  voxel_grid_flat_.filter(*local_map_flat_points_);

  kdtree_local_map_edge_points_->setInputCloud(local_map_edge_points_);
  kdtree_local_map_flat_points_->setInputCloud(local_map_flat_points_);
}

}  // namespace aut_lidar_odometry

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500)
  );

  auto lidar_odometry_node = std::make_shared<aut_lidar_odometry::LidarOdometry>();
  
  exec.add_node(lidar_odometry_node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Lidar Odometry Started.\033[0m");

  exec.spin();
  rclcpp::shutdown();
  return 0;
}