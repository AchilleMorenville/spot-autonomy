#ifndef AUT_SLAM_GRAPH_OPTIMIZATION_H_
#define AUT_SLAM_GRAPH_OPTIMIZATION_H_

#include <vector>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <builtin_interfaces/msg/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

#include "aut_msgs/msg/point_cloud_with_pose.hpp"
#include "aut_msgs/msg/fiducial.hpp"
#include "aut_msgs/srv/save_map.hpp"
#include "aut_common/graph.h"

namespace aut_slam {

class GraphOptimization : public rclcpp::Node {

 public:
  explicit GraphOptimization(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());  // TODO: See if we need to put "= default" or "= delete" 

  void MapThread();

  void LoopThread();

 private:

  void GravityCallBack(const geometry_msgs::msg::TransformStamped::SharedPtr geometry_msg);

  void PointCloudWithPoseCallBack(const aut_msgs::msg::PointCloudWithPose::SharedPtr point_cloud_with_pose_msg);

  void FiducialCallBack(const aut_msgs::msg::Fiducial::SharedPtr fiducial_msg);

  void GetInput(const aut_msgs::msg::PointCloudWithPose::SharedPtr point_cloud_with_pose_msg);

  bool NeedSave();

  bool AddFrame();

  void AddOdomFactor();

  void AddGravityFactor();

  void AddLoopFactor();

  void UpdatePoses();

  void PublishPose();

  void PerformLoopClosure();

  bool DetectLoopClosure();

  void BuildSourceAndTargetPointClouds();

  bool ComputeICP();

  void ComputeFactor();

  void PublishMap();

  void Start(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void Stop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void Reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void SaveMap(const std::shared_ptr<aut_msgs::srv::SaveMap::Request> request, std::shared_ptr<aut_msgs::srv::SaveMap::Response> response);

  // Parameters

  bool running_;

  int n_keyframe_;

  float rad_new_key_frame_;
  float dist_new_key_frame_;

  bool loop_is_closed_;

  float map_frequency_;
  float loop_frequency_;

  float loop_search_dist_;

  int nbr_frames_before_loop_;

  int loop_candidate_local_map_size_;

  // Inputs
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_point_cloud_;
  std_msgs::msg::Header input_header_;
  Eigen::Matrix4f input_odom_;

  // Last
  Eigen::Matrix4f last_key_frame_odom_;

  // Corrected
  Eigen::Matrix4f corrected_input_odom_;

  // KeyFrames
  std::vector<Eigen::Matrix4f> poses_6D_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_frames_point_cloud_;
  std::vector<std_msgs::msg::Header> key_frames_headers_;

  std::vector<Eigen::Matrix4f> key_frames_l_odom_;
  std::vector<Eigen::Matrix4f> key_frames_k_odom_;
  std::vector<Eigen::Matrix4f> key_frames_v_odom_;


  pcl::PointCloud<pcl::PointXYZL>::Ptr poses_3D_;

  // Graph
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimate_;
  gtsam::Values optimized_estimate_;
  gtsam::ISAM2* isam_;
  gtsam::Values isam_current_estimate_;

  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_; 

  // Nav graph 

  aut_common::Graph nav_graph_;

  // Fiducials
  std::unordered_set<int> fiducials_tag_id_;
  std::unordered_map<int, float> fiducials_dist_;
  std::unordered_map<int, Eigen::Matrix4f> fiducials_base_link_to_fiducial_;
  std::unordered_map<int, builtin_interfaces::msg::Time> fiducials_time_;

  // Loops
  pcl::KdTreeFLANN<pcl::PointXYZL>::Ptr kdtree_poses_3D_;

  // std::unordered_map<int, std::unordered_set<int>> loops_;

  Eigen::Matrix4f loop_current_pose_;
  Eigen::Matrix4f loop_candidate_pose_;

  Eigen::Matrix4f icp_transformation_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr loop_current_point_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr loop_candidate_point_cloud_;

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_loop_;

  int idx_loop_candidate_;
  int idx_current_;

  std::vector<gtsam::BetweenFactor<gtsam::Pose3>> loop_factors_;

  // Subscription
  rclcpp::Subscription<aut_msgs::msg::PointCloudWithPose>::SharedPtr point_cloud_with_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr gravity_subscription_;
  rclcpp::Subscription<aut_msgs::msg::Fiducial>::SharedPtr fiducial_subscription_;


  // Callback groups
  rclcpp::CallbackGroup::SharedPtr callback_group_point_cloud_with_pose_;
  rclcpp::CallbackGroup::SharedPtr callback_group_gravity_;
  rclcpp::CallbackGroup::SharedPtr callback_group_fiducial_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

  // tf2 Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // tf buffer
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // tf2 Listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Services
  rclcpp::Service<aut_msgs::srv::SaveMap>::SharedPtr save_map_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service;

  // Mutexes
  std::mutex state_mtx_;
  std::mutex tf_buffer_mtx_;
  std::mutex fiducial_mtx_;
  std::mutex running_mtx_;

};

}  // namespace aut_slam

#endif  // AUT_SLAM_GRAPH_OPTIMIZATION_H_