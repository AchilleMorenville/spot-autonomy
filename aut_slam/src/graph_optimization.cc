#include "aut_slam/graph_optimization.h"

#include <memory>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstddef>
#include <cmath>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

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
#include <gtsam_unstable/slam/PartialPriorFactor.h>

#include <dynamicEDT3D/dynamicEDT3D.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTree.h>

#include <aut_msgs/msg/point_cloud_with_pose.hpp>
#include <aut_msgs/msg/fiducial.hpp>
#include <aut_utils/utils.h>
#include <aut_common/graph.h>

namespace aut_slam {

GraphOptimization::GraphOptimization(const rclcpp::NodeOptions& options)
    : Node("graph_optimization", options) {

  n_keyframe_ = 0;
  loop_is_closed_ = false;
  running_ = false;

  idx_current_ = 0;
  idx_loop_candidate_ = 0;

  this->declare_parameter("rad_new_key_frame", 0.17);
  rad_new_key_frame_ = this->get_parameter("rad_new_key_frame").get_parameter_value().get<float>();
  this->declare_parameter("dist_new_key_frame", 0.2);
  dist_new_key_frame_ = this->get_parameter("dist_new_key_frame").get_parameter_value().get<float>();

  this->declare_parameter("map_frequency", 1.0);
  map_frequency_ = this->get_parameter("map_frequency").get_parameter_value().get<float>();
  this->declare_parameter("loop_frequency", 1.0);
  loop_frequency_ = this->get_parameter("loop_frequency").get_parameter_value().get<float>();

  this->declare_parameter("loop_search_dist", 1.0);
  loop_search_dist_ = this->get_parameter("loop_search_dist").get_parameter_value().get<float>();

  this->declare_parameter("nbr_frames_before_loop", 50);
  nbr_frames_before_loop_ = this->get_parameter("nbr_frames_before_loop").get_parameter_value().get<int>();
  this->declare_parameter("loop_candidate_local_map_size", 50);
  loop_candidate_local_map_size_ = this->get_parameter("loop_candidate_local_map_size").get_parameter_value().get<int>();

  this->declare_parameter("start_directly", false);
  running_ = this->get_parameter("start_directly").get_parameter_value().get<bool>();
  
  callback_group_point_cloud_with_pose_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );

  rclcpp::SubscriptionOptions point_cloud_with_pose_options = rclcpp::SubscriptionOptions();
  point_cloud_with_pose_options.callback_group = callback_group_point_cloud_with_pose_;

  point_cloud_with_pose_subscription_ = this->create_subscription<aut_msgs::msg::PointCloudWithPose>(
    "aut_lidar_odometry/point_cloud_with_pose", 10, 
    std::bind(&GraphOptimization::PointCloudWithPoseCallBack, this, std::placeholders::_1),
    point_cloud_with_pose_options
  );

  callback_group_gravity_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );

  callback_group_fiducial_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );

  rclcpp::SubscriptionOptions fiducial_options = rclcpp::SubscriptionOptions();
  fiducial_options.callback_group = callback_group_fiducial_;

  fiducial_subscription_ = this->create_subscription<aut_msgs::msg::Fiducial>(
    "aut_spot/fiducial", 10, 
    std::bind(&GraphOptimization::FiducialCallBack, this, std::placeholders::_1),
    fiducial_options
  );

  save_map_service = this->create_service<aut_msgs::srv::SaveMap>("aut_slam/save_map", std::bind(&GraphOptimization::SaveMap, this, std::placeholders::_1, std::placeholders::_2));
  stop_service = this->create_service<std_srvs::srv::Trigger>("aut_slam/stop", std::bind(&GraphOptimization::Stop, this, std::placeholders::_1, std::placeholders::_2));
  start_service = this->create_service<std_srvs::srv::Trigger>("aut_slam/start", std::bind(&GraphOptimization::Start, this, std::placeholders::_1, std::placeholders::_2));
  reset_service = this->create_service<std_srvs::srv::Trigger>("aut_slam/reset", std::bind(&GraphOptimization::Reset, this, std::placeholders::_1, std::placeholders::_2));


  map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aut_slam/map", 10);

  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("aut_slam/graph", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Memory allocation

  input_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  input_odom_ = Eigen::Matrix4f::Identity();
  last_key_frame_odom_ = Eigen::Matrix4f::Identity();

  corrected_input_odom_ = Eigen::Matrix4f::Identity();

  poses_3D_.reset(new pcl::PointCloud<pcl::PointXYZL>());

  kdtree_poses_3D_.reset(new pcl::KdTreeFLANN<pcl::PointXYZL>);

  loop_current_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  loop_candidate_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  loop_current_pose_ = Eigen::Matrix4f::Identity();
  loop_candidate_pose_ = Eigen::Matrix4f::Identity();
  icp_transformation_ = Eigen::Matrix4f::Identity();

  voxel_grid_loop_.setLeafSize(0.1, 0.1, 0.1);

  // Graph
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 1;
  isam_ = new gtsam::ISAM2(parameters);

  gtsam::Vector prior_Vector6(6);
  prior_Vector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;

  gtsam::Vector odometry_Vector6(6);
  odometry_Vector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;

  prior_noise_ = gtsam::noiseModel::Diagonal::Variances(prior_Vector6);
  odometry_noise_ = gtsam::noiseModel::Diagonal::Variances(odometry_Vector6);
}

void GraphOptimization::FiducialCallBack(const aut_msgs::msg::Fiducial::SharedPtr fiducial_msg) {
  std::lock_guard<std::mutex> lock(fiducial_mtx_);

  if (poses_6D_.empty()) {
    return;
  }

  float squared_dist = fiducial_msg->pose.translation.x * fiducial_msg->pose.translation.x
                     + fiducial_msg->pose.translation.y * fiducial_msg->pose.translation.y
                     + fiducial_msg->pose.translation.z * fiducial_msg->pose.translation.z;

  int tag_id = fiducial_msg->tag_id;

  if (fiducials_tag_id_.find(tag_id) != fiducials_tag_id_.end()) {
    if (squared_dist < fiducials_dist_[tag_id]) {
      fiducials_dist_[tag_id] = squared_dist;
      fiducials_base_link_to_fiducial_[tag_id] = aut_utils::TransformToMatrix(fiducial_msg->pose);
      fiducials_time_[tag_id] = fiducial_msg->header.stamp;
    }
  } else {
    fiducials_tag_id_.insert(tag_id);
    fiducials_dist_[tag_id] = squared_dist;
    fiducials_base_link_to_fiducial_[tag_id] = aut_utils::TransformToMatrix(fiducial_msg->pose);
    fiducials_time_[tag_id] = fiducial_msg->header.stamp;
  }
}

void GraphOptimization::PointCloudWithPoseCallBack(const aut_msgs::msg::PointCloudWithPose::SharedPtr point_cloud_with_pose_msg) {
  {
    std::lock_guard<std::mutex> lock(running_mtx_);
    if (!running_) {
      return;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Received point cloud");
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    GetInput(point_cloud_with_pose_msg); 

    if (!AddFrame()) {
      return;
    }

    UpdatePoses();

    PublishPose();
  }
}

void GraphOptimization::GetInput(const aut_msgs::msg::PointCloudWithPose::SharedPtr point_cloud_with_pose_msg) {
  pcl::fromROSMsg(point_cloud_with_pose_msg->point_cloud, *input_point_cloud_);
  input_header_ = point_cloud_with_pose_msg->header;
  input_odom_ = aut_utils::TransformToMatrix(point_cloud_with_pose_msg->pose);
}

bool GraphOptimization::AddFrame() {

  if (!NeedSave()) {
    return false;
  }

  AddOdomFactor();

  AddGravityFactor();

  AddLoopFactor();

  last_key_frame_odom_ = input_odom_;

  isam_->update(graph_, initial_estimate_);
  isam_->update();

  if (true || loop_is_closed_ == true) {
    isam_->update();
    isam_->update();
    isam_->update();
    isam_->update();
    isam_->update();
  }

  graph_.resize(0);
  initial_estimate_.clear();

  poses_6D_.push_back(corrected_input_odom_);

  key_frames_l_odom_.push_back(input_odom_);

  geometry_msgs::msg::Transform trans_v = tf_buffer_->lookupTransform("v_odom", "base_link", input_header_.stamp).transform;
  Eigen::Matrix4f v_odom = aut_utils::TransformToMatrix(trans_v);
  key_frames_v_odom_.push_back(v_odom);

  geometry_msgs::msg::Transform trans_k = tf_buffer_->lookupTransform("k_odom", "base_link", input_header_.stamp).transform;
  Eigen::Matrix4f k_odom = aut_utils::TransformToMatrix(trans_k);
  key_frames_k_odom_.push_back(k_odom);

  pcl::PointXYZL point(corrected_input_odom_(0, 3),
                       corrected_input_odom_(1, 3),
                       corrected_input_odom_(2, 3),
                       (int) poses_6D_.size() - 1);

  poses_3D_->push_back(point);

  pcl::PointCloud<pcl::PointXYZI>::Ptr copy_input_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*input_point_cloud_,  *copy_input_point_cloud);
  key_frames_point_cloud_.push_back(copy_input_point_cloud);
  key_frames_headers_.push_back(input_header_);

  nav_graph_.AddVertex(corrected_input_odom_);
  if (n_keyframe_ > 0) {
    nav_graph_.AddEdge(n_keyframe_ - 1, n_keyframe_);
  }
  n_keyframe_++;

  return true;
}

bool GraphOptimization::NeedSave() {
  if (poses_6D_.empty()) {
    return true;
  }

  Eigen::Matrix4f displacement = aut_utils::GetDifferenceTransformation(last_key_frame_odom_, input_odom_);

  Eigen::Affine3f affine_displacement;
  affine_displacement.matrix() = displacement;

  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(affine_displacement, x, y, z, roll, pitch, yaw);

  return std::abs(yaw) >= rad_new_key_frame_ || 
         std::abs(pitch) >= rad_new_key_frame_ || 
         std::abs(roll) >= rad_new_key_frame_ || 
         x * x + y * y + z * z >= dist_new_key_frame_ * dist_new_key_frame_;
}

void GraphOptimization::AddOdomFactor() {
  if (poses_6D_.empty()) {
    if (tf_buffer_->canTransform("gravity", "base_link", input_header_.stamp)) {
      geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("gravity", "base_link", input_header_.stamp).transform;
      corrected_input_odom_ = aut_utils::TransformToMatrix(trans);
    }
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', (int) 0), gtsam::Pose3(corrected_input_odom_.cast<double>()), prior_noise_));
    initial_estimate_.insert(gtsam::Symbol('x', (int) 0), gtsam::Pose3(corrected_input_odom_.cast<double>()));
  } else {
    Eigen::Matrix4f displacement = aut_utils::GetDifferenceTransformation(last_key_frame_odom_, input_odom_);
    corrected_input_odom_ = poses_6D_[poses_6D_.size() - 1] * displacement;
    gtsam::Pose3 pose_from = gtsam::Pose3(poses_6D_[poses_6D_.size() - 1].cast<double>());
    gtsam::Pose3 pose_to = gtsam::Pose3(corrected_input_odom_.cast<double>());
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', (int) poses_6D_.size() - 1), gtsam::Symbol('x', (int) poses_6D_.size()), pose_from.between(pose_to), odometry_noise_));
    initial_estimate_.insert(gtsam::Symbol('x', (int) poses_6D_.size()), gtsam::Pose3(corrected_input_odom_.cast<double>()));
  } 
}

void GraphOptimization::AddGravityFactor() {
  if (poses_6D_.empty()) {
    return;
  }

  if (!tf_buffer_->canTransform("gravity", "base_link", input_header_.stamp)) {
    return;
  }
  geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("gravity", "base_link", input_header_.stamp).transform;
  Eigen::Matrix4f gravity_tform_body = aut_utils::TransformToMatrix(trans);

  Eigen::Affine3f affine_gravity_tform_body;
  affine_gravity_tform_body.matrix() = gravity_tform_body;

  // float x, y, z, roll, pitch, yaw;
  // pcl::getTranslationAndEulerAngles(affine_gravity_tform_body, x, y, z, roll, pitch, yaw);

  // RCLCPP_INFO(this->get_logger(), "Gravity roll : %f, pitch : %f, yaw : %f, x : %f, y : %f, z : %f", roll, pitch, yaw, x, y, z);

  // gtsam::Vector2 roll_pitch_prior(roll, pitch);
  // std::vector<std::size_t> roll_pitch_indices = { 0, 1 };
  gtsam::Vector grav_Vector2(2);
  grav_Vector2 << 1e-3, 1e-3;
  gtsam::noiseModel::Isotropic::shared_ptr roll_pitch_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1e-3);
  

  // gtsam::noiseModel::Diagonal::shared_ptr roll_pitch_noise = gtsam::noiseModel::Diagonal::Variances(grav_Vector2);
  // auto partial_roll_pitch_prior_factor = gtsam::PartialPriorFactor<gtsam::Pose3>(gtsam::Symbol('x', (int) poses_6D_.size()), roll_pitch_indices, roll_pitch_prior, roll_pitch_noise);
  // graph_.add(partial_roll_pitch_prior_factor);

  Eigen::Vector3f vec_point(0.0, 0.0, -1.0);
  Eigen::Vector3f org_vec_point(0.0, 0.0, -1.0);

  vec_point = affine_gravity_tform_body * vec_point;
  vec_point = vec_point.normalized();

  gtsam::Unit3 grav_direction(vec_point(0), vec_point(1), vec_point(2));

  RCLCPP_INFO(this->get_logger(), "Gravity : %f, %f, %f", vec_point(0), vec_point(1), vec_point(2));

  gtsam::Unit3 ref(0, 0, -1); // TODO: The robot must be up and flat to start. The alternative is to detect the gravity at startup and set that as the ref

  if (vec_point.dot(org_vec_point) < 0.9999) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Add gravity dot : %f", vec_point.dot(org_vec_point));

  // graph_.add(gtsam::Pose3AttitudeFactor(gtsam::Symbol('x', (int) poses_6D_.size()), grav_direction, roll_pitch_noise, ref));

  gtsam::Vector grav_Vector6(6);
  grav_Vector6 << 1e-3, 1e-3, 1e8, 1e8, 1e8, 1e8;
  gtsam::noiseModel::Diagonal::shared_ptr grav_noise_ = gtsam::noiseModel::Diagonal::Variances(grav_Vector6);
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', (int) poses_6D_.size()), gtsam::Pose3(gravity_tform_body.cast<double>()), grav_noise_));
}

void GraphOptimization::AddLoopFactor() {
  if (loop_factors_.size() > 0) {
    for (int i = 0; i < (int) loop_factors_.size(); ++i) {
      graph_.add(loop_factors_[i]);
    }
    loop_factors_.clear();
    loop_is_closed_ = true;
  }
}

void GraphOptimization::UpdatePoses() {
  if (true || loop_is_closed_) {
    isam_current_estimate_ = isam_->calculateEstimate();
    for (int i = 0; i < (int) poses_6D_.size(); ++i) {
      Eigen::Matrix4f estimate = isam_current_estimate_.at<gtsam::Pose3>(gtsam::Symbol('x', i)).matrix().cast<float>();
      poses_3D_->points[i].x = estimate(0, 3);
      poses_3D_->points[i].y = estimate(1, 3);
      poses_3D_->points[i].z = estimate(2, 3);

      poses_6D_[i] = estimate;
    }

    loop_is_closed_ = false;
  }

  nav_graph_.UpdatePoses(poses_6D_);
}

void GraphOptimization::PublishPose() {
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = input_header_.stamp;
  t.header.frame_id = "map";
  t.child_frame_id = "l_odom";

  Eigen::Matrix4f map_tform_body = corrected_input_odom_ * aut_utils::InverseTransformation(input_odom_);

  t.transform.translation.x = map_tform_body(0, 3);
  t.transform.translation.y = map_tform_body(1, 3);
  t.transform.translation.z = map_tform_body(2, 3);

  Eigen::Quaternionf quat(map_tform_body.block<3, 3>(0, 0));

  t.transform.rotation.w = quat.w();
  t.transform.rotation.x = quat.x();
  t.transform.rotation.y = quat.y();
  t.transform.rotation.z = quat.z();

  tf_broadcaster_->sendTransform(t); 

  // sensor_msgs::msg::PointCloud2 map_msg;
  // pcl::toROSMsg(*key_frames_point_cloud_[poses_6D_.size() - 1], map_msg);

  // map_msg.header.frame_id = "map";
  // map_publisher_->publish(map_msg);
}

void GraphOptimization::LoopThread() {
  rclcpp::Rate rate(loop_frequency_);
  while (rclcpp::ok()) {
    rate.sleep();
    PerformLoopClosure();
  }
}

void GraphOptimization::PerformLoopClosure() {

  {
    std::lock_guard<std::mutex> lock(running_mtx_);
    if (!running_) {
      return;
    }
  }

  if (poses_6D_.empty()) {
    return;
  }

  if (!DetectLoopClosure()) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Loop closure detected");

  BuildSourceAndTargetPointClouds();

  if (!ComputeICP()) {
    return;
  }

  ComputeFactor();
}

bool GraphOptimization::DetectLoopClosure() {

  pcl::PointCloud<pcl::PointXYZL>::Ptr copy_poses_3D(new pcl::PointCloud<pcl::PointXYZL>());

  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    *copy_poses_3D += *poses_3D_;
  }

  idx_current_ = copy_poses_3D->points.size() - 1;

  // Select a pose for loop closure
  std::vector<int> poses_search_idx;
  std::vector<float> poses_search_sq_dist;

  kdtree_poses_3D_->setInputCloud(copy_poses_3D);
  kdtree_poses_3D_->radiusSearch(copy_poses_3D->points[idx_current_], loop_search_dist_, poses_search_idx, poses_search_sq_dist, 0);

  idx_loop_candidate_ = -1;
  for (int i = 0; i < (int) poses_search_idx.size(); i++) {
    if (idx_current_ - poses_search_idx[i] > nbr_frames_before_loop_) {
      idx_loop_candidate_ = poses_search_idx[i];
      break;
    }
  }

  return idx_loop_candidate_ >= 0;
}

void GraphOptimization::BuildSourceAndTargetPointClouds() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_loop_current_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_loop_candidate_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());

  loop_current_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  loop_candidate_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  {
    std::lock_guard<std::mutex> lock(state_mtx_);

    loop_candidate_pose_ = poses_6D_[idx_loop_candidate_];
    loop_current_pose_ = poses_6D_[idx_current_];

    pcl::transformPointCloud(*key_frames_point_cloud_[idx_current_], *temp_loop_current_point_cloud, poses_6D_[idx_current_]);
    for (int i = -loop_candidate_local_map_size_ / 2; i < loop_candidate_local_map_size_ / 2; ++i) {
      if (idx_loop_candidate_ + i >= 0 && idx_loop_candidate_ + i < idx_current_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_key_frame_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*key_frames_point_cloud_[idx_loop_candidate_ + i], *transformed_key_frame_point_cloud, poses_6D_[idx_loop_candidate_ + i]);
        *temp_loop_candidate_point_cloud += *transformed_key_frame_point_cloud;
      }
    }
  }

  // voxel_grid_loop_.setInputCloud(loop_current_point_cloud_);
  // voxel_grid_loop_.filter(*loop_current_point_cloud_);
  // voxel_grid_loop_.setInputCloud(loop_candidate_point_cloud_);
  // voxel_grid_loop_.filter(*loop_candidate_point_cloud_);

  aut_utils::OctreeVoxelGrid(temp_loop_current_point_cloud, loop_current_point_cloud_, 0.1);
  aut_utils::OctreeVoxelGrid(temp_loop_candidate_point_cloud, loop_candidate_point_cloud_, 0.1);
}

bool GraphOptimization::ComputeICP() {
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(0.3);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputSource(loop_current_point_cloud_);
  icp.setInputTarget(loop_candidate_point_cloud_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
  icp.align(*result);

  icp_transformation_ = icp.getFinalTransformation();

  return icp.hasConverged() && icp.getFitnessScore(0.3) < 0.1;
}

void GraphOptimization::ComputeFactor() {
  gtsam::Pose3 pose_from = gtsam::Pose3(
    (icp_transformation_ * loop_current_pose_).cast<double>()
  );

  gtsam::Pose3 pose_to = gtsam::Pose3(
    loop_candidate_pose_.cast<double>()
  );

  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    loop_factors_.push_back(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', idx_current_), gtsam::Symbol('x', idx_loop_candidate_), pose_from.between(pose_to), odometry_noise_));
    nav_graph_.AddEdge(idx_current_, idx_loop_candidate_);
  }
}

void GraphOptimization::MapThread() {
  rclcpp::Rate rate(map_frequency_);
  while (rclcpp::ok()) {
    rate.sleep();
    PublishMap();
  }
}

void GraphOptimization::PublishMap() {

  return;

  if (poses_6D_.empty()) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());

  // pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_map;
  // voxel_grid_map.setLeafSize(0.05, 0.05, 0.05);

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(0.2, 0.2, 0.2);

  state_mtx_.lock();
  int size_poses_6D = poses_6D_.size();
  state_mtx_.unlock();

  for (int i = 0; i < size_poses_6D; ++i) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_key_frame_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    state_mtx_.lock();
    // voxel_grid.setInputCloud(key_frames_point_cloud_[i]);
    // voxel_grid.filter(*transformed_key_frame_point_cloud);
    // pcl::transformPointCloud(*transformed_key_frame_point_cloud, *transformed_key_frame_point_cloud, poses_6D_[i]);
    pcl::transformPointCloud(*key_frames_point_cloud_[i], *transformed_key_frame_point_cloud, poses_6D_[i]);

    // TODO: ERROR voxel grid

    state_mtx_.unlock();

    *map += *transformed_key_frame_point_cloud;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_map(new pcl::PointCloud<pcl::PointXYZI>);

  aut_utils::OctreeVoxelGrid(map, output_map, 0.2);

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*output_map, map_msg);

  map_msg.header.frame_id = "map";

  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    map_msg.header.stamp = key_frames_headers_[size_poses_6D - 1].stamp;
  }

  map_publisher_->publish(map_msg);


  visualization_msgs::msg::MarkerArray marker_array;
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    nav_graph_.TidyGraph();
    marker_array = nav_graph_.GetMarkerArray();
  }

  marker_array_publisher_->publish(marker_array);

}

void GraphOptimization::Start(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  {
    std::lock_guard<std::mutex> lock(running_mtx_);
    running_ = true;

    response->success = true;
    response->message = std::string("Graph Optimization is running");
  }
}

void GraphOptimization::Stop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  {
    std::lock_guard<std::mutex> lock(running_mtx_);
    running_ = false;

    response->success = true;
    response->message = std::string("Graph Optimization is stopped");
  }
}

void GraphOptimization::Reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  {
    std::lock_guard<std::mutex> lock(running_mtx_);
    if (running_) {
      response->success = false;
      response->message = std::string("Graph Optimization is not stopped. Can't reset");
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    input_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    input_odom_ = Eigen::Matrix4f::Identity();
    last_key_frame_odom_ = Eigen::Matrix4f::Identity();

    corrected_input_odom_ = Eigen::Matrix4f::Identity();

    poses_3D_.reset(new pcl::PointCloud<pcl::PointXYZL>());

    kdtree_poses_3D_.reset(new pcl::KdTreeFLANN<pcl::PointXYZL>);

    loop_current_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    loop_candidate_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    loop_current_pose_ = Eigen::Matrix4f::Identity();
    loop_candidate_pose_ = Eigen::Matrix4f::Identity();
    icp_transformation_ = Eigen::Matrix4f::Identity();

    poses_6D_.clear();
    key_frames_point_cloud_.clear();
    key_frames_headers_.clear();

    nav_graph_.Reset();
    fiducials_tag_id_.clear();
    fiducials_dist_.clear();
    fiducials_base_link_to_fiducial_.clear();
    fiducials_time_.clear();
    loop_factors_.clear();

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam_ = new gtsam::ISAM2(parameters);

    graph_.resize(0);
    initial_estimate_.clear();

    n_keyframe_ = 0;

    response->success = true;
    response->message = std::string("Graph Optimization is reset");
  }

}

void GraphOptimization::SaveMap(const std::shared_ptr<aut_msgs::srv::SaveMap::Request> request, std::shared_ptr<aut_msgs::srv::SaveMap::Response> response) {

  {
    std::lock_guard<std::mutex> lock(running_mtx_);
    if (running_) {
      response->success = false;
      response->message = std::string("Graph Optimization is not stopped. Can't save");
      return;
    }
  }

  bool directory_exits = std::filesystem::exists(std::string(request->destination));
  if (!directory_exits) {
    bool directory_success = std::filesystem::create_directories(std::string(request->destination));
    if (!directory_success) {
      response->success = false;
      response->message = std::string("Can't save map, cannot create directory");
      return;
    }
  }

  RCLCPP_INFO(this->get_logger(), "SaveMap service");

  state_mtx_.lock();
  std::vector<Eigen::Matrix4f> copy_poses_6D = poses_6D_;
  std::vector<std_msgs::msg::Header> copy_key_frames_headers = key_frames_headers_;
  state_mtx_.unlock();

  if (copy_poses_6D.empty()) {
    response->success = false;
    response->message = std::string("Can't save map, it is empty");
    return;
  }

  octomap::OcTree octree(0.1); // TODO: See if 0.02 is enough
  octree.setProbHit(1.0);
  octree.setProbMiss(0.40);

  octree.setClampingThresMax(1.0);
  octree.setClampingThresMin(0.0);

  octree.setOccupancyThres(0.7);

  pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());  

  float total_displacement = 0.0f;
  for (int i = 0; i < (int) copy_poses_6D.size(); ++i) {
    if (i > 0) {
      total_displacement += (copy_poses_6D[i - 1].block<3, 1>(0, 3) - copy_poses_6D[i].block<3, 1>(0, 3)).norm();
    }
  }

  RCLCPP_INFO(this->get_logger(), "Total displacement : %f", total_displacement);

  for (int i = 0; i < (int) copy_poses_6D.size(); ++i) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_key_frame_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    state_mtx_.lock();
    pcl::transformPointCloud(*key_frames_point_cloud_[i], *transformed_key_frame_point_cloud, copy_poses_6D[i]);
    state_mtx_.unlock();

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_key_frame_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    aut_utils::OctreeVoxelGrid(transformed_key_frame_point_cloud, filtered_key_frame_point_cloud, 0.01);

    *map += *filtered_key_frame_point_cloud;

    octomap::Pointcloud* p = new octomap::Pointcloud();
    for (int j = 0; j < (int) transformed_key_frame_point_cloud->points.size(); ++j) {
      octomap::point3d point(
        transformed_key_frame_point_cloud->points[j].x, 
        transformed_key_frame_point_cloud->points[j].y, 
        transformed_key_frame_point_cloud->points[j].z
      );
      p->push_back(point);
    }

    Eigen::Quaternionf quat(copy_poses_6D[i].block<3, 3>(0, 0));
    octomath::Quaternion quat_octo(quat.w(), quat.x(), quat.y(), quat.z());

    octomath::Vector3 trans(copy_poses_6D[i](0, 3), copy_poses_6D[i](1, 3), copy_poses_6D[i](2, 3));
    octomap::pose6d frame(trans, quat_octo);

    octree.insertPointCloud(*p, frame.trans(), 15, false, true);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZI>());  

  aut_utils::OctreeVoxelGrid(map, filtered_map, request->resolution);

  pcl::io::savePCDFileASCII(std::string(request->destination) + std::string("/map.pcd"), *filtered_map);

  octree.writeBinary(std::string(request->destination) + std::string("/map.bt"));

  std::vector<int> fiducials_tag_id_map;
  std::vector<Eigen::Matrix4f> fiducials_map_to_fiducial;

  {
    std::lock_guard<std::mutex> lock(fiducial_mtx_);
    for (const auto& tag_id : fiducials_tag_id_) {

      double fiducial_seconds = rclcpp::Time(fiducials_time_[tag_id]).seconds();

      for (int i = 0; i < (int) copy_poses_6D.size() - 1; ++i) {

        double before_seconds = rclcpp::Time(copy_key_frames_headers[i].stamp).seconds();
        double after_seconds = rclcpp::Time(copy_key_frames_headers[i + 1].stamp).seconds();

        if (before_seconds > fiducial_seconds || fiducial_seconds >= after_seconds) {
          continue;
        }

        Eigen::Matrix4f before_map_to_base_link = copy_poses_6D[i];
        Eigen::Matrix4f after_map_to_base_link = copy_poses_6D[i + 1];

        float alpha = (fiducial_seconds - before_seconds) / (after_seconds - before_seconds);

        Eigen::Quaternionf before_quat(before_map_to_base_link.block<3, 3>(0, 0));
        Eigen::Quaternionf after_quat(after_map_to_base_link.block<3, 3>(0, 0));

        Eigen::Vector3f before_trans(before_map_to_base_link.block<3, 1>(0, 3));
        Eigen::Vector3f after_trans(after_map_to_base_link.block<3, 1>(0, 3));

        Eigen::Matrix4f fiducial_map_to_base_link = Eigen::Matrix4f::Identity();
        fiducial_map_to_base_link.block<3, 3>(0, 0) = before_quat.slerp(alpha, after_quat).toRotationMatrix();
        fiducial_map_to_base_link.block<3, 1>(0, 3) = (1 - alpha) * before_trans + alpha * after_trans;

        fiducials_tag_id_map.push_back(tag_id);
        fiducials_map_to_fiducial.push_back(fiducial_map_to_base_link * fiducials_base_link_to_fiducial_[tag_id]);

        break;
      }
    }
  }

  std::ofstream file_fiducials(std::string(request->destination) + std::string("/fiducials.txt"));
  file_fiducials << fiducials_tag_id_map.size() << "\n";

  for (int i = 0; i < (int) fiducials_tag_id_map.size(); ++i) {
    file_fiducials << fiducials_tag_id_map[i] << "\n";
    file_fiducials << fiducials_map_to_fiducial[i] << "\n";
  }

  file_fiducials.close();

  nav_graph_.SaveFile(std::string(request->destination) + std::string("/graph.txt"));

  response->success = true;
  response->message = std::string("");
}


}  // namespace aut_slam