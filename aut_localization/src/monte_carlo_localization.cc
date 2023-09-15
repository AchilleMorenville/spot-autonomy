#include "aut_localization/monte_carlo_localization.h"

#include <memory>
#include <string>
#include <fstream>
#include <random>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/common/transforms.h>

#include <Eigen/Core>

#include <dynamicEDT3D/dynamicEDT3D.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTree.h>

#include "aut_msgs/msg/point_cloud_with_pose.hpp"
#include "aut_msgs/msg/fiducial.hpp"
#include "aut_utils/utils.h"
#include "aut_msgs/srv/load_map.hpp"

namespace aut_localization {

MonteCarloLocalization::MonteCarloLocalization(const rclcpp::NodeOptions& options)
    : Node("monte_carlo_localization", options), generator_(std::random_device()()) {

  running_ = false;
  map_loaded_ = false;

  this->declare_parameter("n_particles", 1000);
  n_particles_ = this->get_parameter("n_particles").get_parameter_value().get<int>();

  found_fiducial_ = false;
  initialized_ = false;

  fiducials_enabled_ = false;

  destination = std::string("/ros2_ws/src/old/data");
  
  callback_group_point_cloud_with_pose_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );

  rclcpp::SubscriptionOptions point_cloud_with_pose_options = rclcpp::SubscriptionOptions();
  point_cloud_with_pose_options.callback_group = callback_group_point_cloud_with_pose_;

  point_cloud_with_pose_subscription_ = this->create_subscription<aut_msgs::msg::PointCloudWithPose>(
    "aut_lidar_odometry/point_cloud_with_pose", 10, 
    std::bind(&MonteCarloLocalization::PointCloudWithPoseCallBack, this, std::placeholders::_1),
    point_cloud_with_pose_options
  );

  callback_group_fiducial_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );

  rclcpp::SubscriptionOptions fiducial_options = rclcpp::SubscriptionOptions();
  fiducial_options.callback_group = callback_group_fiducial_;

  fiducial_subscription_ = this->create_subscription<aut_msgs::msg::Fiducial>(
    "aut_spot/fiducial", 10, 
    std::bind(&MonteCarloLocalization::FiducialCallBack, this, std::placeholders::_1),
    fiducial_options
  );

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pose_array_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("aut_localization/pose_array", 10);

  load_map_service_ = this->create_service<aut_msgs::srv::LoadMap>("aut_localization/load_map", std::bind(&MonteCarloLocalization::LoadMapService, this, std::placeholders::_1, std::placeholders::_2));
  stop_service_ = this->create_service<std_srvs::srv::Trigger>("aut_localization/stop", std::bind(&MonteCarloLocalization::Stop, this, std::placeholders::_1, std::placeholders::_2));
  start_service_ = this->create_service<std_srvs::srv::Trigger>("aut_localization/start", std::bind(&MonteCarloLocalization::Start, this, std::placeholders::_1, std::placeholders::_2));


  // Memory allocation

  particles_.reserve(n_particles_);
  weights_.reserve(n_particles_);

  last_odom_to_base_link_ = Eigen::Matrix4f::Identity();

  // LoadMap(destination);
}

void MonteCarloLocalization::LoadMap(std::string map_path) {

  map_octree = std::shared_ptr<octomap::OcTree>(new octomap::OcTree(0.1));
  map_octree->readBinary(map_path + std::string("/map.bt"));

  double x,y,z;
  map_octree->getMetricMin(x,y,z);
  octomap::point3d min(x,y,z);
  map_octree->getMetricMax(x,y,z);
  octomap::point3d max(x,y,z);

  map_edt = std::shared_ptr<DynamicEDTOctomap>(new DynamicEDTOctomap(1.0, &(*map_octree), min, max, false));
  map_edt->update();

  std::ifstream file_fiducials(map_path + std::string("/fiducials.txt"));

  int nbr;
  file_fiducials >> nbr;

  int tag_id;
  Eigen::Matrix4f map_to_fiducial;

  for (int i = 0; i < nbr; ++i) {

    file_fiducials >> tag_id;
    for (int m = 0; m < 4; ++m) {
      for (int n = 0; n < 4; ++n) {
        file_fiducials >> map_to_fiducial(m, n);
      }
    }

    tag_ids.insert(tag_id);
    map_to_fiducials[tag_id] = map_to_fiducial;

  }
  
}

void MonteCarloLocalization::PointCloudWithPoseCallBack(const aut_msgs::msg::PointCloudWithPose::SharedPtr point_cloud_with_pose_msg) {

  {
    std::lock_guard<std::mutex> lock(running_mtx_);
    if (!running_ || !map_loaded_) {
      return;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Received point cloud");

  // Save input
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(point_cloud_with_pose_msg->point_cloud, *input_point_cloud);

  {
    std::lock_guard<std::mutex> lock(fiducial_mtx_);
    if (!found_fiducial_) {
      return;
    }
  }

  if (!initialized_) {

    Eigen::Matrix4f base_link_to_fiducial = aut_utils::TransformToMatrix(
      init_fiducial_.pose
    );

    Eigen::Matrix4f fiducial_to_base_link = aut_utils::InverseTransformation(base_link_to_fiducial);

    Eigen::Matrix4f odom_to_base_link_current = aut_utils::TransformToMatrix(
      point_cloud_with_pose_msg->pose
    );

    // Eigen::Matrix4f map_to_base_link_current = Eigen::Matrix4f::Identity() * odom_to_base_link_current;

    Eigen::Matrix4f map_to_base_link = map_to_fiducials[init_fiducial_.tag_id] * fiducial_to_base_link;

    particles_.clear();
    weights_.clear();

    for (int i = 0; i < n_particles_; ++i) {
      // particles_.push_back(map_to_base_link_current * RandomMatrix(0.5, 0.5, 0.5, 0.3, 0.3, 0.3));
      // particles_.push_back(map_to_base_link_current);
      particles_.push_back(map_to_base_link * RandomMatrix(0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
      weights_.push_back(1.0f / n_particles_);
    }

    initialized_ = true;

    last_odom_to_base_link_ = odom_to_base_link_current;
  } else {

    Eigen::Matrix4f odom_to_base_link_current = aut_utils::TransformToMatrix(
      point_cloud_with_pose_msg->pose
    );

    Eigen::Matrix4f displacement = aut_utils::GetDifferenceTransformation(last_odom_to_base_link_, odom_to_base_link_current);

    for (int i = 0; i < n_particles_; ++i) {
      particles_[i] = particles_[i] * displacement * RandomMatrix(0.005, 0.005, 0.005, 0.02, 0.02, 0.02);
      // particles_[i] = particles_[i] * displacement;
    }

    last_odom_to_base_link_ = odom_to_base_link_current;
  }

  Correct(input_point_cloud);

  Eigen::Matrix4f best_particles = Eigen::Matrix4f::Identity();

  float max_weight = -std::numeric_limits<float>::max();
  for (int i = 0; i < n_particles_; ++i) {
    if (max_weight < weights_[i]) {
      max_weight = weights_[i];
      best_particles = particles_[i];
    }
  }

  std::cout << "Max weight : " << max_weight << std::endl;

  Eigen::Matrix4f odom_to_base_link_current = aut_utils::TransformToMatrix(
    point_cloud_with_pose_msg->pose
  );

  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = point_cloud_with_pose_msg->header.stamp;
  t.header.frame_id = "l_odom";
  t.child_frame_id = "map";

  Eigen::Matrix4f map_to_odom = aut_utils::InverseTransformation(best_particles * aut_utils::InverseTransformation(odom_to_base_link_current));

  t.transform.translation.x = map_to_odom(0, 3);
  t.transform.translation.y = map_to_odom(1, 3);
  t.transform.translation.z = map_to_odom(2, 3);

  Eigen::Quaternionf quat(map_to_odom.block<3, 3>(0, 0));

  t.transform.rotation.w = quat.w();
  t.transform.rotation.x = quat.x();
  t.transform.rotation.y = quat.y();
  t.transform.rotation.z = quat.z();

  tf_broadcaster_->sendTransform(t); 

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "map";

  for (int i = 0; i < n_particles_; ++i) {
    pose_array.poses.push_back(aut_utils::MatrixToPose(particles_[i]));
  }

  pose_array_publisher_->publish(pose_array);

  Resample();

}

Eigen::Matrix4f MonteCarloLocalization::RandomMatrix(float std_x, float std_y, float std_z, float std_roll, float std_pitch, float std_yaw) {
  std::normal_distribution<float> d_x{0.0, std_x};
  std::normal_distribution<float> d_y{0.0, std_y};
  std::normal_distribution<float> d_z{0.0, std_z};
  std::normal_distribution<float> d_roll{0.0, std_roll};
  std::normal_distribution<float> d_pitch{0.0, std_pitch};
  std::normal_distribution<float> d_yaw{0.0, std_yaw};

  float x = d_x(generator_);
  float y = d_y(generator_);
  float z = d_z(generator_);
  float roll = d_roll(generator_);
  float pitch = d_pitch(generator_);
  float yaw = d_yaw(generator_);

  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose(0, 3) = x;
  pose(1, 3) = y;
  pose(2, 3) = z;

  return pose;
}

float MonteCarloLocalization::RandomNumber() {
  std::uniform_real_distribution<float> dis(0.0, 1.0);
  return dis(generator_);
}

void MonteCarloLocalization::Correct(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr reduced_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  aut_utils::OctreeVoxelGrid(cloud, reduced_cloud, 0.2);

  for (int i = 0; i < n_particles_; ++i) { // Compute the weights

    // Transform point cloud to be able to perform directions
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*reduced_cloud, *cloud_trans, particles_[i]);

    octomap::point3d origin(particles_[i](0, 3), particles_[i](1, 3), particles_[i](2, 3));

    float w = 0;

    for (int j = 0; j < static_cast<int>(cloud_trans->points.size()); ++j) {

      // float range = (particles_[i](0, 3) - cloud_trans->points[j].x) * (particles_[i](0, 3) - cloud_trans->points[j].x) + (particles_[i](1, 3) - cloud_trans->points[j].y) * (particles_[i](1, 3) - cloud_trans->points[j].y) + (particles_[i](2, 3) - cloud_trans->points[j].z) * (particles_[i](2, 3) - cloud_trans->points[j].z);

      octomap::point3d endPoint(cloud_trans->points[j].x, cloud_trans->points[j].y, cloud_trans->points[j].z);
      float dist = map_edt->getDistance(endPoint);

      if (dist > 0.0f){ // endpoint is inside map:
        if (dist >= 0.5) {
          w += std::log(0.1);
        } else {
          w += std::log(std::exp(- 0.5f * dist * dist / (0.1 * 0.1)) * 0.9f + 0.1f);
        }
      } 
    }
    weights_[i] = w;
  }
}



void MonteCarloLocalization::Resample() {
  // Normalize weigths
  float max_weight = -std::numeric_limits<float>::max();

  for (int i = 0; i < static_cast<int>(weights_.size()); ++i) {
    if (weights_[i] > max_weight) {
      max_weight = weights_[i];
    }
  }

  for (int i = 0; i < static_cast<int>(weights_.size()); ++i) {
    weights_[i] = std::exp(weights_[i] - max_weight);
  }

  float total_weights = 0.0;
  for (int i = 0; i < n_particles_; ++i) {
    total_weights += weights_[i];
  }

  for (int i = 0; i < n_particles_; ++i) {
    weights_[i] /= total_weights;
  }

  float interval = 1.0 / n_particles_;

  float r = interval * RandomNumber();

  double c = weights_[0];
  std::vector<int> indices;

  int n = 0;
  for (int i = 0; i < n_particles_; ++i){
    float u = r + i * interval;
    while (u > c && n < n_particles_){
      n = n + 1;
      c = c + weights_[n];
    }
    indices.push_back(n);
  }

  std::vector<Eigen::Matrix4f> old_particles;
  old_particles = particles_;

  particles_.clear();
  weights_.clear();

  // Particle generation
  for (int i = 0; i < n_particles_; ++i) {
    particles_.push_back(old_particles[indices[i]]);
    weights_.push_back(1.0 / n_particles_);
  }
}

void MonteCarloLocalization::FiducialCallBack(const aut_msgs::msg::Fiducial::SharedPtr fiducial_msg) {

  if (found_fiducial_) {
    return;
  }

  if (!tf_buffer_->canTransform("v_odom", "base_link", fiducial_msg->header.stamp)) {
    return;
  }

  if (!tf_buffer_->canTransform("l_odom", "v_odom", fiducial_msg->header.stamp)) {
    if (!tf_buffer_->canTransform("l_odom", "v_odom", tf2::TimePointZero)) {
      return;
    }
  }

  float squared_dist = fiducial_msg->pose.translation.x * fiducial_msg->pose.translation.x
                       + fiducial_msg->pose.translation.y * fiducial_msg->pose.translation.y
                       + fiducial_msg->pose.translation.z * fiducial_msg->pose.translation.z;

  if (squared_dist > 9.0f || tag_ids.find(fiducial_msg->tag_id) == tag_ids.end()) {
    return;
  }

  fiducial_mtx_.lock();
  found_fiducial_ = true;
  init_fiducial_ = *fiducial_msg;
  fiducial_mtx_.unlock();
}

void MonteCarloLocalization::Start(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;

  {
    std::lock_guard<std::mutex> lock(running_mtx_);

    if (!map_loaded_) {
      response->success = false;
      response->message = std::string("Cannot start Localization if no map is loaded");
      return;
    }

    running_ = true;
    response->success = true;
    response->message = std::string("Localization is running");
  }
}

void MonteCarloLocalization::Stop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;

  {
    std::lock_guard<std::mutex> lock(running_mtx_);
    running_ = false;

    response->success = true;
    response->message = std::string("Localization is stopped");
  }
}

void MonteCarloLocalization::LoadMapService(const std::shared_ptr<aut_msgs::srv::LoadMap::Request> request, std::shared_ptr<aut_msgs::srv::LoadMap::Response> response) {

  (void)request;

  {
    std::lock_guard<std::mutex> lock(running_mtx_);
    if (running_) {
      response->success = false;
      response->message = std::string("Can't load map because localization is running");
      return;
    }
  }

  initialized_ = false;

  last_odom_to_base_link_ = Eigen::Matrix4f::Identity();
  particles_.clear();
  weights_.clear();
  tag_ids.clear();
  map_to_fiducials.clear();

  LoadMap(std::string(request->map_directory_path));

  {
    std::lock_guard<std::mutex> lock(running_mtx_);
    map_loaded_ = true;
  }
  response->success = true;
  response->message = std::string("Map loaded");
}



// void MonteCarloLocalization::PointCloudWithPoseCallBack(const aut_msgs::msg::PointCloudWithPose::SharedPtr point_cloud_with_pose_msg) {
//   if (!found_fiducial_ && fiducials_enabled_) {
//     return;
//   }

//   RCLCPP_INFO(this->get_logger(), "Received point cloud");

//   // Save input
//   pcl::PointCloud<pcl::PointXYZI>::Ptr input_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
//   pcl::fromROSMsg(point_cloud_with_pose_msg->point_cloud, *input_point_cloud);

//   if (!initialized_) {
//     if (!fiducials_enabled_) {

//       Eigen::Matrix4f odom_to_base_link_current = aut_utils::TransformToMatrix(
//         point_cloud_with_pose_msg->pose
//       );

//       particles_.clear();
//       weights_.clear();

//       for (int i = 0; i < n_particles_; ++i) {
//         // particles_.push_back(odom_to_base_link_current * RandomMatrix(0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
//         particles_.push_back(RandomMatrix(0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
//         weights_.push_back(1.0f / n_particles_);
//       }

//       initialized_ = true;

//       last_odom_to_base_link_ = odom_to_base_link_current;
//     } else {
//       fiducial_mtx_.lock();
//       Eigen::Matrix4f base_link_to_fiducial = aut_utils::TransformToMatrix(init_fiducial_.pose);
//       fiducial_mtx_.unlock();

//       geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("base_link", "odom", init_fiducial_.header.stamp).transform;

//       Eigen::Matrix4f base_link_to_odom_fiducial = aut_utils::TransformToMatrix(
//         trans
//       );

//       Eigen::Matrix4f odom_to_base_link_current = aut_utils::TransformToMatrix(
//         point_cloud_with_pose_msg->pose
//       );

//       Eigen::Matrix4f fiducial_to_base_link_current = (aut_utils::InverseTransformation(base_link_to_fiducial) * base_link_to_odom_fiducial) * odom_to_base_link_current;

//       Eigen::Matrix4f map_to_base_link_current = map_to_fiducials[init_fiducial_.tag_id] * fiducial_to_base_link_current;

//       particles_.clear();
//       weights_.clear();

//       for (int i = 0; i < n_particles_; ++i) {
//         particles_.push_back(map_to_base_link_current * RandomMatrix(0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
//         weights_.push_back(1.0f / n_particles_);
//       }

//       initialized_ = true;

//       last_odom_to_base_link_ = odom_to_base_link_current;
//     }
//   } else {

//     Eigen::Matrix4f odom_to_base_link_current = aut_utils::TransformToMatrix(
//       point_cloud_with_pose_msg->pose
//     );

//     Eigen::Matrix4f displacement = aut_utils::GetDifferenceTransformation(last_odom_to_base_link_, odom_to_base_link_current);

//     for (int i = 0; i < n_particles_; ++i) {
//       particles_[i] *= displacement * RandomMatrix(0.01, 0.01, 0.01, 0.05, 0.05, 0.05);
//     }

//     last_odom_to_base_link_ = odom_to_base_link_current;

//   }

//   Correct(input_point_cloud);

//   Eigen::Matrix4f best_particles = Eigen::Matrix4f::Identity();

//   float max_weight = -std::numeric_limits<float>::max();
//   for (int i = 0; i < n_particles_; ++i) {
//     if (max_weight < weights_[i]) {
//       max_weight = weights_[i];
//       best_particles = particles_[i];
//     }
//   }

//   Eigen::Matrix4f odom_to_base_link_current = aut_utils::TransformToMatrix(
//     point_cloud_with_pose_msg->pose
//   );

//   geometry_msgs::msg::TransformStamped t;

//   t.header.stamp = point_cloud_with_pose_msg->header.stamp;
//   t.header.frame_id = "map";
//   t.child_frame_id = "odom";

//   Eigen::Matrix4f map_to_odom = best_particles * aut_utils::InverseTransformation(odom_to_base_link_current);

//   t.transform.translation.x = map_to_odom(0, 3);
//   t.transform.translation.y = map_to_odom(1, 3);
//   t.transform.translation.z = map_to_odom(2, 3);

//   Eigen::Quaternionf quat(map_to_odom.block<3, 3>(0, 0));

//   t.transform.rotation.w = quat.w();
//   t.transform.rotation.x = quat.x();
//   t.transform.rotation.y = quat.y();
//   t.transform.rotation.z = quat.z();

//   tf_broadcaster_->sendTransform(t); 

//   Resample();

// }

// Eigen::Matrix4f MonteCarloLocalization::RandomMatrix(float std_x, float std_y, float std_z, float std_roll, float std_pitch, float std_yaw) {
//   std::normal_distribution<float> d_x{0.0, std_x};
//   std::normal_distribution<float> d_y{0.0, std_y};
//   std::normal_distribution<float> d_z{0.0, std_z};
//   std::normal_distribution<float> d_roll{0.0, std_roll};
//   std::normal_distribution<float> d_pitch{0.0, std_pitch};
//   std::normal_distribution<float> d_yaw{0.0, std_yaw};

//   float x = d_x(generator_);
//   float y = d_y(generator_);
//   float z = d_z(generator_);
//   float roll = d_roll(generator_);
//   float pitch = d_pitch(generator_);
//   float yaw = d_yaw(generator_);

//   Eigen::Quaternionf q;
//   q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
//       * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
//       * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

//   Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
//   pose.block<3, 3>(0, 0) = q.toRotationMatrix();
//   pose(0, 3) = x;
//   pose(1, 3) = y;
//   pose(2, 3) = z;

//   return pose;
// }

// float MonteCarloLocalization::RandomNumber() {
//   std::uniform_real_distribution<float> dis(0.0, 1.0);
//   return dis(generator_);
// }

// void MonteCarloLocalization::Correct(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {

//   pcl::PointCloud<pcl::PointXYZI>::Ptr reduced_cloud(new pcl::PointCloud<pcl::PointXYZI>());
//   aut_utils::OctreeVoxelGrid(cloud, reduced_cloud, 0.2);

//   float coeff_1 = - std::log(sigma_hit_) - std::log(std::sqrt(2 * M_PI));
//   float coeff_2 = - 1.0 / (2.0 * sigma_hit_ * sigma_hit_);
//   float min_log_p = std::log(z_rand_ / max_range_);

//   for (int i = 0; i < n_particles_; ++i) { // Compute the weights

//     // Transform point cloud to be able to perform directions
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::transformPointCloud(*reduced_cloud, *cloud_trans, particles_[i]);

//     octomap::point3d origin(particles_[i](0, 3), particles_[i](1, 3), particles_[i](2, 3));

//     float w = 0;

//     for (int j = 0; j < cloud_trans->points.size(); ++j) {

//       float range = (particles_[i](0, 3) - cloud_trans->points[j].x) * (particles_[i](0, 3) - cloud_trans->points[j].x) + (particles_[i](1, 3) - cloud_trans->points[j].y) * (particles_[i](1, 3) - cloud_trans->points[j].y) + (particles_[i](2, 3) - cloud_trans->points[j].z) * (particles_[i](2, 3) - cloud_trans->points[j].z);

//       if (range > max_range_ * max_range_) {
//         continue;
//       }

//       octomap::point3d endPoint(cloud_trans->points[j].x, cloud_trans->points[j].y, cloud_trans->points[j].z);
//       float dist = map_edt->getDistance(endPoint);

//       if (dist > 0.0){ // endpoint is inside map:
//         float log_p = coeff_1 + coeff_2 * (dist * dist); 
//         if (log_p < min_log_p) {
//           w += min_log_p;
//         } else {
//           w += log_p;
//         }
//       } 
//     }
//     weights_[i] = w;
//   }
// }

// void MonteCarloLocalization::Resample() {
//   // Normalize weigths
//   float max_weight = -std::numeric_limits<float>::max();

//   for (int i = 0; i < weights_.size(); ++i) {
//     if (weights_[i] > max_weight) {
//       max_weight = weights_[i];
//     }
//   }

//   for (int i = 0; i < weights_.size(); ++i) {
//     weights_[i] = std::exp(weights_[i] - max_weight);
//   }

//   float total_weights = 0.0;
//   for (int i = 0; i < n_particles_; ++i) {
//     total_weights += weights_[i];
//   }

//   for (int i = 0; i < n_particles_; ++i) {
//     weights_[i] /= total_weights;
//   }

//   float interval = 1.0 / n_particles_;

//   float r = interval * RandomNumber();

//   double c = weights_[0];
//   std::vector<int> indices;

//   int n = 0;
//   for (int i = 0; i < n_particles_; ++i){
//     float u = r + i * interval;
//     while (u > c && n < n_particles_){
//       n = n + 1;
//       c = c + weights_[n];
//     }
//     indices.push_back(n);
//   }

//   std::vector<Eigen::Matrix4f> old_particles;
//   old_particles = particles_;

//   particles_.clear();
//   weights_.clear();

//   // Particle generation
//   for (int i = 0; i < n_particles_; ++i) {
//     particles_.push_back(old_particles[indices[i]]);
//     weights_.push_back(1.0 / n_particles_);
//   }
// }


// void MonteCarloLocalization::FiducialCallBack(const aut_msgs::msg::Fiducial::SharedPtr fiducial_msg) {

//   if (found_fiducial_) {
//     return;
//   }

//   if (!tf_buffer_->canTransform("odom", "base_link", fiducial_msg->header.stamp)) {
//     return;
//   }

//   float squared_dist = fiducial_msg->pose.translation.x * fiducial_msg->pose.translation.x
//                        + fiducial_msg->pose.translation.y * fiducial_msg->pose.translation.y
//                        + fiducial_msg->pose.translation.z * fiducial_msg->pose.translation.z;

//   if (squared_dist > 9.0f || tag_ids.find(fiducial_msg->tag_id) == tag_ids.end()) {
//     return;
//   }

//   fiducial_mtx_.lock();
//   found_fiducial_ = true;
//   init_fiducial_ = *fiducial_msg;
//   fiducial_mtx_.unlock();
// }




}  // namespace aut_localization