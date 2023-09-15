#ifndef AUT_LOCALIZATION_MONTE_CARLO_LOCALIZATION_H_
#define AUT_LOCALIZATION_MONTE_CARLO_LOCALIZATION_H_

#include <mutex>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <string>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <dynamicEDT3D/dynamicEDT3D.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTree.h>

#include "aut_msgs/msg/point_cloud_with_pose.hpp"
#include "aut_msgs/msg/fiducial.hpp"
#include "aut_msgs/srv/load_map.hpp"

namespace aut_localization {

class MonteCarloLocalization : public rclcpp::Node {

 public:
  explicit MonteCarloLocalization(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());  // TODO: See if we need to put "= default" or "= delete" 

  void Start(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void Stop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void LoadMapService(const std::shared_ptr<aut_msgs::srv::LoadMap::Request> request, std::shared_ptr<aut_msgs::srv::LoadMap::Response> response);

 private:

  void PointCloudWithPoseCallBack(const aut_msgs::msg::PointCloudWithPose::SharedPtr point_cloud_with_pose_msg);
  void FiducialCallBack(const aut_msgs::msg::Fiducial::SharedPtr fiducial_msg);
  void LoadMap(std::string map_path);
  void Correct(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  void Resample();
  float RandomNumber();
  Eigen::Matrix4f RandomMatrix(float std_x, float std_y, float std_z, float std_roll, float std_pitch, float std_yaw);

  // Parameters

  bool running_;
  bool map_loaded_;

  int n_particles_;

  float sigma_hit_;
  float z_rand_;
  float z_hit_;
  float max_range_;

  bool fiducials_enabled_;

  // State
  bool found_fiducial_;
  aut_msgs::msg::Fiducial init_fiducial_;

  bool initialized_;

  std::string destination;

  Eigen::Matrix4f last_odom_to_base_link_;

  // Octree & DynamicEDT3D
  std::shared_ptr<octomap::OcTree> map_octree;
  std::shared_ptr<DynamicEDTOctomap> map_edt;

  // Particles
  std::vector<Eigen::Matrix4f> particles_;
  std::vector<float> weights_;

  // Fiducials
  std::unordered_set<int> tag_ids;
  std::unordered_map<int, Eigen::Matrix4f> map_to_fiducials;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_publisher_;

  // Subscription
  rclcpp::Subscription<aut_msgs::msg::PointCloudWithPose>::SharedPtr point_cloud_with_pose_subscription_;
  rclcpp::Subscription<aut_msgs::msg::Fiducial>::SharedPtr fiducial_subscription_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr callback_group_point_cloud_with_pose_;
  rclcpp::CallbackGroup::SharedPtr callback_group_fiducial_;

  // tf2 Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // tf2 Listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // tf buffer
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Services
  rclcpp::Service<aut_msgs::srv::LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;

  // Mutex
  std::mutex fiducial_mtx_;
  std::mutex running_mtx_;

  // Random
  std::mt19937 generator_;
};

}  // namespace aut_localization

#endif  // AUT_LOCALIZATION_MONTE_CARLO_LOCALIZATION_H_