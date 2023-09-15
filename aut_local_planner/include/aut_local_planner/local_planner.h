#ifndef AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_
#define AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_

#include <vector>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Core>

#include "aut_common/transformer.h"
#include "aut_msgs/msg/local_grid.hpp"
#include "aut_msgs/srv/set_global_path.hpp"
#include "aut_msgs/msg/speed_command.hpp"
#include "aut_msgs/srv/graph_modif.hpp"
#include "aut_local_planner/a_star.h"

namespace aut_local_planner {

class LocalPlanner : public rclcpp::Node {

 public:
  explicit LocalPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
 
 private:
  void LocalGridCallBack(const aut_msgs::msg::LocalGrid::SharedPtr local_grid_msg);
  aut_msgs::msg::SpeedCommand GetSpeedCommand(std::vector<Eigen::Vector2f>& local_path);
  void SetGlobalPathService(
      const std::shared_ptr<aut_msgs::srv::SetGlobalPath::Request> request, 
      std::shared_ptr<aut_msgs::srv::SetGlobalPath::Response> response);

  nav_msgs::msg::OccupancyGrid LocalGridToOccupancyGrid(
      std::vector<float>& local_grid, 
      geometry_msgs::msg::Transform& base_link_to_local_grid);
  
  // State
  std::atomic_bool path_is_set_;
  std::vector<Eigen::Matrix4f> global_path_poses_;
  int count_of_failed;
  int last_target_idx_;

  std::vector<Eigen::Matrix4f>::iterator closest_pose_it_;

  // A*
  AStar a_star_;

  // tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<aut_common::Transformer> transformer_;

  // Subscriptions
  rclcpp::Subscription<aut_msgs::msg::LocalGrid>::SharedPtr local_grid_subscription_;

  // Services
  rclcpp::Service<aut_msgs::srv::SetGlobalPath>::SharedPtr set_global_path_service_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr local_grid_callback_group_;
  rclcpp::CallbackGroup::SharedPtr set_global_path_callback_group_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  rclcpp::Publisher<aut_msgs::msg::SpeedCommand>::SharedPtr speed_command_publisher_;

  // Service Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr goal_achieved_client_;
  rclcpp::Client<aut_msgs::srv::GraphModif>::SharedPtr graph_modif_client_;

  // Mutex
  std::mutex global_path_mtx_;
};

}  // namespace aut_local_planner

#endif  // AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_

// #ifndef AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_
// #define AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_

// #include <mutex>
// #include <vector>
// #include <deque>

// #include <rclcpp/rclcpp.hpp>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <geometry_msgs/msg/transform.hpp>
// #include <visualization_msgs/msg/marker.hpp>

// #include <aut_msgs/msg/local_grid.hpp>
// #include <aut_msgs/msg/nav.hpp>
// #include <aut_msgs/msg/nav_modif.hpp>
// #include <aut_msgs/msg/nav_command.hpp>

// #include "aut_common/transformer.h"
// #include <aut_local_planner/local_grid.h>

// namespace aut_local_planner {

// class LocalPlanner : public rclcpp::Node {

//  public:
//   explicit LocalPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

//  private:
//   void LocalGridCallBack(const aut_msgs::msg::LocalGrid::SharedPtr local_grid_msg);
//   void NavCallBack(const aut_msgs::msg::Nav::SharedPtr nav_msg);

//   // State
//   LocalGrid local_grid_;

//   // Publisher
//   rclcpp::Publisher<aut_msgs::msg::NavCommand>::SharedPtr nav_command_publisher_;
//   rclcpp::Publisher<aut_msgs::msg::NavModif>::SharedPtr nav_modif_publisher_;
//   rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
//   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_command_publisher_;

//   // Subscription
//   rclcpp::Subscription<aut_msgs::msg::LocalGrid>::SharedPtr local_grid_subscription_;
//   rclcpp::Subscription<aut_msgs::msg::Nav>::SharedPtr nav_subscription_;

//   // Callback groups
//   rclcpp::CallbackGroup::SharedPtr callback_group_local_grid_;
//   rclcpp::CallbackGroup::SharedPtr callback_group_nav_;

//   // tf2 Listener
//   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//   // tf buffer
//   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

//   // Transformer
//   std::unique_ptr<aut_common::Transformer> transformer_;

//   // Mutex
//   std::mutex state_mtx_;

// };

// }  // namespace aut_local_planner

// #endif  // AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_