#ifndef AUT_GLOBAL_PLANNER_GLOBAL_PLANNER_H_
#define AUT_GLOBAL_PLANNER_GLOBAL_PLANNER_H_

#include <memory>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/msg/marker_array.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Core>

#include <aut_common/graph.h>
#include <aut_common/transformer.h>
#include <aut_msgs/action/navigate_to_goal.hpp>
#include <aut_msgs/srv/load_map.hpp>
#include <aut_msgs/srv/set_global_path.hpp>
#include <aut_msgs/srv/graph_modif.hpp>
#include <aut_msgs/msg/nav.hpp>
#include <aut_msgs/msg/nav_modif.hpp>

namespace aut_global_planner {

class GlobalPlanner : public rclcpp::Node {

 public:
  explicit GlobalPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const aut_msgs::action::NavigateToGoal::Goal> goal
  );

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle
  );

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle
  );

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle
  );

  void AbortNavigation(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle,
      std::string abort_msg);
  void ResetNavigation();
  bool CreatPath(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle,
      const Eigen::Matrix4f& start_pose, 
      const Eigen::Matrix4f& goal_pose, 
      std::vector<Eigen::Matrix4f>& path_poses);
  bool SendPathToLocalPlanner(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle,
      const std::vector<Eigen::Matrix4f>& path_poses);
    
  void LoadMapService(const std::shared_ptr<aut_msgs::srv::LoadMap::Request> request, std::shared_ptr<aut_msgs::srv::LoadMap::Response> response);
  void GraphModifService(const std::shared_ptr<aut_msgs::srv::GraphModif::Request> request, std::shared_ptr<aut_msgs::srv::GraphModif::Response> response);
  void GoalAchievedService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // State

  std::string path_to_graph_;

  std::atomic_bool goal_achieved_;
  std::atomic_bool navigation_executing_;
  std::atomic_bool nav_graph_has_changed_;
  std::atomic_bool map_loaded_;
  
  int inaccessible_node_in_path_;

  aut_common::Graph nav_graph_;

  // tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<aut_common::Transformer> transformer_;
  
  // Publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

  // Action server
  rclcpp_action::Server<aut_msgs::action::NavigateToGoal>::SharedPtr action_server_;

  // Services
  rclcpp::Service<aut_msgs::srv::LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<aut_msgs::srv::GraphModif>::SharedPtr graph_modif_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goal_achieved_service_;

  // Service Clients
  rclcpp::Client<aut_msgs::srv::SetGlobalPath>::SharedPtr set_global_path_client_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr load_map_callback_group_;
  rclcpp::CallbackGroup::SharedPtr graph_modif_callback_group_;
  rclcpp::CallbackGroup::SharedPtr goal_achieved_callback_group_;

  // Mutex
  std::mutex state_mtx_;
};

}  // namespace aut_global_planner

#endif  // AUT_GLOBAL_PLANNER_GLOBAL_PLANNER_H_