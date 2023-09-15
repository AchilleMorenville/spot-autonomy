#include "aut_global_planner/global_planner.h"

#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <limits>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/point.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <aut_common/graph.h>
#include <aut_common/transformer.h>
#include <aut_utils/utils.h>
#include <aut_msgs/action/navigate_to_goal.hpp>
#include <aut_msgs/srv/graph_modif.hpp>

namespace aut_global_planner {

GlobalPlanner::GlobalPlanner(const rclcpp::NodeOptions& options)
    : Node("global_planner", options) {

  path_to_graph_ = std::string("");

  map_loaded_ = false;
  navigation_executing_ = false;
  nav_graph_has_changed_ = false;
  goal_achieved_ = false;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  transformer_ = std::make_unique<aut_common::Transformer>(tf_buffer_);

  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("aut_global_planner/graph", 10);

  action_server_ = rclcpp_action::create_server<aut_msgs::action::NavigateToGoal>(
      this,
      "aut_global_planner/navigate_to_goal",
      std::bind(&GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GlobalPlanner::handle_cancel, this, std::placeholders::_1),
      std::bind(&GlobalPlanner::handle_accepted, this, std::placeholders::_1)
  );

  load_map_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );
  graph_modif_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );
  goal_achieved_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );
  load_map_service_ = this->create_service<aut_msgs::srv::LoadMap>(
      "aut_global_planner/load_map", 
      std::bind(&GlobalPlanner::LoadMapService, this, std::placeholders::_1, 
                std::placeholders::_2),
      rmw_qos_profile_services_default,
      load_map_callback_group_
  );
  graph_modif_service_ = this->create_service<aut_msgs::srv::GraphModif>(
      "aut_global_planner/graph_modif", 
      std::bind(&GlobalPlanner::GraphModifService, this, std::placeholders::_1, 
                std::placeholders::_2),
      rmw_qos_profile_services_default,
      graph_modif_callback_group_);
  goal_achieved_service_ = this->create_service<std_srvs::srv::Trigger>(
      "aut_global_planner/goal_achieved", 
      std::bind(&GlobalPlanner::GoalAchievedService, this, std::placeholders::_1, 
                std::placeholders::_2),
      rmw_qos_profile_services_default,
      goal_achieved_callback_group_);

  set_global_path_client_ = this->create_client<aut_msgs::srv::SetGlobalPath>("aut_local_planner/set_global_path");
}

void GlobalPlanner::LoadMapService(
    const std::shared_ptr<aut_msgs::srv::LoadMap::Request> request, 
    std::shared_ptr<aut_msgs::srv::LoadMap::Response> response) {
  if (navigation_executing_) {
    response->success = false;
    response->message = std::string("Cannot change map while navigation is executing");
    return;
  }

  path_to_graph_ = std::string(request->map_directory_path) + std::string("/graph.txt");

  nav_graph_.LoadFile(path_to_graph_);
  nav_graph_.Simplify(0.5);
  nav_graph_.TidyGraph();

  map_loaded_ = true;
  response->success = true;
  response->message = std::string("Map loaded");
}

void GlobalPlanner::GraphModifService(
    const std::shared_ptr<aut_msgs::srv::GraphModif::Request> request, 
    std::shared_ptr<aut_msgs::srv::GraphModif::Response> response) {
  Eigen::Vector3f position_1(request->position_1.x, request->position_1.y, request->position_1.z);
  Eigen::Vector3f position_2(request->position_2.x, request->position_2.y, request->position_2.z);

  std::unique_lock<std::mutex> nav_graph_lock(state_mtx_);
  nav_graph_has_changed_ = true;
  int position_1_idx = nav_graph_.ClosestNode(position_1);
  int position_2_idx = nav_graph_.ClosestNode(position_2);
  bool removed = nav_graph_.RemoveEdge(position_1_idx, position_2_idx);
  if (removed) {
    RCLCPP_INFO(this->get_logger(), "Remove edge");
  } else {
    RCLCPP_INFO(this->get_logger(), "Cannot remove edge");
  }
  response->success = true;
}

void GlobalPlanner::GoalAchievedService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  goal_achieved_ = true;
  response->success = true;
  response->message = std::string("Global Planner is notified of the achieved goal");
}

rclcpp_action::GoalResponse GlobalPlanner::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const aut_msgs::action::NavigateToGoal::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), 
              "Received goal request with goal -> x: %f, y: %f, z: %f", 
              goal->position.x, 
              goal->position.y, 
              goal->position.z);
  (void)uuid;
  if (!map_loaded_ || navigation_executing_) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  navigation_executing_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle) {
  std::thread{std::bind(&GlobalPlanner::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void GlobalPlanner::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  navigation_executing_ = true;
  if (!transformer_->CanTransformMapToBaseLink()) {
    AbortNavigation(goal_handle, "Navigation aborted, no localization on map");
    ResetNavigation();
    return;
  }
  const auto goal = goal_handle->get_goal();
  Eigen::Matrix4f goal_pose = Eigen::Matrix4f::Identity();
  goal_pose(0, 3) = goal->position.x;
  goal_pose(1, 3) = goal->position.y;
  goal_pose(2, 3) = goal->position.z;
  Eigen::Quaternionf goal_quaternion(1.0f, 0.0f, 0.0f, 0.0f);
  goal_pose.block<3, 3>(0, 0) = goal_quaternion.toRotationMatrix();
  Eigen::Matrix4f start_pose = transformer_->LookupTransformMapToBaseLink();
  std::vector<Eigen::Matrix4f> path_poses;
  if (!CreatPath(goal_handle, start_pose, goal_pose, path_poses)) {
    return;
  }
  if (!SendPathToLocalPlanner(goal_handle, path_poses)) {
    return;
  }

  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      auto set_global_path_empty_request = std::make_shared<aut_msgs::srv::SetGlobalPath::Request>();
      auto set_global_path_result = set_global_path_client_->async_send_request(set_global_path_empty_request);
      // rclcpp::spin_until_future_complete(this->get_node_base_interface(), set_global_path_result);

      auto result = std::make_shared<aut_msgs::action::NavigateToGoal::Result>();
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Navigation canceled");
      ResetNavigation();
      return;
    }
    if (goal_achieved_) {
      auto result = std::make_shared<aut_msgs::action::NavigateToGoal::Result>();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Navigation succeded");
      ResetNavigation();
      return;
    }
    if (nav_graph_has_changed_) {

      RCLCPP_INFO(this->get_logger(), "Modify graph");

      nav_graph_has_changed_ = false;
      Eigen::Matrix4f current_pose = transformer_->LookupTransformMapToBaseLink();
      path_poses.clear();
      if (!CreatPath(goal_handle, current_pose, goal_pose, path_poses)) {
        return;
      }
      if (!SendPathToLocalPlanner(goal_handle, path_poses)) {
        return;
      }
    }
    loop_rate.sleep();
  }
}

void GlobalPlanner::AbortNavigation(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle,
    std::string abort_msg) {
  auto result = std::make_shared<aut_msgs::action::NavigateToGoal::Result>();
  result->success = false;
  goal_handle->abort(result);
  RCLCPP_INFO(this->get_logger(), abort_msg.c_str());
}

void GlobalPlanner::ResetNavigation() {
  navigation_executing_ = false;
  goal_achieved_ = false;
  nav_graph_has_changed_ = false;
  map_loaded_ = true;

  nav_graph_ = aut_common::Graph();
  nav_graph_.LoadFile(path_to_graph_);
  nav_graph_.Simplify(0.5);
  nav_graph_.TidyGraph();
}

bool GlobalPlanner::CreatPath(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle,
    const Eigen::Matrix4f& start_pose, 
    const Eigen::Matrix4f& goal_pose, 
    std::vector<Eigen::Matrix4f>& path_poses) {
  
  Eigen::Vector3f start_position = start_pose.block<3, 1>(0, 3);
  Eigen::Vector3f goal_position = goal_pose.block<3, 1>(0, 3);
  
  std::unique_lock<std::mutex> nav_graph_lock(state_mtx_);
  int closest_idx_start_position = nav_graph_.ClosestNode(start_position);
  int closest_idx_goal_position = nav_graph_.ClosestNode(goal_position);
  if (nav_graph_.Empty() ||
      (start_position - nav_graph_.GetPose(closest_idx_start_position).block<3, 1>(0, 3)).norm() >= 1.8f ||
      (goal_position - nav_graph_.GetPose(closest_idx_goal_position).block<3, 1>(0, 3)).norm() >= 1.8f) {
    AbortNavigation(goal_handle, "Navigation aborted, goal or start too far from known map");
    ResetNavigation();
    return false;
  }
  std::vector<int> path_idx;
  bool found_path = nav_graph_.AStar(closest_idx_start_position, closest_idx_goal_position, path_idx);
  if (!found_path) {
    AbortNavigation(goal_handle, "Navigation aborted, no path to goal");
    ResetNavigation();
    return false;
  }
  for (int &idx : path_idx) {
    path_poses.push_back(nav_graph_.GetPose(idx));
  }
  path_poses.push_back(goal_pose);
  return true;
}

bool GlobalPlanner::SendPathToLocalPlanner(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle,
    const std::vector<Eigen::Matrix4f>& path_poses) {
  auto set_global_path_request = std::make_shared<aut_msgs::srv::SetGlobalPath::Request>(); 
  for (const Eigen::Matrix4f& pose : path_poses) {
    geometry_msgs::msg::PoseStamped g_pose;
    g_pose.pose.position.x = pose(0, 3);
    g_pose.pose.position.y = pose(1, 3);
    g_pose.pose.position.z = pose(2, 3);
    set_global_path_request->global_path.poses.push_back(g_pose);
  }

  auto set_global_path_result = set_global_path_client_->async_send_request(set_global_path_request);
  if (set_global_path_result.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
    AbortNavigation(goal_handle, "Cannot send global path to local planner");
    ResetNavigation();
    return false;
  }
  // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), set_global_path_result) !=
  //     rclcpp::FutureReturnCode::SUCCESS) {
  //   AbortNavigation(goal_handle, "Cannot send global path to local planner");
  //   ResetNavigation();
  //   return false;
  // }
  return true;
}

// void GlobalPlanner::execute(
//     const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle) {
//   RCLCPP_INFO(this->get_logger(), "Executing goal");

//   rclcpp::Rate loop_rate(10);
//   const auto goal = goal_handle->get_goal();
//   auto feedback = std::make_shared<aut_msgs::action::NavigateToGoal::Feedback>();
//   auto result = std::make_shared<aut_msgs::action::NavigateToGoal::Result>();

//   if (!transformer_->CanTransformMapToBaseLink()) {
//     result->success = false;
//     goal_handle->abort(result);
//     RCLCPP_INFO(this->get_logger(), "Goal aborted, no localization on map");
//     return;
//   }

//   Eigen::Vector3f goal_position(goal->position.x, goal->position.y, goal->position.z);
//   if (goal->position.x == 0.0 && goal->position.y == 0.0 && goal->position.z) {
//     Eigen::Matrix4f current_pose = transformer_->LookupTransformMapToBaseLink();
//     Eigen::Vector3f current_position = current_pose.block<3, 1>(0, 3);
//     nav_graph_.FurthestNode(current_position);
//   }

//   state_mtx_.lock();
//   int goal_closest_node_idx = nav_graph_.ClosestNode(goal_position);
//   Eigen::Matrix4f goal_closest_node_pose = nav_graph_.GetPose(goal_closest_node_idx);
//   state_mtx_.unlock();

//   if (goal_closest_node_idx < 0 || (goal_closest_node_pose.block<3, 1>(0, 3) - goal_position).squaredNorm() > 2.5f) {
//     result->success = false;
//     goal_handle->abort(result);
//     RCLCPP_INFO(this->get_logger(), "Goal aborted, goal too far from known map");
//     return;
//   }

//   // geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero).transform;
//   Eigen::Matrix4f current_pose = transformer_->LookupTransformMapToBaseLink();

//   state_mtx_.lock();
//   int start_closest_node_idx = nav_graph_.ClosestNode(current_pose);
//   Eigen::Matrix4f start_closest_node_pose = nav_graph_.GetPose(start_closest_node_idx);
//   state_mtx_.unlock();

//   if (start_closest_node_idx < 0 || (start_closest_node_pose.block<3, 1>(0, 3) - current_pose.block<3, 1>(0, 3)).squaredNorm() > 1.8f) {
//     result->success = false;
//     goal_handle->abort(result);
//     RCLCPP_INFO(this->get_logger(), "Goal aborted, start too far from known map");
//     return;
//   }

//   std::vector<int> path_idx;
//   state_mtx_.lock();
//   bool found_path = nav_graph_.AStar(start_closest_node_idx, goal_closest_node_idx, path_idx);
//   state_mtx_.unlock();

//   if (!found_path) {
//     result->success = false;
//     goal_handle->abort(result);
//     RCLCPP_INFO(this->get_logger(), "Goal aborted, no path to goal");
//     return;
//   }

//   std::vector<Eigen::Vector3f> path_position;
//   state_mtx_.lock();
//   for (int i = 0; i < (int) path_idx.size(); ++i) {
//     path_position.push_back(nav_graph_.GetPose(path_idx[i]).block<3, 1>(0, 3));
//   }
//   visualization_msgs::msg::MarkerArray current_marker_array = nav_graph_.GetMarkerArrayWithPath(path_idx);
//   state_mtx_.unlock();

//   goal_is_set_ = true;

//   aut_msgs::msg::Nav nav_msg;

//   for (int i = 0; i < (int) path_position.size(); ++i) {
//     geometry_msgs::msg::Point point;
//     point.x = path_position[i](0);
//     point.y = path_position[i](1);
//     point.z = path_position[i](2);
//     nav_msg.positions.push_back(point);
//   }

//   nav_publisher_->publish(nav_msg);

//   while (rclcpp::ok()) {

//     if (goal_handle->is_canceling()) {

//       aut_msgs::msg::Nav empty_nav_msg;
//       nav_publisher_->publish(empty_nav_msg);

//       result->success = false;
//       goal_handle->canceled(result);
//       RCLCPP_INFO(this->get_logger(), "Goal canceled");
//       goal_is_set_ = false;
//       return;
//     }

//     // geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero).transform;
//     Eigen::Matrix4f map_tform_base_link = transformer_->LookupTransformMapToBaseLink();
//     if ((map_tform_base_link.block<3, 1>(0, 3) - goal_position).norm() < 0.2)  {
//       aut_msgs::msg::Nav empty_nav_msg;
//       nav_publisher_->publish(empty_nav_msg);

//       result->success = true;
//       goal_handle->succeed(result);
//       RCLCPP_INFO(this->get_logger(), "Goal succeded");

//       state_mtx_.lock();
//       goal_is_set_ = false;
//       state_mtx_.unlock();
//       return;
//     }

//     // trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero).transform;
//     // current_pose = aut_utils::TransformToMatrix(trans);

//     state_mtx_.lock();
//     if (nav_graph_has_changed_) {

//       if (inaccessible_node_in_path_ < 0) {
//         aut_msgs::msg::Nav empty_nav_msg;
//         nav_publisher_->publish(empty_nav_msg);

//         result->success = false;
//         goal_handle->abort(result);
//         RCLCPP_INFO(this->get_logger(), "Goal aborted, no path feasible");
//         return;
//       }

//       // Modify nav_graph_
//       nav_graph_.RemoveEdge(path_idx[inaccessible_node_in_path_], path_idx[inaccessible_node_in_path_ - 1]);

//       start_closest_node_idx = nav_graph_.ClosestNode(map_tform_base_link);
//       start_closest_node_pose = nav_graph_.GetPose(start_closest_node_idx);
      
//       path_idx.clear();
//       found_path = nav_graph_.AStar(start_closest_node_idx, goal_closest_node_idx, path_idx);

//       if (!found_path) {
//         aut_msgs::msg::Nav empty_nav_msg;
//         nav_publisher_->publish(empty_nav_msg);

//         result->success = false;
//         goal_handle->abort(result);
//         RCLCPP_INFO(this->get_logger(), "Goal aborted, no more path to goal");
//         return;
//       }

//       path_position.clear();
//       for (int i = 0; i < (int) path_idx.size(); ++i) {
//         path_position.push_back(nav_graph_.GetPose(path_idx[i]).block<3, 1>(0, 3));
//       }

//       aut_msgs::msg::Nav new_nav_msg;

//       for (int i = 0; i < (int) path_position.size(); ++i) {
//         geometry_msgs::msg::Point point;
//         point.x = path_position[i](0);
//         point.y = path_position[i](1);
//         point.z = path_position[i](2);
//         new_nav_msg.positions.push_back(point);
//       }

//       nav_publisher_->publish(new_nav_msg);

//       current_marker_array = nav_graph_.GetMarkerArrayWithPath(path_idx);

//       nav_graph_has_changed_ = false;

//       feedback->msg = std::string("Rerouting");
//     } else {
//       feedback->msg = std::string("Following");
//     }
//     state_mtx_.unlock();

//     int start_path_idx = 0;
//     float closest_dist = (map_tform_base_link.block<3, 1>(0, 3) - path_position[0]).squaredNorm();
//     for (int i = 1; i < (int) path_position.size(); ++i) {
//       if ((map_tform_base_link.block<3, 1>(0, 3) - path_position[i]).squaredNorm() < closest_dist) {
//         closest_dist = (map_tform_base_link.block<3, 1>(0, 3) - path_position[i]).squaredNorm();
//         start_path_idx = i;
//       }
//     }

//     float path_dist = (map_tform_base_link.block<3, 1>(0, 3) - path_position[start_path_idx]).norm();

//     for (int i = start_path_idx; i < (int) path_idx.size() - 1; ++i) {
//       path_dist += (path_position[i] - path_position[i + 1]).norm();
//     }

//     feedback->distance = path_dist;
//     goal_handle->publish_feedback(feedback);
//     RCLCPP_INFO(this->get_logger(), "Publish feedback");

//     marker_array_publisher_->publish(current_marker_array);

//     loop_rate.sleep();
//   }
// }

// void GlobalPlanner::NavModifCallBack(const aut_msgs::msg::NavModif::SharedPtr nav_modif_msg) {
//   state_mtx_.lock();
//   nav_graph_has_changed_ = true;
//   inaccessible_node_in_path_ = nav_modif_msg->inaccessible_node_in_path;
//   state_mtx_.unlock();
// }

}  // namespace aut_global_planner