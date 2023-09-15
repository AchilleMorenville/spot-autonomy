#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "aut_global_planner/global_planner.h"

int main(int argc, char * argv[]) {
   rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(
  	rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500)
  );

  auto global_planner_node = std::make_shared<aut_global_planner::GlobalPlanner>();
  
  exec.add_node(global_planner_node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Global Planner Started.\033[0m");

  exec.spin();

  rclcpp::shutdown();
  return 0;
}