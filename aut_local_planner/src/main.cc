#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "aut_local_planner/local_planner.h"

int main(int argc, char * argv[]) {
   rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(
  	rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500)
  );

  auto local_planner_node = std::make_shared<aut_local_planner::LocalPlanner>();
  
  exec.add_node(local_planner_node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Local Planner Started.\033[0m");

  exec.spin();

  rclcpp::shutdown();
  return 0;
}