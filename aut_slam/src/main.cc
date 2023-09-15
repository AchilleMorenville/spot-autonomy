#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "aut_slam/graph_optimization.h"

int main(int argc, char * argv[]) {
   rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(
  	rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500)
  );

  // rclcpp::executors::SingleThreadedExecutor exec;
  auto graph_optimization_node = std::make_shared<aut_slam::GraphOptimization>();
  // auto lidar_odometry_node = std::make_shared<aut_lidar_odometry::LidarOdometry>();
  
  exec.add_node(graph_optimization_node);
  // exec.add_node(lidar_odometry_node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Graph Optimization Started.\033[0m");
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Lidar Odometry Started.\033[0m");

  std::thread map_thread(&aut_slam::GraphOptimization::MapThread, graph_optimization_node);
  std::thread loop_thread(&aut_slam::GraphOptimization::LoopThread, graph_optimization_node);

  exec.spin();

  map_thread.join();
  loop_thread.join();

  rclcpp::shutdown();
  return 0;
}