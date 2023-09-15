#include <chrono>
#include <memory>

#include "aut_lidar_odometry/feature_extraction.h"
#include "aut_lidar_odometry/lidar_odometry.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(
  	rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500)
  );

  // rclcpp::executors::SingleThreadedExecutor exec;
  auto feature_extraction_node = std::make_shared<aut_lidar_odometry::FeatureExtraction>();
  auto lidar_odometry_node = std::make_shared<aut_lidar_odometry::LidarOdometry>();
  
  exec.add_node(feature_extraction_node);
  exec.add_node(lidar_odometry_node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Feature Extraction Started.\033[0m");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Lidar Odometry Started.\033[0m");

  exec.spin();
  rclcpp::shutdown();
  return 0;
}