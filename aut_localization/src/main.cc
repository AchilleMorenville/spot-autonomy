#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "aut_localization/monte_carlo_localization.h"

int main(int argc, char * argv[]) {
   rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(
  	rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500)
  );

  auto monte_carlo_localization_node = std::make_shared<aut_localization::MonteCarloLocalization>();
  
  exec.add_node(monte_carlo_localization_node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Localization Started.\033[0m");

  exec.spin();

  rclcpp::shutdown();
  return 0;
}