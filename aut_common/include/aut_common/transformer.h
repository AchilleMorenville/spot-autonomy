#ifndef AUT_COMMON_TRANSFORMER_H_
#define AUT_COMMON_TRANSFORMER_H_

#include <memory>

#include <Eigen/Core>

#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace aut_common {

class Transformer {

 public:
  explicit Transformer(std::shared_ptr<tf2_ros::Buffer> tf_buffer);
  bool CanTransformMapToBaseLink(builtin_interfaces::msg::Time time);
  bool CanTransformMapToBaseLink();
  Eigen::Matrix4f LookupTransformMapToBaseLink(builtin_interfaces::msg::Time time);
  Eigen::Matrix4f LookupTransformMapToBaseLink();

 private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

};

}  // namespace aut_common

#endif  // AUT_COMMON_TRANSFORMER_H_