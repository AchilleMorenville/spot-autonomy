#include "aut_common/transformer.h"

#include <memory>
#include <chrono>

#include <Eigen/Core>

#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "aut_utils/utils.h"

namespace aut_common {

Transformer::Transformer(std::shared_ptr<tf2_ros::Buffer> tf_buffer) : tf_buffer_(tf_buffer) {}

bool Transformer::CanTransformMapToBaseLink(builtin_interfaces::msg::Time time) {
  return tf_buffer_->canTransform("v_odom", "base_link", time, std::chrono::milliseconds(50)) && 
         tf_buffer_->canTransform("l_odom", "v_odom", tf2::TimePointZero) &&
         tf_buffer_->canTransform("map", "l_odom", tf2::TimePointZero);
}

bool Transformer::CanTransformMapToBaseLink() {
  return tf_buffer_->canTransform("v_odom", "base_link", tf2::TimePointZero) && 
         tf_buffer_->canTransform("l_odom", "v_odom", tf2::TimePointZero) &&
         tf_buffer_->canTransform("map", "l_odom", tf2::TimePointZero);
}

Eigen::Matrix4f Transformer::LookupTransformMapToBaseLink(builtin_interfaces::msg::Time time) {
  geometry_msgs::msg::Transform t_v_odom_to_base_link = tf_buffer_->lookupTransform("v_odom", "base_link", time, std::chrono::milliseconds(50)).transform;
  geometry_msgs::msg::Transform t_l_odom_to_v_odom = tf_buffer_->lookupTransform("l_odom", "v_odom", tf2::TimePointZero).transform;
  geometry_msgs::msg::Transform t_map_to_l_odom = tf_buffer_->lookupTransform("map", "l_odom", tf2::TimePointZero).transform;

  Eigen::Matrix4f m_v_odom_to_base_link = aut_utils::TransformToMatrix(t_v_odom_to_base_link);
  Eigen::Matrix4f m_l_odom_to_v_odom = aut_utils::TransformToMatrix(t_l_odom_to_v_odom);
  Eigen::Matrix4f m_map_to_l_odom = aut_utils::TransformToMatrix(t_map_to_l_odom);

  return (m_map_to_l_odom * m_l_odom_to_v_odom) * m_v_odom_to_base_link;
}

Eigen::Matrix4f Transformer::LookupTransformMapToBaseLink() {
  geometry_msgs::msg::Transform t_v_odom_to_base_link = tf_buffer_->lookupTransform("v_odom", "base_link", tf2::TimePointZero).transform;
  geometry_msgs::msg::Transform t_l_odom_to_v_odom = tf_buffer_->lookupTransform("l_odom", "v_odom", tf2::TimePointZero).transform;
  geometry_msgs::msg::Transform t_map_to_l_odom = tf_buffer_->lookupTransform("map", "l_odom", tf2::TimePointZero).transform;

  Eigen::Matrix4f m_v_odom_to_base_link = aut_utils::TransformToMatrix(t_v_odom_to_base_link);
  Eigen::Matrix4f m_l_odom_to_v_odom = aut_utils::TransformToMatrix(t_l_odom_to_v_odom);
  Eigen::Matrix4f m_map_to_l_odom = aut_utils::TransformToMatrix(t_map_to_l_odom);

  return (m_map_to_l_odom * m_l_odom_to_v_odom) * m_v_odom_to_base_link;
}

}  // namespace aut_common