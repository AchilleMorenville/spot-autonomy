// This code is inspired by:
// Shan, Tixiao and Englot, Brendan. LeGO-LOAM. https://github.com/RobustFieldAutonomyLab/LeGO-LOAM (Under BSD-3 License)

#include "aut_lidar_odometry/feature_extraction.h"

#include <chrono>
#include <cstddef>
#include <memory>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <pcl/filters/voxel_grid.h>

#include "aut_lidar_odometry/msg/features.hpp"

namespace aut_lidar_odometry {

struct by_value {
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

FeatureExtraction::FeatureExtraction(const rclcpp::NodeOptions& options)
    : Node("feature_extraction", options) {
      
  this->declare_parameter("max_cache_size", 20);
  max_cache_size_ = this->get_parameter("max_cache_size").get_parameter_value().get<int>();    

  RCLCPP_INFO(this->get_logger(), "Received cloud");

  this->declare_parameter("max_lidar_range", 30.0f);
  max_lidar_range_ = this->get_parameter("max_lidar_range").get_parameter_value().get<float>();

  this->declare_parameter("min_box_x", -0.3f);
  min_box_x_ = this->get_parameter("min_box_x").get_parameter_value().get<float>();
  this->declare_parameter("max_box_x", 0.8f);
  max_box_x_ = this->get_parameter("max_box_x").get_parameter_value().get<float>();

  this->declare_parameter("min_box_y", -0.25f);
  min_box_y_ = this->get_parameter("min_box_y").get_parameter_value().get<float>();
  this->declare_parameter("max_box_y", 0.25f);
  max_box_y_ = this->get_parameter("max_box_y").get_parameter_value().get<float>();

  this->declare_parameter("min_box_z", -0.3f);
  min_box_z_ = this->get_parameter("min_box_z").get_parameter_value().get<float>();
  this->declare_parameter("max_box_z", 0.3f);
  max_box_z_ = this->get_parameter("max_box_z").get_parameter_value().get<float>();

  this->declare_parameter("threshold_edge", 1.0f);
  threshold_edge_ = this->get_parameter("threshold_edge").get_parameter_value().get<float>();

  this->declare_parameter("threshold_flat", 0.1f);
  threshold_flat_ = this->get_parameter("threshold_flat").get_parameter_value().get<float>();

  Eigen::Quaternionf rotation_body_tform_velodyne(1.0f, -0.0f, 0.0f, -0.0f);
  Eigen::Vector3f translation_body_tform_velodyne(-0.202f, 0.0f, 0.151f);

  affine_body_tform_velodyne_.translation() = translation_body_tform_velodyne;
  affine_body_tform_velodyne_.linear() = rotation_body_tform_velodyne.toRotationMatrix();

  // tf buffer
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // // Subscriptions
  // v_odom_callback_group_ = this->create_callback_group(
  //     rclcpp::CallbackGroupType::MutuallyExclusive
  // );

  // rclcpp::SubscriptionOptions v_odom_options = rclcpp::SubscriptionOptions();
  // v_odom_options.callback_group = v_odom_callback_group_;

  // v_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "aut_spot/odometry/v_odom", 10, 
  //   std::bind(&FeatureExtraction::VOdomCallBack, this, std::placeholders::_1),
  //   v_odom_options
  // );


  point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", 10, 
      std::bind(&FeatureExtraction::PointCloudCallBack, this, 
                std::placeholders::_1)
  );

  features_publisher_ = this->create_publisher<aut_lidar_odometry::msg::Features>("aut_lidar_odometry/features", 10);

  current_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());

  image_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  image_cloud_->points.resize(16 * 1800);

  range_matrix_ = Eigen::MatrixXf::Constant(16, 1800, -1.0f);

  organized_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  start_ring_index_.resize(16);
  end_ring_index_.resize(16);

  cloud_column_id_.resize(16 * 1800);
  cloud_range_.resize(16 * 1800);

  cloud_curvature_.resize(16 * 1800);
  cloud_smoothness_.resize(16 * 1800);
  cloud_label_.resize(16 * 1800);
  cloud_neighbor_picked_.resize(16 * 1800);

  edge_points_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  flat_points_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  flat_points_scan_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  flat_points_scan_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void FeatureExtraction::ResetState() {
  current_cloud_->clear();
  image_cloud_->clear();
  image_cloud_->points.resize(16 * 1800);
  range_matrix_ = Eigen::MatrixXf::Constant(16, 1800, -1.0f);
  organized_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  edge_points_->clear();
  flat_points_->clear();
  flat_points_scan_->clear();
  flat_points_scan_ds_->clear();
} 

// void FeatureExtraction::VOdomCallBack(
//     const nav_msgs::msg::Odometry::SharedPtr v_odom_msg) {
//   geometry_msgs::msg::TransformStamped t;
//   t.header = v_odom_msg->header;
//   t.child_frame_id = v_odom_msg->child_frame_id;
//   t.transform.translation.x = v_odom_msg->pose.pose.position.x;
//   t.transform.translation.y = v_odom_msg->pose.pose.position.y;
//   t.transform.translation.z = v_odom_msg->pose.pose.position.z;
//   t.transform.rotation = v_odom_msg->pose.pose.orientation;

//   std::lock_guard<std::mutex> lock(tf_buffer_mtx_);
//   tf_buffer_->setTransform(t, "spot");
// }

void FeatureExtraction::PointCloudCallBack(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg) {

  rclcpp::Time init_time = this->get_clock()->now();

  ResetState();

  RCLCPP_INFO(this->get_logger(), "Received cloud");

  // cache_point_cloud_msg_.push_back(*point_cloud_msg);
  // if ((int) cache_point_cloud_msg_.size() > max_cache_size_) {
  //   cache_point_cloud_msg_.pop_front();
  // }

  pcl::moveFromROSMsg(*point_cloud_msg, *current_cloud_);
  current_header_ = point_cloud_msg->header;

  if (!CanProcessPointCloud()) {
    RCLCPP_INFO(this->get_logger(), "Cannot process cloud, queue size : %d", (int) cache_point_cloud_msg_.size());
    return;
  }
  // cache_point_cloud_msg_.pop_front();

  PreprocessPointCloud();

  ImageProjection();

  OrganizeCloud();

  ComputeSmoothness();

  MarkUnreliable();

  ExctractFeatures();

  PublishFeatures();

  rclcpp::Time end_time = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "Time to process : %f", (end_time - init_time).seconds());
}

bool FeatureExtraction::CanProcessPointCloud() {

  // sensor_msgs::msg::PointCloud2 current_cloud_msg = cache_point_cloud_msg_.front();
  // pcl::moveFromROSMsg(current_cloud_msg, *current_cloud_);

  // current_header_ = cache_point_cloud_msg_.front().header;

  long current_stamp_nano = rclcpp::Time(current_header_.stamp).nanoseconds();

  RCLCPP_INFO(this->get_logger(), "Current stamp : %f", rclcpp::Time(current_header_.stamp).seconds());

  current_point_start_time_ = current_cloud_->points.front().time;
  current_point_end_time_ = current_cloud_->points.back().time;

  RCLCPP_INFO(this->get_logger(), "first point delta : %f, last point delta : %f", current_point_start_time_, current_point_end_time_);
  
  current_start_time_cloud_ = current_stamp_nano + static_cast<long>(current_point_start_time_ * 1e9);
  current_end_time_cloud_ = current_stamp_nano + static_cast<long>(current_point_end_time_ * 1e9);

  bool can_transform_start = tf_buffer_->canTransform("v_odom", "base_link", rclcpp::Time(current_start_time_cloud_), rclcpp::Duration::from_seconds(0.05));
  bool can_transform_end = tf_buffer_->canTransform("v_odom", "base_link", rclcpp::Time(current_end_time_cloud_), rclcpp::Duration::from_seconds(0.05));

  if (!can_transform_start) {
    RCLCPP_INFO(this->get_logger(), "Cannot transform start");
  }
  if (!can_transform_end) {
    RCLCPP_INFO(this->get_logger(), "Cannot transform end");
  }

  if (!can_transform_start || !can_transform_end) {
    return false;
  }

  return true;
}

void FeatureExtraction::PreprocessPointCloud() {

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*current_cloud_, *current_cloud_, indices); // needed the include <pcl/filters/impl/filter.hpp>

  transform_start_ = tf_buffer_->lookupTransform("v_odom", "base_link", rclcpp::Time(current_start_time_cloud_));
  transform_end_ = tf_buffer_->lookupTransform("v_odom", "base_link", rclcpp::Time(current_end_time_cloud_));

  rotation_start_ = Eigen::Quaternionf(transform_start_.transform.rotation.w, 
                                       transform_start_.transform.rotation.x, 
                                       transform_start_.transform.rotation.y, 
                                       transform_start_.transform.rotation.z);
  rotation_end_ = Eigen::Quaternionf(transform_end_.transform.rotation.w, 
                                     transform_end_.transform.rotation.x, 
                                     transform_end_.transform.rotation.y, 
                                     transform_end_.transform.rotation.z);

  translation_start_ = Eigen::Vector3f(transform_start_.transform.translation.x, 
                                       transform_start_.transform.translation.y, 
                                       transform_start_.transform.translation.z);
  translation_end_ = Eigen::Vector3f(transform_end_.transform.translation.x, 
                                     transform_end_.transform.translation.y, 
                                     transform_end_.transform.translation.z);

  affine_start_.translation() = translation_start_;
  affine_start_.linear() = rotation_start_.toRotationMatrix();

  affine_end_.translation() = translation_end_;
  affine_end_.linear() = rotation_end_.toRotationMatrix();

  affine_start_ = affine_start_ * affine_body_tform_velodyne_;
  affine_end_ = affine_end_ * affine_body_tform_velodyne_;

  rotation_start_ = affine_start_.linear();
  rotation_end_ = affine_end_.linear();

  translation_start_ = affine_start_.translation();
  translation_end_ = affine_end_.translation();
}

void FeatureExtraction::ImageProjection() {

  int current_cloud_size = current_cloud_->points.size();
  for (int i = 0; i < current_cloud_size; ++i) {

    pcl::PointXYZI point;
    point.x = current_cloud_->points[i].x;
    point.y = current_cloud_->points[i].y;
    point.z = current_cloud_->points[i].z;
    point.intensity = current_cloud_->points[i].intensity;

    float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    // Range filtering
    if (range > max_lidar_range_) {
      continue;
    }

    // Box filtering
    if ((point.x > min_box_x_ && point.x < max_box_x_) && 
        (point.y > min_box_y_ && point.y < max_box_y_) && 
        (point.z > min_box_z_ && point.z < max_box_z_)) {
      continue;
    }

    int row_id = current_cloud_->points[i].ring;

    if ((row_id < 0) || (row_id >= 16)) {
      continue;
    }

    float horizontal_angle = std::atan2(point.x, point.y) * 180 / M_PI;

    int column_id = -std::round((horizontal_angle - 90.0) / (360.0 / 1800.0)) + 1800 / 2;
    if (column_id >= 1800) {
      column_id -= 1800;
    }

    if ((column_id < 0) || (column_id >= 1800)) {
      continue;
    }

    if (range_matrix_(row_id, column_id) >= 0.0f) {
      continue;
    }

    DeskewPoint(point, current_cloud_->points[i].time);

    range_matrix_(row_id, column_id) = range;
    image_cloud_->points[column_id + row_id * 1800] = point;
  }
}

void FeatureExtraction::DeskewPoint(pcl::PointXYZI & p, float point_time) {
  float alpha = (point_time - current_point_start_time_) / (current_point_end_time_ - current_point_start_time_);

  Eigen::Affine3f affine_correction;
  affine_correction.translation() = (1.0 - alpha) * translation_start_ + alpha * translation_end_;
  affine_correction.linear() = rotation_start_.slerp(alpha, rotation_end_).toRotationMatrix();

  Eigen::Vector3f vec_point(p.x, p.y, p.z);
  Eigen::Vector3f vec_point_orig(p.x, p.y, p.z);
  vec_point = ( affine_start_.inverse() * affine_correction ) * vec_point;

  p.x = vec_point(0);
  p.y = vec_point(1);
  p.z = vec_point(2);
}

void FeatureExtraction::OrganizeCloud() {
  int count = 0;
  for (int i = 0; i < 16; ++i) {
    start_ring_index_[i] = count - 1 + 5;
    for (int j = 0; j < 1800; ++j) {
      if (range_matrix_(i, j) >= 0.0f) {
        cloud_column_id_[count] = j;
        cloud_range_[count] = range_matrix_(i, j);
        organized_cloud_->push_back(image_cloud_->points[j + i * 1800]);
        ++count;
      }
    }
    end_ring_index_[i] = count - 1 - 5;
  }
}

void FeatureExtraction::ComputeSmoothness() {
  int cloud_size = (int) organized_cloud_->points.size();
  RCLCPP_INFO(this->get_logger(), "Size point cloud : %d", cloud_size);
  for (int i = 5; i < cloud_size - 5; ++i) {
    float diff_range = cloud_range_[i - 5] + cloud_range_[i - 4] 
                     + cloud_range_[i - 3] + cloud_range_[i - 2] 
                     + cloud_range_[i - 1] - 10 * cloud_range_[i]
                     + cloud_range_[i + 1] + cloud_range_[i + 2]
                     + cloud_range_[i + 3] + cloud_range_[i + 4]
                     + cloud_range_[i + 5];
    cloud_curvature_[i] = diff_range * diff_range;
    cloud_label_[i] = 0;
    cloud_neighbor_picked_[i] = 0;
    cloud_smoothness_[i].value = cloud_curvature_[i];
    cloud_smoothness_[i].ind = i;
  }
}

void FeatureExtraction::MarkUnreliable() {
  int cloud_size = (int) organized_cloud_->points.size();
  for (int i = 5; i < cloud_size - 6; ++i) {
    float range_1 = cloud_range_[i];
    float range_2 = cloud_range_[i + 1];
    int column_diff = std::abs(cloud_column_id_[i + 1] - cloud_column_id_[i]);  // Check if the int() call is usefull

    // Occluded points
    if (column_diff < 10) {
      if (range_1 - range_2 > 0.3) {
        cloud_neighbor_picked_[i - 5] = 1;
        cloud_neighbor_picked_[i - 4] = 1;
        cloud_neighbor_picked_[i - 3] = 1;
        cloud_neighbor_picked_[i - 2] = 1;
        cloud_neighbor_picked_[i - 1] = 1;
        cloud_neighbor_picked_[i] = 1;
      } else if (range_2 - range_1 > 0.3) {
        cloud_neighbor_picked_[i + 1] = 1;
        cloud_neighbor_picked_[i + 2] = 1;
        cloud_neighbor_picked_[i + 3] = 1;
        cloud_neighbor_picked_[i + 4] = 1;
        cloud_neighbor_picked_[i + 5] = 1;
        cloud_neighbor_picked_[i + 6] = 1;
      }
    }

    // Parallel beam
    float diff_1 = std::abs(cloud_range_[i - 1] - cloud_range_[i]);
    float diff_2 = std::abs(cloud_range_[i] - cloud_range_[i + 1]);
    if (diff_1 > 0.02 * cloud_range_[i] && diff_2 > 0.02 * cloud_range_[i]) {
      cloud_neighbor_picked_[i] = 1;
    }
  }
}

void FeatureExtraction::ExctractFeatures() {

  for (int i = 0; i < 16; ++i) {
    flat_points_scan_->clear();
    for (int j = 0; j < 6; ++j) {
      int sp = (start_ring_index_[i] * (6 - j) + end_ring_index_[i] * j) / 6;
      int ep = (start_ring_index_[i] * (5 - j) + end_ring_index_[i] * (j + 1)) / 6 - 1;

      if (sp >= ep) {
        continue;
      }

      std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep, by_value());

      int n_edge_points = 0;
      for (int k = ep; k >= sp; --k) {

        int idx = cloud_smoothness_[k].ind;

        if (cloud_neighbor_picked_[idx] == 0 && cloud_curvature_[idx] > threshold_edge_) {

          ++n_edge_points;
          if (n_edge_points <= 20) {
            cloud_label_[idx] = 1;
            edge_points_->push_back(organized_cloud_->points[idx]);
          } else {
            break;
          }

          cloud_neighbor_picked_[idx] = 1;
          for (int l = 1; l <= 5; ++l) {
            int column_diff = std::abs(cloud_column_id_[idx + l] - cloud_column_id_[idx + l - 1]);
            if (column_diff > 10) {
              break;
            }
            cloud_neighbor_picked_[idx + l] = 1;
          }
          for (int l = -1; l >= -5; --l) {
            int column_diff = std::abs(cloud_column_id_[idx + l] - cloud_column_id_[idx + l + 1]);
            if (column_diff > 10) {
              break;
            }
            cloud_neighbor_picked_[idx + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; ++k) {

        int idx = cloud_smoothness_[k].ind;
        if (cloud_neighbor_picked_[idx] == 0 && cloud_curvature_[idx] < threshold_flat_) {

          cloud_label_[idx] = -1;
          cloud_neighbor_picked_[idx] = 1;

          for (int l = 1; l <= 5; ++l) {
            int column_diff = std::abs(cloud_column_id_[idx + l] - cloud_column_id_[idx + l - 1]);
            if (column_diff > 10) {
              break;
            }
            cloud_neighbor_picked_[idx + l] = 1;
          }
          for (int l = -1; l >= -5; --l) {
            int column_diff = std::abs(cloud_column_id_[idx + l] - cloud_column_id_[idx + l + 1]);
            if (column_diff > 10) {
              break;
            }
            cloud_neighbor_picked_[idx + l] = 1;
          }
        }

      }

      for (int k = sp; k <= ep; ++k) {
        if (cloud_label_[k] < 0) {
          flat_points_scan_->push_back(organized_cloud_->points[k]);
        }
      }

      *flat_points_ += *flat_points_scan_ ;
    }
  }
}

void FeatureExtraction::PublishFeatures() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_edge_points(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_flat_points(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_organized_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_organized_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  for (int i = 0; i < (int) organized_cloud_->points.size(); ++i) {

    pcl::PointXYZI point;
    point.x = organized_cloud_->points[i].x;
    point.y = organized_cloud_->points[i].y;
    point.z = organized_cloud_->points[i].z;
    point.intensity = organized_cloud_->points[i].intensity;

    float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    if (range > 30.0f) {
      continue;
    }

    filtered_organized_cloud->push_back(point);

  }

  pcl::transformPointCloud(*edge_points_, *transformed_edge_points, affine_body_tform_velodyne_.matrix());
  pcl::transformPointCloud(*flat_points_, *transformed_flat_points, affine_body_tform_velodyne_.matrix());
  pcl::transformPointCloud(*filtered_organized_cloud, *transformed_organized_cloud, affine_body_tform_velodyne_.matrix());

  sensor_msgs::msg::PointCloud2 temp_edge_points;
  sensor_msgs::msg::PointCloud2 temp_flat_points;
  sensor_msgs::msg::PointCloud2 temp_all_points;
  
  pcl::toROSMsg(*transformed_edge_points, temp_edge_points);
  pcl::toROSMsg(*transformed_flat_points, temp_flat_points);
  pcl::toROSMsg(*transformed_organized_cloud, temp_all_points);

  aut_lidar_odometry::msg::Features features_msg;
  
  features_msg.cloud = temp_all_points;
  features_msg.cloud_edge = temp_edge_points;
  features_msg.cloud_flat = temp_flat_points;
  features_msg.header = current_header_;

  features_msg.initial_guess = transform_start_.transform;

  features_publisher_->publish(features_msg);

  RCLCPP_INFO(this->get_logger(), "Features published");
}

}  // namespace aut_lidar_odometry

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500)
  );

  auto feature_extraction_node = std::make_shared<aut_lidar_odometry::FeatureExtraction>();
  
  exec.add_node(feature_extraction_node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Feature Extraction Started.\033[0m");

  exec.spin();
  rclcpp::shutdown();
  return 0;
}