// This code is inspired by:
// Shan, Tixiao and Englot, Brendan. LeGO-LOAM. https://github.com/RobustFieldAutonomyLab/LeGO-LOAM (Under BSD-3 License)

#ifndef LIDAR_ODOMETRY_OPTIMIZATION_H_
#define LIDAR_ODOMETRY_OPTIMIZATION_H_

#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace aut_lidar_odometry {

Eigen::Vector3f GetAnglesFromMatrix(Eigen::Matrix3f rot);

Eigen::Matrix4f GetMatrixFromTransform(float transform[6]);

Eigen::Matrix3f GetMatrixFromAngles(Eigen::Vector3f angles);

Eigen::Matrix4f Optimize(
  Eigen::Matrix4f transform_estimation,
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_edge_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_flat_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_edge_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_flat_points,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_edge,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_flat,
  float k
);
void ComputeEdgeCoeffs(
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_edge_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_edge_points,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_edge,
  std::vector<pcl::PointXYZI>* coeffs_corr,
  std::vector<pcl::PointXYZI>* points_corr, 
  std::vector<float>* weights_corr,
  float transform[6],
  float k
);
void ComputeFlatCoeffs(
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_flat_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_flat_points,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_flat,
  std::vector<pcl::PointXYZI>* coeffs_corr, 
  std::vector<pcl::PointXYZI>* points_corr, 
  std::vector<float>* weights_corr,
  float transform[6],
  float k
);

void TransformPoint(pcl::PointXYZI* p_in, pcl::PointXYZI* p_out, float transform[6]);

}  // namespace aut_lidar_odometry

#endif  // LIDAR_ODOMETRY_OPTIMIZATION_H_