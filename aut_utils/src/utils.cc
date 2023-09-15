#include "aut_utils/utils.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_iterator.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace aut_utils {

Eigen::Affine3f PoseToAffine(geometry_msgs::msg::Pose& pose) {
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Vector3f t(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Affine3f a;
  a.translation() = t;
  a.linear() = q.toRotationMatrix();
  return a;
}

Eigen::Affine3f TransformToAffine(geometry_msgs::msg::Transform& transform) {
  Eigen::Quaternionf q(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  Eigen::Vector3f t(transform.translation.x, transform.translation.y, transform.translation.z);
  Eigen::Affine3f a;
  a.translation() = t;
  a.linear() = q.toRotationMatrix();
  return a;
}

Eigen::Matrix4f TransformToMatrix(geometry_msgs::msg::Transform& transform) {
  Eigen::Quaternionf rot(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  Eigen::Vector3f trans(transform.translation.x, transform.translation.y, transform.translation.z);
  Eigen::Affine3f affine;
  affine.translation() = trans;
  affine.linear() = rot.toRotationMatrix();
  Eigen::Matrix4f matrix;
  matrix = affine.matrix();
  return matrix;
}

geometry_msgs::msg::Pose MatrixToPose(Eigen::Matrix4f matrix) {
  
  geometry_msgs::msg::Pose pose;

  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);

  Eigen::Quaternionf quat(matrix.block<3, 3>(0, 0));

  pose.orientation.w = quat.w();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();

  return pose;
}

Eigen::Matrix4f GetDifferenceTransformation(Eigen::Matrix4f m0, Eigen::Matrix4f m1) {
  return InverseTransformation(m0) * m1;
}

Eigen::Matrix4f InverseTransformation(Eigen::Matrix4f m) {
  Eigen::Matrix4f inv = Eigen::Matrix4f::Identity();
  inv.block<3, 3>(0, 0) = m.block<3, 3>(0, 0).transpose();
  inv.block<3, 1>(0, 3) = - m.block<3, 3>(0, 0).transpose() * m.block<3, 1>(0, 3);
  return inv;
}

void OctreeVoxelGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out, float resolution) {
  
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_leaf;
  voxel_grid_leaf.setLeafSize(resolution, resolution, resolution);

  float octree_resolution = 8.0f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree(octree_resolution);
  octree.setInputCloud(cloud_in);
  octree.addPointsFromInputCloud();

  for (auto it = octree.leaf_breadth_begin(); it != octree.leaf_breadth_end(); ++it) {

    pcl::IndicesPtr indexVector(new std::vector<int>);
    pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
    container.getPointIndices(*indexVector);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_leaf(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_grid_leaf.setInputCloud(cloud_in);
    voxel_grid_leaf.setIndices(indexVector);
    voxel_grid_leaf.filter(*filtered_leaf);
    *cloud_out += *filtered_leaf;
  }

}

}  // namespace aut_utils