#ifndef AUT_UTILS_UTILS_H_
#define AUT_UTILS_UTILS_H_

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

namespace aut_utils {

Eigen::Affine3f PoseToAffine(geometry_msgs::msg::Pose& pose);
Eigen::Affine3f TransformToAffine(geometry_msgs::msg::Transform& transform);
Eigen::Matrix4f TransformToMatrix(geometry_msgs::msg::Transform& transform);
geometry_msgs::msg::Pose MatrixToPose(Eigen::Matrix4f matrix);
Eigen::Matrix4f GetDifferenceTransformation(Eigen::Matrix4f m0, Eigen::Matrix4f m1);
Eigen::Matrix4f InverseTransformation(Eigen::Matrix4f m);
void OctreeVoxelGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out, float resolution);

}  // namespace aut_utils

#endif  // AUT_UTILS_UTILS_H_