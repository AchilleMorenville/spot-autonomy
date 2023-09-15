// This code is inspired by:
// Shan, Tixiao and Englot, Brendan. LeGO-LOAM. https://github.com/RobustFieldAutonomyLab/LeGO-LOAM (Under BSD-3 License)

#include "aut_lidar_odometry/optimization.h"

#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/opencv.hpp>

namespace aut_lidar_odometry {

Eigen::Matrix4f Optimize(
  Eigen::Matrix4f transform_estimation,
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_edge_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_flat_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_edge_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_flat_points,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_edge,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_flat,
  float k
) {

  bool is_degenerate = false;
  cv::Mat mat_P (6, 6, CV_32F, cv::Scalar::all(0));

  // Init transformation
  Eigen::Vector3f angles = GetAnglesFromMatrix(transform_estimation.block<3, 3>(0, 0));

  float transform[6];
  transform[0] = angles(0);
  transform[1] = angles(1);
  transform[2] = angles(2);
  transform[3] = transform_estimation(0, 3);
  transform[4] = transform_estimation(1, 3);
  transform[5] = transform_estimation(2, 3);

  std::vector<pcl::PointXYZI> coeffs_corr;
  std::vector<pcl::PointXYZI> points_corr;
  std::vector<float> weights_corr;

  for (int iter = 0; iter < 30; iter++) {

    coeffs_corr.clear();
    points_corr.clear();
    weights_corr.clear();

    // find corners correspondences
    ComputeEdgeCoeffs(
      source_edge_points,
      target_edge_points,
      kdtree_target_edge,
      &coeffs_corr, 
      &points_corr, 
      &weights_corr,
      transform,
      k
    );

    // find surfs correspondences
    ComputeFlatCoeffs(
      source_flat_points,
      target_flat_points,
      kdtree_target_flat,
      &coeffs_corr, 
      &points_corr, 
      &weights_corr,
      transform,
      k
    );

    cv::Mat mat_A (coeffs_corr.size(), 6, CV_32F, cv::Scalar::all(0));
    cv::Mat mat_At (6, coeffs_corr.size(), CV_32F, cv::Scalar::all(0));
    cv::Mat mat_AtA (6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat mat_B (coeffs_corr.size(), 1, CV_32F, cv::Scalar::all(0));
    cv::Mat mat_AtB (6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat mat_X (6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat mat_W (coeffs_corr.size(), coeffs_corr.size(), CV_32F, cv::Scalar::all(0));

    float s1 = std::sin(transform[0]);
    float c1 = std::cos(transform[0]);
    float s2 = std::sin(transform[1]);
    float c2 = std::cos(transform[1]);
    float s3 = std::sin(transform[2]);
    float c3 = std::cos(transform[2]);

    // float tx = transform[3];
    // float ty = transform[4];
    // float tz = transform[5];

    if (points_corr.size() < 50) {
      std::cout << "Few point cloud" << std::endl;
      continue;
    }

    for (int i = 0; i < (int) points_corr.size(); i++) {

      float x = points_corr[i].x;
      float y = points_corr[i].y;
      float z = points_corr[i].z;

      // rx
      mat_A.at<float>(i, 0) = ((-s1 * c2) * x + (-s1 * s2 * s3 - c3 * c1) * y + (c1 * s3 - s1 * c3 * s2) * z) * coeffs_corr[i].x
                            + ((c2 * c1) * x + (-s1 * c3 + c1 * s2 * s3) * y + (c3 * c1 * s2 + s1 * s3) * z) * coeffs_corr[i].y;
      // ry
      mat_A.at<float>(i, 1) = ((-c1 * s2) * x + (c1 * c2 * s3) * y + (c1 * c3 * c2) * z) * coeffs_corr[i].x
                            + ((-s2 * s1) * x + (s1 * c2 * s3) * y + (c3 * s1 * c2) * z) * coeffs_corr[i].y
                            + ((-c2) * x + (-s2 * s3) * y + (-s2 * c3) * z) * coeffs_corr[i].z;
      
      // rz
      mat_A.at<float>(i, 2) = ((c1 * s2 * c3 + s3 * s1) * y + (s1 * c3 - c1 * s3 * s2) * z) * coeffs_corr[i].x
                            + ((-c1 * s3 + s1 * s2 * c3) * y + (-s3 * s1 * s2 - c1 * c3) * z) * coeffs_corr[i].y
                            + ((c2 * c3) * y + (-c2 * s3) * z) * coeffs_corr[i].z;
      
      // tx
      mat_A.at<float>(i, 3) = coeffs_corr[i].x;
      
      //ty
      mat_A.at<float>(i, 4) = coeffs_corr[i].y;
      
      // tz
      mat_A.at<float>(i, 5) = coeffs_corr[i].z;

      // dist
      mat_B.at<float>(i, 0) = coeffs_corr[i].intensity;

      // weight
      mat_W.at<float>(i, i) = weights_corr[i];

    }

    cv::transpose(mat_A, mat_At);
    mat_AtA = mat_At * (mat_W * mat_A);
    mat_AtB = - mat_At * (mat_W * mat_B);
    cv::solve(mat_AtA, mat_AtB, mat_X, cv::DECOMP_QR);

    if (iter == 0) {
      cv::Mat mat_E(1, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat mat_V(6, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat mat_V2(6, 6, CV_32F, cv::Scalar::all(0));

      cv::eigen(mat_AtA, mat_E, mat_V);
      mat_V.copyTo(mat_V2);

      is_degenerate = false;
      for (int i = 5; i >= 0; i--) {
        if (mat_E.at<float>(0, i) < 100) {
          for (int j = 0; j < 6; j++) {
            mat_V2.at<float>(i, j) = 0;
          }
          is_degenerate = true;
        } else {
          break;
        }
      }

      mat_P = mat_V.inv() * mat_V2;
    }

    if (is_degenerate) {
      cv::Mat mat_X2(6, 1, CV_32F, cv::Scalar::all(0));
      mat_X.copyTo(mat_X2);
      mat_X = mat_P * mat_X2;
    }

    transform[0] += mat_X.at<float>(0, 0);
    transform[1] += mat_X.at<float>(1, 0);
    transform[2] += mat_X.at<float>(2, 0);
    transform[3] += mat_X.at<float>(3, 0);
    transform[4] += mat_X.at<float>(4, 0);
    transform[5] += mat_X.at<float>(5, 0);

    float drx = mat_X.at<float>(0, 0) * 180 / M_PI;
    float dry = mat_X.at<float>(1, 0) * 180 / M_PI;
    float drz = mat_X.at<float>(2, 0) * 180 / M_PI;

    float dtx = mat_X.at<float>(3, 0) * 100;
    float dty = mat_X.at<float>(4, 0) * 100;
    float dtz = mat_X.at<float>(5, 0) * 100;

    float delta_r = std::sqrt(drx * drx + dry * dry + drz * drz);
    float delta_t = std::sqrt(dtx * dtx + dty * dty + dtz * dtz);

    if (delta_t < 0.05 && delta_r < 0.05) { 
      return GetMatrixFromTransform(transform);
    }
  }

  std::cout << "Attained the end of optimization iter" << std::endl;

  return GetMatrixFromTransform(transform);
}

void ComputeEdgeCoeffs(
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_edge_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_edge_points,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_edge,
  std::vector<pcl::PointXYZI>* coeffs_corr,
  std::vector<pcl::PointXYZI>* points_corr, 
  std::vector<float>* weights_corr,
  float transform[6],
  float k
) {
  
  std::vector<int> point_search_idx;
  std::vector<float> point_search_sq_dist;

  pcl::PointXYZI point_transformed;
  pcl::PointXYZI coeff;

  for (int i = 0; i < (int) source_edge_points->points.size(); i++) {

    TransformPoint(&(source_edge_points->points[i]), &point_transformed, transform);

    kdtree_target_edge->nearestKSearch(point_transformed, 5, point_search_idx, point_search_sq_dist);

    if (point_search_sq_dist[4] < 0.3) { // The last point is close enough

      float mean_x = 0;
      float mean_y = 0;
      float mean_z = 0;

      for (int j = 0; j < 5; j++) {
        mean_x += target_edge_points->points[point_search_idx[j]].x;
        mean_y += target_edge_points->points[point_search_idx[j]].y;
        mean_z += target_edge_points->points[point_search_idx[j]].z;
      }
      mean_x /= 5;
      mean_y /= 5;
      mean_z /= 5;

      cv::Mat mat_cov (3, 3, CV_32F, cv::Scalar::all(0));
      cv::Mat mat_E_cov (1, 3, CV_32F, cv::Scalar::all(0));
      cv::Mat mat_V_cov (3, 3, CV_32F, cv::Scalar::all(0));

      float cov11 = 0, cov12 = 0, cov13 = 0, cov22 = 0, cov23 = 0, cov33 = 0;
      for (int j = 0; j < 5; j++) {

        float diff_mean_x = target_edge_points->points[point_search_idx[j]].x - mean_x;
        float diff_mean_y = target_edge_points->points[point_search_idx[j]].y - mean_y;
        float diff_mean_z = target_edge_points->points[point_search_idx[j]].z - mean_z;

        cov11 += diff_mean_x * diff_mean_x;
        cov12 += diff_mean_x * diff_mean_y;
        cov13 += diff_mean_x * diff_mean_z;
        cov22 += diff_mean_y * diff_mean_y;
        cov23 += diff_mean_y * diff_mean_z;
        cov33 += diff_mean_z * diff_mean_z;

      }
      cov11 /= 5;
      cov12 /= 5;
      cov13 /= 5;
      cov22 /= 5;
      cov23 /= 5;
      cov33 /= 5;

      mat_cov.at<float>(0, 0) = cov11; mat_cov.at<float>(0, 1) = cov12; mat_cov.at<float>(0, 2) = cov13;
      mat_cov.at<float>(1, 0) = cov11; mat_cov.at<float>(1, 1) = cov22; mat_cov.at<float>(1, 2) = cov23;
      mat_cov.at<float>(2, 0) = cov13; mat_cov.at<float>(2, 1) = cov23; mat_cov.at<float>(2, 2) = cov33; 

      cv::eigen(mat_cov, mat_E_cov, mat_V_cov);

      if (mat_E_cov.at<float>(0, 0) > 3 * mat_E_cov.at<float>(0, 1)) { // We are on an edge

        float x0 = point_transformed.x;
        float y0 = point_transformed.y;
        float z0 = point_transformed.z;

        float x1 = mean_x + 0.1 * mat_V_cov.at<float>(0, 0);
        float y1 = mean_y + 0.1 * mat_V_cov.at<float>(0, 1);
        float z1 = mean_z + 0.1 * mat_V_cov.at<float>(0, 2);

        float x2 = mean_x - 0.1 * mat_V_cov.at<float>(0, 0);
        float y2 = mean_y - 0.1 * mat_V_cov.at<float>(0, 1);
        float z2 = mean_z - 0.1 * mat_V_cov.at<float>(0, 2);

        float cross1 = (y0 - y1) * (z0 - z2) - (z0 - z1) * (y0 - y2);
        float cross2 = (z0 - z1) * (x0 - x2) - (x0 - x1) * (z0 - z2);
        float cross3 = (x0 - x1) * (y0 - y2) - (y0 - y1) * (x0 - x2);

        float norm_normal = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

        float norm_cross = std::sqrt(cross1 * cross1 + cross2 * cross2 + cross3 * cross3);

        float coeff_x = ((y1 - y2) * (cross3) - (z1 - z2) * (cross2)) / (norm_normal * norm_cross);
        float coeff_y = ((z1 - z2) * (cross1) - (x1 - x2) * (cross3)) / (norm_normal * norm_cross);
        float coeff_z = ((x1 - x2) * (cross2) - (y1 - y2) * (cross1)) / (norm_normal * norm_cross);

        float dist = (norm_cross / norm_normal);

        if (fabs(dist) <= k) {
          float weight = (1 - (dist / k) * (dist / k)) * (1 - (dist / k) * (dist / k));
          coeff.x = coeff_x;
          coeff.y = coeff_y;
          coeff.z = coeff_z;
          coeff.intensity = dist;
          coeffs_corr->push_back(coeff);
          weights_corr->push_back(weight);
          points_corr->push_back(source_edge_points->points[i]);
        }
      }   
    }
  }
}

void ComputeFlatCoeffs(
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_flat_points,
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_flat_points,
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_target_flat,
  std::vector<pcl::PointXYZI>* coeffs_corr, 
  std::vector<pcl::PointXYZI>* points_corr, 
  std::vector<float>* weights_corr,
  float transform[6],
  float k
) {

  std::vector<int> point_search_idx;
  std::vector<float> point_search_sq_dist;

  pcl::PointXYZI point_transformed;
  pcl::PointXYZI coeff;

  for (int i = 0; i < (int) source_flat_points->points.size(); i++) {
    TransformPoint(&(source_flat_points->points[i]), &point_transformed, transform);
    kdtree_target_flat->nearestKSearch(point_transformed, 5, point_search_idx, point_search_sq_dist);

    if (point_search_sq_dist[4] < 0.3) {

      cv::Mat mat_A0 (5, 3, CV_32F, cv::Scalar::all(0));
      cv::Mat mat_B0 (5, 1, CV_32F, cv::Scalar::all(-1));
      cv::Mat mat_X0 (3, 1, CV_32F, cv::Scalar::all(0));

      for (int j = 0; j < 5; j++) {
        mat_A0.at<float>(j, 0) = target_flat_points->points[point_search_idx[j]].x;
        mat_A0.at<float>(j, 1) = target_flat_points->points[point_search_idx[j]].y;
        mat_A0.at<float>(j, 2) = target_flat_points->points[point_search_idx[j]].z;
      }

      cv::solve(mat_A0, mat_B0, mat_X0, cv::DECOMP_QR);

      float pa = mat_X0.at<float>(0, 0);
      float pb = mat_X0.at<float>(0, 1);
      float pc = mat_X0.at<float>(0, 2);
      float pd = 1;

      float norm_normal = std::sqrt(pa * pa + pb * pb + pc * pc);

      pa /= norm_normal;
      pb /= norm_normal;
      pc /= norm_normal;
      pd /= norm_normal;

      bool plane_valid = true;
      for (int j = 0; j < 5; j++) {
        if (fabs(pa * target_flat_points->points[point_search_idx[j]].x + 
                 pb * target_flat_points->points[point_search_idx[j]].y +
                 pc * target_flat_points->points[point_search_idx[j]].z + pd) > 0.05) {
          plane_valid = false;
          break;
        }
      }

      if (plane_valid) {

        float dist = pa * point_transformed.x + pb * point_transformed.y + pc * point_transformed.z + pd;

        if (fabs(dist) <= k) {
          float weight = (1 - (dist / k) * (dist / k)) * (1 - (dist / k) * (dist / k));
          coeff.x = pa;
          coeff.y = pb;
          coeff.z = pc;
          coeff.intensity = dist;
          coeffs_corr->push_back(coeff);
          weights_corr->push_back(weight);
          points_corr->push_back(source_flat_points->points[i]);
        }
      }
    }
  }
}

void TransformPoint(pcl::PointXYZI* p_in, pcl::PointXYZI* p_out, float transform[6]) {
  float c1 = std::cos(transform[0]);
  float c2 = std::cos(transform[1]);
  float c3 = std::cos(transform[2]);

  float s1 = std::sin(transform[0]);
  float s2 = std::sin(transform[1]);
  float s3 = std::sin(transform[2]);

  p_out->x = (c1 * c2) * p_in->x + (c1 * s2 * s3 - c3 * s1) * p_in->y + (s1 * s3 + c1 * c3 * s2) * p_in->z + transform[3];
  p_out->y = (c2 * s1) * p_in->x + (c1 * c3 + s1 * s2 * s3) * p_in->y + (c3 * s1 * s2 - c1 * s3) * p_in->z + transform[4];
  p_out->z = (-s2) * p_in->x + (c2 * s3) * p_in->y + (c2 * c3) * p_in->z + transform[5];
  p_out->intensity = p_in->intensity;
}

Eigen::Vector3f GetAnglesFromMatrix(Eigen::Matrix3f rot) {
  float alpha = std::atan2(rot(1, 0), rot(0, 0));
  float beta = std::atan2(-rot(2, 0), std::sqrt(1 - rot(2, 0) * rot(2, 0)));
  float gamma = std::atan2(rot(2, 1), rot(2, 2));
  Eigen::Vector3f angles;
  angles << alpha, beta, gamma;
  return angles;
}

Eigen::Matrix4f GetMatrixFromTransform(float transform[6]) {
  Eigen::Matrix4f m;
  m = Eigen::Matrix4f::Identity();

  Eigen::Vector3f angles(transform[0], transform[1], transform[2]);

  m.block<3, 3>(0, 0) = GetMatrixFromAngles(angles);
  m(0, 3) = transform[3];
  m(1, 3) = transform[4];
  m(2, 3) = transform[5];

  return m;
}

Eigen::Matrix3f GetMatrixFromAngles(Eigen::Vector3f angles) {
  Eigen::Matrix3f rot;

  float c1 = std::cos(angles(0));
  float c2 = std::cos(angles(1));
  float c3 = std::cos(angles(2));

  float s1 = std::sin(angles(0));
  float s2 = std::sin(angles(1));
  float s3 = std::sin(angles(2));

  rot(0, 0) = c1 * c2;
  rot(0, 1) = c1 * s2 * s3 - c3 * s1;
  rot(0, 2) = s1 * s3 + c1 * c3 * s2;

  rot(1, 0) = c2 * s1;
  rot(1, 1) = c1 * c3 + s1 * s2 * s3;
  rot(1, 2) = c3 * s1 * s2 - c1 * s3;

  rot(2, 0) = -s2;
  rot(2, 1) = c2 * s3;
  rot(2, 2) = c2 * c3;

  return rot;
}

}