#include "aut_common/graph.h"

#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <limits>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <aut_utils/utils.h>

namespace aut_common {

Graph::Graph() : n_(0) {}

void Graph::AddVertex(Eigen::Matrix4f pose) {
  std::vector<int> vect;
  graph_[n_] = vect;
  poses_.push_back(pose);
  n_++;
}

bool Graph::AddEdge(int id_a, int id_b) {
  if (graph_.find(id_a) == graph_.end() || graph_.find(id_b) == graph_.end()) {
    return false;
  }
  graph_[id_a].push_back(id_b);
  graph_[id_b].push_back(id_a);
  return true;
}

void Graph::UpdatePoses(std::vector<Eigen::Matrix4f>& updated_poses) {
  int size_poses = poses_.size();
  for (int i = 0; i < size_poses; i++) {
    poses_[i] = updated_poses[i];
  }
}

void Graph::TidyGraph() {
  for (auto& it: graph_) {
    std::sort(it.second.begin(), it.second.end());
    it.second.erase(std::unique( it.second.begin(), it.second.end() ), it.second.end());
  }
}

void Graph::Simplify(float resolution) {

  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_nodes(new pcl::PointCloud<pcl::PointXYZL>());

  for (auto& it: graph_) {
    pcl::PointXYZL point;
    point.x = poses_[it.first](0, 3);
    point.y = poses_[it.first](1, 3);
    point.z = poses_[it.first](2, 3);
    point.label = it.first;
    cloud_nodes->push_back(point);
  }

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZL> octree(resolution);
  octree.setInputCloud(cloud_nodes);
  octree.addPointsFromInputCloud();

  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_selected_nodes(new pcl::PointCloud<pcl::PointXYZL>());

  int count = 0;
  int total = 0;
  for (auto it = octree.leaf_breadth_begin(); it != octree.leaf_breadth_end(); ++it) {
    pcl::IndicesPtr index_vector(new std::vector<int>);

    std::vector<int> indices;

    pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
    // container.getPointIndices(*index_vector);
    container.getPointIndices(indices);

    std::cout << "Leaf n: " << count++ << ", n points: " << indices.size() << "\n";
    total += indices.size();

    float mean_x = 0;
    float mean_y = 0;
    float mean_z = 0;
    for (int i = 0; i < indices.size(); ++i) {
      mean_x += cloud_nodes->points[indices[i]].x;
      mean_y += cloud_nodes->points[indices[i]].y;
      mean_z += cloud_nodes->points[indices[i]].z;
    }
    mean_x /= indices.size();
    mean_y /= indices.size();
    mean_z /= indices.size();

    int closest_idx = -1;
    float min_sq_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < indices.size(); ++i) {

      float sq_dist = (cloud_nodes->points[indices[i]].x - mean_x) * (cloud_nodes->points[indices[i]].x - mean_x)
                    + (cloud_nodes->points[indices[i]].y - mean_y) * (cloud_nodes->points[indices[i]].y - mean_y)
                    + (cloud_nodes->points[indices[i]].z - mean_z) * (cloud_nodes->points[indices[i]].z - mean_z);
      if (sq_dist < min_sq_dist) {
        closest_idx = indices[i];
        min_sq_dist = sq_dist;
      }
    }

    cloud_selected_nodes->push_back(cloud_nodes->points[closest_idx]);
  }

  std::cout << "Before : " << cloud_nodes->points.size() << ", After : " << cloud_selected_nodes->points.size() << ", Total visited: " << total << "\n";

  pcl::KdTreeFLANN<pcl::PointXYZL>::Ptr kdtree_selected_nodes(new pcl::KdTreeFLANN<pcl::PointXYZL>);
  kdtree_selected_nodes->setInputCloud(cloud_selected_nodes);

  for (int i = 0; i < cloud_nodes->points.size(); ++i) {

    std::vector<int> point_search_idx;
    std::vector<float> point_search_sq_dist;
    kdtree_selected_nodes->nearestKSearch(cloud_nodes->points[i], 1, point_search_idx, point_search_sq_dist);

    if (point_search_sq_dist.size() != 0 && cloud_selected_nodes->points[point_search_idx[0]].label != cloud_nodes->points[i].label) {
      MergeVertices(cloud_nodes->points[i].label, cloud_selected_nodes->points[point_search_idx[0]].label);
    }
  }
}

void Graph::MergeVertices(int source_idx, int target_idx) {

  std::vector<int> source_edges = graph_[source_idx];
  for (int i = 0; i < source_edges.size(); ++i) {

    int source_neighbor_node = source_edges[i];
    std::vector<int>::iterator it = std::find(graph_[source_neighbor_node].begin(), graph_[source_neighbor_node].end(), source_idx);

    if (source_neighbor_node == target_idx && it != graph_[source_neighbor_node].end()) {
      graph_[source_neighbor_node].erase(it);
    } else {
      *it = target_idx;
    }

    if (source_neighbor_node != target_idx) {
      graph_[target_idx].push_back(source_neighbor_node);
    }

  }
  graph_.erase(source_idx);

  TidyGraph();
}

std::vector<int> Graph::GetEdges(int id) {
  if (graph_.find(id) == graph_.end()) {
    return std::vector<int>{}; // Return empty vector
  }
  return graph_[id];
}

Eigen::Matrix4f Graph::GetPose(int id) {
  if (graph_.find(id) == graph_.end()) {
    return Eigen::Matrix4f::Zero(); // Return empty vector
  }
  return poses_[id];
}

float Graph::GetEuclideanDist(int id1, int id2) {
  return (poses_[id1].block<3, 1>(0, 3) - poses_[id2].block<3, 1>(0, 3)).norm();
}

bool Graph::AStar(int start, int goal, std::vector<int> &path) {

  std::vector<int> open_set;
  open_set.push_back(start);

  std::unordered_map<int, int> came_from;

  std::unordered_map<int, float> g_score;
  g_score[start] = 0;

  std::unordered_map<int, float> f_score;
  f_score[start] = GetEuclideanDist(start, goal);

  while (!open_set.empty()) {

    std::vector<int>::iterator current_it = open_set.begin();
    int current = *current_it;

    for (auto it = open_set.begin(); it != open_set.end(); it++) {
      if (f_score[*it] < f_score[current]) {
        current_it = it;
        current = *it;
      }
    }

    if (current == goal) {

      // Compute path

      while (came_from.find(current) != came_from.end()) {
        path.push_back(current);
        current = came_from[current];
      }

      path.push_back(start);

      std::reverse(path.begin(), path.end());

      return true;
    }

    open_set.erase(current_it);

    std::vector<int> neighbors = GetEdges(current);

    for (int neighbor : neighbors) {

      if (GetEuclideanDist(current, neighbor) > 1.5) {
        continue;
      }

      float tentative_g_score = g_score[current] + GetEuclideanDist(current, neighbor);

      float neighbor_g_score = g_score.find(neighbor) == g_score.end() ? std::numeric_limits<float>::max() : g_score[neighbor];

      if (tentative_g_score < neighbor_g_score) {

        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g_score;
        f_score[neighbor] = tentative_g_score + GetEuclideanDist(neighbor, goal);

        std::vector<int>::iterator it = std::find(open_set.begin(), open_set.end(), neighbor);
        if (it == open_set.end()) {
          open_set.push_back(neighbor);
        }

      }

    }

  }

  return false;
}

int Graph::ClosestNode(Eigen::Matrix4f pose) {
  Eigen::Vector3f position = pose.block<3, 1>(0, 3);
  return ClosestNode(position);
}

int Graph::ClosestNode(Eigen::Vector3f position) {
  int closest_idx = -1;
  float closest_dist = std::numeric_limits<float>::max();
  for (auto& it: graph_) {

    Eigen::Vector3f bewteen_vector = poses_[it.first].block<3, 1>(0, 3) - position;
    float dist = bewteen_vector.squaredNorm();
    if (std::abs(bewteen_vector(2)) < 1 && dist < closest_dist) {
      closest_dist = dist;
      closest_idx = it.first;
    }
  }
  return closest_idx;
}

void Graph::SaveFile(std::string file_path) {
  std::fstream fs;
  fs.open(file_path, std::fstream::out | std::fstream::trunc);

  fs << poses_.size() << "\n";

  for (int i = 0; i < poses_.size(); i++) {

    fs << graph_[i].size() << " ";

    for (int j = 0; j < graph_[i].size(); j++) {

      fs << graph_[i][j] << " ";

    }

    fs << "\n";

    Eigen::Quaternionf quat(poses_[i].block<3, 3>(0, 0));

    fs << poses_[i](0, 3) << " " 
       << poses_[i](1, 3) << " " 
       << poses_[i](2, 3) << " " 
       << quat.w() << " " 
       << quat.x() << " " 
       << quat.y() << " " 
       << quat.z() << "\n";
  }

  fs.close();
}

void Graph::LoadFile(std::string file_path) {
  std::fstream fs;
  fs.open(file_path, std::fstream::in);

  int graph_size;
  fs >> graph_size;

  n_ = graph_size;
  for (int i = 0; i < graph_size; i++) {
    int number_edges;
    fs >> number_edges;
    for (int j = 0; j < number_edges; j++) {
      int edge;
      fs >> edge;
      graph_[i].push_back(edge);
    }

    float x;
    float y;
    float z;

    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;

    fs >> x;
    fs >> y;
    fs >> z;

    fs >> quat_w;
    fs >> quat_x;
    fs >> quat_y;
    fs >> quat_z;

    Eigen::Quaternionf quat(quat_w, quat_x, quat_y, quat_z);

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = quat.toRotationMatrix();
    pose(0, 3) = x;
    pose(1, 3) = y;
    pose(2, 3) = z;
    poses_.push_back(pose);
  }

  fs.close();
}

visualization_msgs::msg::MarkerArray Graph::GetMarkerArrayWithPath(std::vector<int> &path) {
  visualization_msgs::msg::MarkerArray marker_array;

  int count = 0;
  for (auto& it: graph_) {
    for (int neighbor: it.second) {

      std::vector<int>::iterator it_first = std::find(path.begin(), path.end(), it.first);
      std::vector<int>::iterator it_second = std::find(path.begin(), path.end(), neighbor);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.ns = std::string("arrows");
      marker.id = count;
      marker.type = 0;
      marker.action = 0;

      geometry_msgs::msg::Point point_1;
      point_1.x = poses_[it.first](0, 3);
      point_1.y = poses_[it.first](1, 3);
      point_1.z = poses_[it.first](2, 3);

      geometry_msgs::msg::Point point_2;
      point_2.x = poses_[neighbor](0, 3);
      point_2.y = poses_[neighbor](1, 3);
      point_2.z = poses_[neighbor](2, 3);
      marker.points.push_back(point_1);
      marker.points.push_back(point_2);

      marker.scale.x = 0.02;
      marker.scale.y = 0.05;
      marker.scale.z = 0;

      marker.lifetime.sec = 1;

      if (it_first != path.end() && it_second != path.end()) {
        if (it_first - path.begin() < it_second - path.begin()) {
          marker.color.a = 1.0;
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
        } else {
          continue;
        }
      } else {
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      }

      marker_array.markers.push_back(marker);

      count++;
    }
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray Graph::GetMarkerArray() {
  visualization_msgs::msg::MarkerArray marker_array;

  int count = 0;
  for (auto& it: graph_) {
    for (int neighbor: it.second) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.ns = std::string("arrows");
      marker.id = count;
      marker.type = 0;
      marker.action = 0;

      geometry_msgs::msg::Point point_1;
      point_1.x = poses_[it.first](0, 3);
      point_1.y = poses_[it.first](1, 3);
      point_1.z = poses_[it.first](2, 3);

      geometry_msgs::msg::Point point_2;
      point_2.x = poses_[neighbor](0, 3);
      point_2.y = poses_[neighbor](1, 3);
      point_2.z = poses_[neighbor](2, 3);
      marker.points.push_back(point_1);
      marker.points.push_back(point_2);

      marker.scale.x = 0.02;
      marker.scale.y = 0.05;
      marker.scale.z = 0;

      marker.lifetime.sec = 1;

      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      marker_array.markers.push_back(marker);

      count++;
    }
  }

  return marker_array;
}

void Graph::PrintGraph() {
  std::cout << "Edges : \n";
  for (auto& it: graph_) {
    std::cout << it.first << " : ";
    for (int i = 0; i < it.second.size(); i++) {
      if (i == 0) {
        std::cout << it.second[i];
      } else {
        std::cout << ", " << it.second[i];
      }
    }
    std::cout << "\n";
  }
}

void Graph::Reset() {
  n_ = 0;
  graph_.clear();
  poses_.clear();
}

bool Graph::RemoveEdge(int id1, int id2) {
  if (graph_.find(id1) == graph_.end() || graph_.find(id2) == graph_.end()) {
    return false;
  }
  auto it1 = std::find(graph_[id1].begin(), graph_[id1].end(), id2);
  if (it1 != graph_[id1].end()) {
    graph_[id1].erase(it1);
  }
  auto it2 = std::find(graph_[id2].begin(), graph_[id2].end(), id1);
  if (it2 != graph_[id2].end()) {
    graph_[id2].erase(it2);
  }
  return true;
}

bool Graph::Empty() {
  return graph_.empty();
}

}  // namespace aut_common