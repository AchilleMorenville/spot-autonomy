#include "aut_local_planner/a_star.h"

#include <ctime>

#include <vector>
#include <utility>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <aut_utils/utils.h>

namespace aut_local_planner {

AStar::AStar(int x_dim, int y_dim, float cell_size, float min_obstacle_dist, 
             float max_obstacle_dist, float obstacle_multiplier_cost)
    : x_dim_(x_dim),
      y_dim_(y_dim),
      cell_size_(cell_size),
      min_obstacle_dist_(min_obstacle_dist),
      max_obstacle_dist_(max_obstacle_dist),
      obstacle_multiplier_cost_(obstacle_multiplier_cost) {
  grid_to_base_link_ = Eigen::Matrix4f::Identity();
  grid_to_map_ = Eigen::Matrix4f::Identity();
}

bool AStar::SetGrid(const std::vector<float>& grid, 
                    const Eigen::Matrix4f& grid_to_base_link, 
                    const Eigen::Matrix4f& map_to_base_link) {
  obstacle_dist_grid_ = grid;
  grid_to_base_link_ = grid_to_base_link;
  grid_to_map_ = grid_to_base_link_ * aut_utils::InverseTransformation(map_to_base_link);
  base_link_coord_ = GetCoordinates(grid_to_base_link_);

  float x = grid_to_base_link_(0, 3);
  float y = grid_to_base_link_(1, 3);
  Eigen::Matrix4f x_axis = Eigen::Matrix4f::Identity();
  x_axis(0, 3) = 1.0f;
  Eigen::Matrix4f grid_to_x_axis = grid_to_base_link_ * x_axis;
  float d_x = grid_to_x_axis(0, 3) - x;
  float d_y = grid_to_x_axis(1, 3) - y;
  float theta = std::atan2(d_y, d_x);

  Eigen::Affine2f af;
  af.translation() = Eigen::Vector2f(x, y);
  af.linear() = Eigen::Rotation2Df(theta).toRotationMatrix();
  af_base_link_to_grid_ = af.inverse();

  return Accessible(base_link_coord_);
}

bool AStar::SetTarget(const Eigen::Matrix4f& map_to_target) {
  Eigen::Matrix4f grid_to_target = grid_to_map_ * map_to_target;
  target_coord_ = GetCoordinates(grid_to_target);
  return Accessible(target_coord_);
}

bool AStar::InsideGrid(const Eigen::Matrix4f& map_to_target) {
  Eigen::Matrix4f grid_to_target = grid_to_map_ * map_to_target;
  std::pair<int, int> coord = GetCoordinates(grid_to_target);
  return InsideGrid(coord);
}

inline std::pair<int, int> AStar::GetCoordinates(const Eigen::Matrix4f& grid_to_pose) {
  return std::pair<int, int>{static_cast<int>(grid_to_pose(0, 3) / cell_size_),
                             static_cast<int>(grid_to_pose(1, 3) / cell_size_)};
}

inline std::pair<int, int> AStar::GetCoordinates(int idx) {
  return std::pair<int, int>{idx % x_dim_, idx / x_dim_};
}

inline int AStar::GetIndex(const std::pair<int, int>& p) {
  return p.first + x_dim_ * p.second;
}

inline bool AStar::Accessible(const std::pair<int, int>& p) {
  return InsideGrid(p) && obstacle_dist_grid_[GetIndex(p)] >= min_obstacle_dist_;
}

inline bool AStar::InsideGrid(const std::pair<int, int>& p) {
  return p.first >= 0 && p.first < x_dim_ && p.second >= 0 && p.second < y_dim_;
}

bool AStar::GetPath(std::vector<Eigen::Vector2f>& path) {
  std::vector<std::pair<int, int>> grid_path;
  bool found = FindPath(grid_path);
  if (!found) { return false; }

  for (std::pair<int, int>& p : grid_path) {
    path.push_back(GetPosition(p));
  }
  return true;
}

inline Eigen::Vector2f AStar::GetPosition(std::pair<int, int>& p) {
  return af_base_link_to_grid_ * Eigen::Vector2f((p.first + 0.5f) * cell_size_, 
                                                 (p.second + 0.5f) * cell_size_);
}

bool AStar::FindPath(std::vector<std::pair<int, int>>& path) {
  int start_idx = GetIndex(base_link_coord_);
  int target_idx = GetIndex(target_coord_);

  std::vector<int> open_set{start_idx};

  std::vector<float> g(x_dim_ * y_dim_, std::numeric_limits<float>::max());
  g[start_idx] = 0.0f;

  std::vector<float> f(x_dim_ * y_dim_, std::numeric_limits<float>::max());
  f[start_idx] = GetHeuristic(base_link_coord_);

  std::vector<int> came_from(x_dim_ * y_dim_, -1);

  int count = 0;
  while (!open_set.empty()) {

    ++count;

    auto best_it = open_set.begin();
    float best_f = f[*best_it];
    for (auto it = open_set.begin(); it != open_set.end(); ++it) {
      float it_f = f[*it];
      if (it_f < best_f) {
        best_it = it;
        best_f = it_f;
      }
    }
    if (*best_it == target_idx) {
      int walker_idx = *best_it;
      path.clear();
      path = std::vector<std::pair<int, int>>{GetCoordinates(walker_idx)};
      while (came_from[walker_idx] >= 0) {
        walker_idx = came_from[walker_idx];
        path.push_back(GetCoordinates(walker_idx));
      }
      std::reverse(path.begin(), path.end());

      std::cout << "Visited node : " << count << "\n";
      return true;
    }
    int current_idx = *best_it;
    open_set.erase(best_it);
    std::pair<int, int> current_coord = GetCoordinates(current_idx);
    std::vector<std::pair<int, int>> neighbors;
    GetNeighbors(current_coord, neighbors);
    for (const std::pair<int, int>& neighbor_coord : neighbors) {
      int neighbor_idx = GetIndex(neighbor_coord);
      float neighbor_g = g[current_idx] + EuclideanDist(current_coord, neighbor_coord) * GetCost(neighbor_idx);
      if (neighbor_g < g[neighbor_idx]) {
        g[neighbor_idx] = neighbor_g;
        f[neighbor_idx] = neighbor_g + GetHeuristic(neighbor_coord);
        came_from[neighbor_idx] = current_idx;
        if (std::find(open_set.begin(), open_set.end(), neighbor_idx) == open_set.end()) {
          open_set.push_back(neighbor_idx);
        }
      }
    }
  }
  return false;
}

inline float AStar::GetHeuristic(const std::pair<int, int>& p) {
  return EuclideanDist(p, target_coord_);
}

inline float AStar::EuclideanDist(const std::pair<int, int>& p1, 
                           const std::pair<int, int>& p2) {
  return cell_size_ * std::sqrt((p1.first - p2.first) * (p1.first - p2.first) + 
                                (p1.second - p2.second) * (p1.second - p2.second));
}

inline float AStar::GetCost(int idx) {
  if (obstacle_dist_grid_[idx] >= max_obstacle_dist_) {
    return 1.0f;
  }
  if (obstacle_dist_grid_[idx] < min_obstacle_dist_) {
    return obstacle_multiplier_cost_;
  }
  return (obstacle_multiplier_cost_ - 1.0f) * (1.0f - (obstacle_dist_grid_[idx] - min_obstacle_dist_)/(max_obstacle_dist_ - min_obstacle_dist_)) + 1.0f;
}

void AStar::GetNeighbors(const std::pair<int, int>& current_coord,
                         std::vector<std::pair<int, int>>& neighbors) {
  for (int i = -1; i < 2; ++i) {
    for (int j = -1; j < 2; ++j) {
      if (i == 0 && j == 0) { continue; }
      std::pair<int, int> neighbor{current_coord.first + i, 
                                   current_coord.second + j};
      if (Accessible(neighbor)) {
        neighbors.push_back(neighbor);
      }
    }
  }
}

}  // namespace aut_local_planner

int main(int argc, char * argv[]) {

  aut_local_planner::AStar a_star(10, 10, 0.03f, 0.25f, 0.55f, 3.0f);

  Eigen::Matrix4f map_to_base_link = Eigen::Matrix4f::Identity();
  map_to_base_link(0, 3) = 0.015f;
  map_to_base_link(1, 3) = 0.015f;

  std::vector<float> grid(10 * 10, 1.0f);
  grid[4] = 0.0f;

  std::cout << a_star.SetGrid(grid, map_to_base_link, map_to_base_link) << "\n";

  Eigen::Matrix4f map_to_target = Eigen::Matrix4f::Identity();
  map_to_target(0, 3) = 9 * 0.03f + 0.015f;
  map_to_target(1, 3) = 3 * 0.03f + 0.015f;

  std::cout << a_star.SetTarget(map_to_target) << "\n";

  std::vector<Eigen::Vector2f> path;

  std::clock_t t_start = std::clock();
  std::cout << a_star.GetPath(path) << "\n";
  std::cout << "Time taken : " << static_cast<double>(1000.0 * (std::clock() - t_start) / CLOCKS_PER_SEC) << "ms\n";

  std::cout << "Path length : " << path.size() << "\n"; 

  std::cout << "------- Path -------\n";
  for (int i = 0; i < (int) path.size(); ++i) {
    std::cout << "Position " << i << ": \n" << path[i] << "\n";
  }

  // Eigen::Affine2f af;
  // af.translation() = Eigen::Vector2f(1.5f, 2.0f);
  // af.linear() = Eigen::Rotation2Df(M_PI / 3.0f).toRotationMatrix();

  // std::cout << af.matrix() << "\n";

  // std::cout << "=============\n";
  // Eigen::Matrix3f m = Eigen::Matrix3f::Identity();
  // m(0, 2) = 1.5f; 
  // m(1, 2) = 2.0f;
  // m(0, 0) = std::cos(M_PI / 3.0f);
  // m(0, 1) = -std::sin(M_PI / 3.0f);
  // m(1, 0) = std::sin(M_PI / 3.0f);
  // m(1, 1) = std::cos(M_PI / 3.0f);

  // std::cout << m << "\n";
  // std::cout << "=============\n";

  // Eigen::Affine2f af_2(m);
  // std::cout << af_2.matrix() << "\n";

  // std::cout << af.inverse() * Eigen::Vector2f(1.0f, 3.0f) << "\n";
}