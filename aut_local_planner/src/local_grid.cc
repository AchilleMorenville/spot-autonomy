#include "aut_local_planner/local_grid.h"

#include <cmath>
#include <vector>
#include <unordered_set>

#include <Eigen/Core>

#include "aut_local_planner/utils.h"
#include "aut_utils/utils.h"

namespace aut_local_planner {

LocalGrid::Pose::Pose() : x(0.0f), y(0.0f), theta(0.0f) {}

LocalGrid::Pose::Pose(float x, float y, float theta) : x(x), y(y), theta(theta) {}

LocalGrid::Pose::Pose(const Pose& p) : x(p.x), y(p.y), theta(p.theta) {}

LocalGrid::Pose& LocalGrid::Pose::operator=(Pose p) {
  std::swap(x, p.x);
  std::swap(y, p.y);
  std::swap(theta, p.theta);
  return *this;
}

LocalGrid::Pose::~Pose() {};

LocalGrid::State::State() 
  : pose(), 
    parent_idx_3D(-1), 
    motion_from_parent(), 
    g(0.0f),
    h(0.0f) {}

LocalGrid::State::State(
    Pose pose, 
    int parent_idx_3D, 
    Motion motion_from_parent, 
    float g,
    float h
) 
    : pose(pose),  
      parent_idx_3D(parent_idx_3D), 
      motion_from_parent(motion_from_parent), 
      g(g),
      h(h) {}

LocalGrid::State::State(const State& s) 
    : pose(s.pose),  
      parent_idx_3D(s.parent_idx_3D), 
      motion_from_parent(s.motion_from_parent), 
      g(s.g),
      h(s.h) {}

LocalGrid::State& LocalGrid::State::operator=(State s) {
  pose = s.pose;
  motion_from_parent = s.motion_from_parent;
  std::swap(parent_idx_3D, s.parent_idx_3D);
  std::swap(g, s.g);
  std::swap(h, s.h);
  return *this;
}

LocalGrid::State::~State() {}

LocalGrid::LocalGrid()
    : x_size_(128), y_size_(128), theta_size_(72), cell_size_(0.03f), min_obstacle_dist_(0.25f) {
  local_grid_.reserve(x_size_ * y_size_);
  obstacle_heuristic_.reserve(x_size_ * y_size_);
  local_grid_to_base_link_ = Eigen::Matrix4f::Identity();
  map_to_base_link_ = Eigen::Matrix4f::Identity();
}

void LocalGrid::SetLocalGrid(const std::vector<float>& local_grid, const Eigen::Matrix4f local_grid_to_base_link, const Eigen::Matrix4f map_to_base_link) {
  local_grid_ = local_grid;
  local_grid_to_base_link_ = local_grid_to_base_link;
  map_to_base_link_ = map_to_base_link;
}

Motion LocalGrid::GetMotion(const Eigen::Matrix4f map_to_goal) {

  Pose pose_start = GetPoseFromMap(map_to_base_link_);
  Pose pose_goal = GetPoseFromMap(map_to_goal);

  PrecomputeHeuristics(pose_goal);

  float start_heuristic = GetHeuristic(pose_start);
  if (start_heuristic < 0.0f) {
    return Motion();
  }

  State start(pose_start, -1, Motion(), 0.0f, start_heuristic);

  HybridPriorityQueue open_set;
  open_set.push(start);

  std::unordered_set<int> close_set;

  while (!open_set.empty()) {

    State current = open_set.top();
    open_set.pop();

  }

  return Motion();
}

LocalGrid::Pose LocalGrid::GetPoseFromMap(Eigen::Matrix4f map_to_p) {
  Eigen::Matrix4f local_grid_to_p = (local_grid_to_base_link_ * aut_utils::InverseTransformation(map_to_base_link_)) * map_to_p;
  Eigen::Matrix4f p_to_x = Eigen::Matrix4f::Identity();
  p_to_x(0, 3) = 1.0f;
  Eigen::Vector2f vec_x = ((local_grid_to_p * p_to_x).block<2, 1>(0, 3) - local_grid_to_p.block<2, 1>(0, 3)).normalized();
  return Pose(local_grid_to_p(0, 3), local_grid_to_p(1, 3), std::atan2(vec_x(1), vec_x(0)));
}

void LocalGrid::PrecomputeHeuristics(const Pose goal) {
  PrecomputeObstacleHeuristic(goal);
  // PrecomputeMovementHeuristic();
}

void LocalGrid::PrecomputeObstacleHeuristic(const Pose goal) {
  obstacle_heuristic_ = std::vector<float>(x_size_ * y_size_, -1.0f);
  
  int goal_idx_2D = GetIndex2D(goal);
  std::vector<std::pair<int, float>> open_set;
  open_set.push_back(std::pair<int, float>(goal_idx_2D, 0.0f));

  while (!open_set.empty()) {

    std::vector<std::pair<int, float>>::iterator current_it = open_set.begin();

    for (auto it = open_set.begin(); it != open_set.end(); it++) {
      if ((*it).second < (*current_it).second) {
        current_it = it;
      }
    }

    open_set.erase(current_it);
    std::pair<int, float> current = *current_it;

    if (obstacle_heuristic_[current.first] < 0.0f || current.second < obstacle_heuristic_[current.first]) {
      obstacle_heuristic_[current.first] = current.second;
    } else {
      continue;
    }

    std::vector<std::pair<int, float>> neighbors = GetNeighbor2D(current);

    for (std::pair<int, float>& neighbor : neighbors) {
      if (obstacle_heuristic_[neighbor.first] < 0.0f || neighbor.second < obstacle_heuristic_[neighbor.first]) {
        open_set.push_back(neighbor);
      }
    }
  }
}

int LocalGrid::GetIndex2D(const Pose pose) {
  int idx_x = static_cast<int>(pose.x / cell_size_);
  int idx_y = static_cast<int>(pose.y / cell_size_);
  if (idx_x < 0 || idx_x > x_size_ || idx_y < 0 || idx_y > y_size_) {
    return -1;
  }
  return idx_x + x_size_ * idx_y;
}

std::vector<std::pair<int, float>> LocalGrid::GetNeighbor2D(const std::pair<int, float> current) {
  int idx_x = current.first % x_size_;
  int idx_y = current.first / x_size_;
  std::vector<std::pair<int, float>> r;
  if (idx_x < 0 || idx_x > x_size_ || idx_y < 0 || idx_y > y_size_) {
    return r;
  }
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      if (i == 0 && j == 0) {
        continue;
      }
      if (idx_x + i < 0 || idx_x + i > x_size_ || idx_y + j < 0 || idx_y + j > y_size_) {
        continue;
      }
      int neighbor_idx = (idx_x + i) + x_size_ * (idx_y + j);
      if (local_grid_[neighbor_idx] <= min_obstacle_dist_) {
        continue;
      }
      r.push_back(std::pair<int, float>(neighbor_idx, current.second + cell_size_ * std::sqrt(i * i + j * j)));
    }
  }
  return r;
}

float LocalGrid::GetHeuristic(const Pose pose) {
  // max(obstacle, movement);
  return obstacle_heuristic_[GetIndex2D(pose)];
}

int LocalGrid::Get3DIndex(Pose pose) {
  int idx_x = static_cast<int>(pose.x / cell_size_);
  int idx_y = static_cast<int>(pose.y / cell_size_);
  int idx_theta = static_cast<int>(pose.theta / (2 * M_PI / theta_size_));
  if (idx_x < 0 || idx_x > x_size_ || idx_y < 0 || idx_y > y_size_ || idx_theta < 0 || idx_theta > theta_size_) {
    return -1;
  }
  return idx_x + x_size_ * idx_y + x_size_ * y_size_ * idx_theta;
} 

}  // namespace aut_local_planner