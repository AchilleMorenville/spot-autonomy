#ifndef AUT_LOCAL_PLANNER_A_STAR_H_
#define AUT_LOCAL_PLANNER_A_STAR_H_

#include <vector>
#include <utility>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace aut_local_planner {

class AStar {
 public:

  AStar(int x_dim, int y_dim, float cell_size, float min_obstacle_dist, 
        float max_obstacle_dist, float obstacle_multiplier_cost);
  bool SetGrid(const std::vector<float>& grid, 
               const Eigen::Matrix4f& grid_to_base_link, 
               const Eigen::Matrix4f& map_to_base_link);
  bool SetTarget(const Eigen::Matrix4f& map_to_target);
  bool GetPath(std::vector<Eigen::Vector2f>& path);
  bool FindPath(std::vector<std::pair<int, int>>& path);
  bool InsideGrid(const Eigen::Matrix4f& map_to_target);

 private:

  inline std::pair<int, int> GetCoordinates(const Eigen::Matrix4f& grid_to_pose);
  inline std::pair<int, int> GetCoordinates(int idx);
  inline int GetIndex(const std::pair<int, int>& p);

  inline Eigen::Vector2f GetPosition(std::pair<int, int>& p);

  inline bool Accessible(const std::pair<int, int>& p);
  inline bool InsideGrid(const std::pair<int, int>& p);

  // bool FindPath(std::vector<std::pair<int, int>>& path);

  inline float GetHeuristic(const std::pair<int, int>& p);
  inline float EuclideanDist(const std::pair<int, int>& p1, 
                             const std::pair<int, int>& p2);

  void GetNeighbors(const std::pair<int, int>& current_coord,
                    std::vector<std::pair<int, int>>& neighbors);

  inline float GetCost(int idx);
  
  // bool InsideGrid(const std::pair<int, int>& p);
  // bool IsFree(const std::pair<int, int>& p);
  // int GetIndex1D(const std::pair<int, int>& p);
  // std::pair<int, int> GetIndex2D(const Eigen::Matrix4f& grid_to_pose);
  // std::pair<int, int> GetIndex2D(int idx);
  // float GetHeuristic(int idx);
  // float GetHeuristic(const std::pair<int, int>& p);
  // float GetEuclideanDist(const std::pair<int, int>& p1, 
  //                             const std::pair<int, int>& p2);
  // float GetCost(const std::pair<int, int>& p1, 
  //                    const std::pair<int, int>& p2);

  // Parameters
  int x_dim_;
  int y_dim_;
  float cell_size_;
  float min_obstacle_dist_;
  float max_obstacle_dist_;
  float obstacle_multiplier_cost_;

  // State

  std::vector<float> obstacle_dist_grid_;
  Eigen::Matrix4f grid_to_base_link_;
  Eigen::Matrix4f grid_to_map_;
  std::pair<int, int> base_link_coord_;

  Eigen::Affine2f af_base_link_to_grid_;

  // Target
  std::pair<int, int> target_coord_;
}; 

}

#endif  // AUT_LOCAL_PLANNER_A_STAR_H_