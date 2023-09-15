#ifndef AUT_COMMON_GRAPH_H_
#define AUT_COMMON_GRAPH_H_

#include <unordered_map>
#include <vector>
#include <string>

#include <Eigen/Core>

#include <visualization_msgs/msg/marker_array.hpp>

namespace aut_common {

class Graph {

 public:
  explicit Graph();

  void AddVertex(Eigen::Matrix4f pose);
  bool AddEdge(int id_a, int id_b);
  void UpdatePoses(std::vector<Eigen::Matrix4f>& updated_poses);
  void TidyGraph();
  void Simplify(float resolution);
  
  float GetEuclideanDist(int id1, int id2);
  bool AStar(int start, int goal, std::vector<int> &path);
  int ClosestNode(Eigen::Matrix4f pose);
  int ClosestNode(Eigen::Vector3f position);
  std::vector<int> GetEdges(int id);
  Eigen::Matrix4f GetPose(int id);
  
  void SaveFile(std::string file_path);
  void LoadFile(std::string file_path);

  bool RemoveEdge(int id1, int id2);

  visualization_msgs::msg::MarkerArray GetMarkerArray();
  visualization_msgs::msg::MarkerArray GetMarkerArrayWithPath(std::vector<int> &path);

  void PrintGraph();

  void Reset();

  bool Empty();

 private:

  int n_;
  std::unordered_map<int, std::vector<int>> graph_;
  std::vector<Eigen::Matrix4f> poses_;

  void MergeVertices(int source_idx, int target_idx);

};

}  // namespace aut_common

#endif  // AUT_COMMON_GRAPH_H_