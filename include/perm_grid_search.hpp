/* ----------------------------------------------------------------------------
 * Nanyang Technological University
 * Authors: Cao Muqing, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once
#ifndef PERM_GRID_SEARCH_HPP
#define PERM_GRID_SEARCH_HPP

#include <vector>
#include <Eigen/Dense>

#include <unordered_map>
#include <queue>
#include <tuple>
#include <vector3d.hpp>

typedef struct Node Node;  // needed to be able to have a pointer inside the struct

struct Node
{
  Node* previous = NULL;
  int state = 0;
  double g = 0;
  double h = 0;
  int index = 1;  
  int next_agent = 1;
  Eigen::Matrix<int, 2, Eigen::Dynamic> perm; 
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> interaction;
  Eigen::Matrix<int, 2, Eigen::Dynamic> agent_positions;
  vector4d<std::vector<Eigen::Vector2i>> interact_3d;
  // e.g., in interact_3d, \sigma_1^1 is (1,1), \sigma_2^-1 is (2,-1)
};

typedef Node* NodePtr;

// Taken from https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
template <typename T>
struct matrix_hash_ : std::unary_function<T, size_t>
{
  std::size_t operator()(T const& matrix) const
  {
    // Note that it is obvious to the storage order of Eigen matrix (column- or
    // row-major). It will give you the same hash value for two different matrices if they
    // are the transpose of each other in different storage order.
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash_<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector2i, NodePtr, matrix_hash_<Eigen::Vector2i>>
      data_xi_;
  std::unordered_map<Eigen::MatrixXi, NodePtr, matrix_hash_<Eigen::MatrixXi>>
    data_xxi_;
 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}

  // void insert(Eigen::Vector2i idx, int time_idx, NodePtr node) {
  //   data_3d_.insert(std::make_pair(
  //       Eigen::Vector3i(idx(0), idx(1), time_idx), node));
  // }

  // void insert(int idx, NodePtr node) {

  //   data_xi_.insert(std::make_pair(Eigen::Vector2i(idx, 0), node));
  // }

  void insert(Eigen::MatrixXi idx, NodePtr node) {

    data_xxi_.insert(std::make_pair(idx, node));
  }

  // NodePtr find(Eigen::Vector2i idx, int time_idx) {
  //   auto iter =
  //       data_3d_.find(Eigen::Vector3i(idx(0), idx(1), time_idx));
  //   return iter == data_3d_.end() ? NULL : iter->second;
  // }

  NodePtr find(Eigen::MatrixXi idx) {
    auto iter = data_xxi_.find(idx);
    return iter == data_xxi_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_xi_.clear();
    data_xxi_.clear();
  }
};

class perm_grid_search
{
public:
  perm_grid_search(int num_agent);  
  ~perm_grid_search();

  void setUp();


  void setGoal(Eigen::Matrix<int, 2, Eigen::Dynamic> goalPerm);

  void setRunTime(double max_runtime);

  void setBias(double bias);

  bool run(Eigen::Matrix<int, 2, Eigen::Dynamic> start_perm,  
           Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& interaction, 
           vector4d<std::vector<Eigen::Vector2i>>& interact_3d, int& status, bool is_second_try=false);

  bool runonce(std::vector<Eigen::Vector3d>& tmp_path, int &status);

  void recoverPath(NodePtr node1_ptr);


  struct CompareCost
  {
    double bias;
    bool operator()(const NodePtr left, const NodePtr right)
    {
      double cost_left = left->g + bias * left->h;
      double cost_right = right->g + bias * right->h;
      if (fabs(cost_left - cost_right) < 1e-5)
      {
        return left->h > right->h;  // If two costs are ~the same, decide only upon heuristic
      }
      else
      {
        return cost_left > cost_right;
      }
    }
  };

  // bool collidesWithObstacles(NodePtr current);
  bool entanglesWithOtherAgents(NodePtr current, double& arc_length);
  void clearProcess();

  double getCost();
  void generatePwpOut();
  void getPermPath(std::vector<Eigen::Matrix<int, 2, Eigen::Dynamic>>& result);
  void getPosPath(std::vector<Eigen::Matrix<int, 2, Eigen::Dynamic>>& result);
  void getRuntime(double& runtime_this_round, double& time_spent_contact_pt, int& node_used_num);
  bool check3robotEnt(std::vector<Eigen::Vector2i>& v, Eigen::Vector2i to_add);
protected:
private:


  void expandAndAddToQueue(NodePtr current);
  void expandAndAddToQueue2(NodePtr current);

  double h(Eigen::Vector2d pos);

  void getCurrentSamplePath(const NodePtr current, std::vector<Eigen::Vector3d>& output);
  void recoverPwpOut(NodePtr result_ptr);

  unsigned int power_int(unsigned int base, unsigned int exponent);
  double getH(NodePtr node);
  double getG(NodePtr node);
  int getIdxNode(NodePtr node);
  void fromPermToPositions(Eigen::Matrix<int, 2, Eigen::Dynamic>& perm,
                            Eigen::Matrix<int, 2, Eigen::Dynamic>& positions);
  void encode3dintoInteract(Eigen::MatrixXi& interaction, 
  vector4d<std::vector<Eigen::Vector2i>>& interact_3d);
  int encode3dSepcific(std::vector<Eigen::Vector2i>& braid);

  std::priority_queue<NodePtr, std::vector<NodePtr>, CompareCost> openList_;  //= OpenSet, = Q
  NodePtr best_node_ptr_ = NULL;
  double bias_ = 1.5;  // page 34 of https://www.cs.cmu.edu/~motionplanning/lecture/Asearch_v8.pdf

  int number_of_agents_ = 1;
  double max_runtime_ = 100.5;  //[s]
  int node_used_num_ = 0;
  int node_num_max_ = 70000;
  NodeHashTable expanded_nodes_;
  NodeHashTable generated_nodes_;
  NodePtr initialnode_;
  double runtime_this_round_;
  Eigen::Matrix<int, 2, Eigen::Dynamic> goalPerm_;
  Eigen::Matrix<int, 2, Eigen::Dynamic> goalPos_;
  std::vector<Eigen::Matrix<int, 2, Eigen::Dynamic>> pastGoalPos_;

  std::vector<NodePtr> node_pool_;
  std::vector<Eigen::Matrix<int, 2, Eigen::Dynamic>> perm_path_;
  std::vector<Eigen::Matrix<int, 2, Eigen::Dynamic>> pos_path_;
  std::vector<int> list_of_agents_;
};

#endif