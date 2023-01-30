#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <math.h>
#include "perm_grid_search.hpp"
#include <Eigen/Dense>
#include "timer.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "termcolor.hpp"
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <vector3d.hpp>
#include "entangle_check.hpp"

using namespace termcolor;
using namespace Eigen;
typedef neptimers::Timer MyTimer;

ros::Publisher positions_pub_;
std::vector<ros::Publisher> cmd_pub_vec_;

enum PlannerStatus
{
  IDLE = 0,
  MOVING_START = 1,
  FOLLOWING_PLAN = 2,
  MOVING_END = 3
};

int number_of_agents_ = 7;
Vector2d grid_pos_origin_(-6.0, -6.0);
Vector2d grid_size_(2.0, 2.0);
bool benchmark_mode_ = true;
double max_vel_along_grid_ = 0.7; //this refer to the actual geometric unit, not grid unit
double max_acc_along_grid_ = 1.0;
std::unique_ptr<perm_grid_search> perm_search_;
std::vector<Eigen::Matrix<int, 2, Dynamic>> pos_path_;
Eigen::Matrix<double, Dynamic, 3> agent_pos_, agent_pos_prev_, agent_target_, agent_start_;
Eigen::Matrix<double, Dynamic, 3> agent_start_on_grid_, agent_end_grid_;

Eigen::Matrix<double, 2, Dynamic> agent_pos_proj_, agent_pos_proj_prev_;
MatrixXi agent_interaction_;

MyTimer setpoint_timer_, update_timer_;
std::vector<MyTimer> timer_getting_odoms_;

bool found_projection_ = false;
bool publishing_setpoint_ = false;
ros::Timer SetpointpubCBTimer_;
bool gotten_all_odoms_ = false;
double tolerance_init_distance_ = 0.5;
Vector2d proj_vector_0_;
Vector2d proj_vector_90_;
Eigen::Matrix<int, 2, Dynamic> agent_perm_;
vector4d<std::vector<Eigen::Vector2i>> agent_interact_3d_;
int planner_status_ = PlannerStatus::IDLE;
ros::Publisher pub_log_;

void SetpointpubCB(const ros::TimerEvent& e);
void odomCB(const nav_msgs::Odometry msg, int id);
void getProjectionPlane();
void getAgentPerm(Matrix<double, 2, Dynamic>& agent_pos_projected, 
                        Matrix<int, 2, Dynamic>& agent_perm);
void getAgentPosFromPerm(Matrix<double, 2, Dynamic>& agent_pos_projected, 
                        Matrix<int, 2, Dynamic>& agent_perm);
void GoalCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
void publishTrajCmd(MatrixXd& agents_cmd_pva);
void reactiveController(bool& controller_enabled, Eigen::Vector2d& tarPos, int& itr, int agent_id);

std_msgs::ColorRGBA getColorJetInt(int id, int min, int max);

class Listener
{
private:
    int agent_id;
public:
  Listener(int id)
  {
    agent_id = id;
  }
  void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
  void setId(int id)
  {
  	agent_id = id;
  }
  int getId()
  {
    return agent_id;
  }
};
std::vector<Listener> idListener_;

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) {

  // initialize original index locations
  std::vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values 
  stable_sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}
