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
#include <numeric>      // std::iota
#include <algorithm>    // std::sort, std::stable_sort

using namespace termcolor;
using namespace Eigen;
typedef neptimers::Timer MyTimer;

ros::Publisher positions_pub_;

Vector2d grid_pos_origin_(0.0, 5.0);
Vector2d grid_size_(1.0, 1.0);
double max_vel_along_grid_ = 1.0; //this refer to the actual geometric unit, not grid unit
double max_acc_along_grid_ = 1.0;
std::unique_ptr<perm_grid_search> perm_search_;
std::vector<Eigen::Matrix<int, 2, Dynamic>> pos_path_;
int number_of_agents_ = 8;
Eigen::Matrix<double, Dynamic, 3> agent_pos_, agent_pos_prev_;
MyTimer setpoint_timer_;
bool publishing_setpoint_ = false;
std::vector<MyTimer> timer_getting_odoms_;
ros::Timer SetpointpubCBTimer_;
bool gotten_all_odoms_ = false;
double tolerance_init_distance_ = 0.5;
Vector2d proj_vector_0_;
Vector2d proj_vector_90_;
Eigen::Matrix<int, 2, Dynamic> agent_perm_;

void SetpointpubCB(const ros::TimerEvent& e);
void odomCB(const nav_msgs::Odometry msg, int id);
void getProjectionPlane();

std_msgs::ColorRGBA getColorJetInt(int id, int min, int max);

class Listener
{
private:
    int agent_id;
public:
  void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
  void setId(int id)
  {
  	agent_id = id;
  }

};

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