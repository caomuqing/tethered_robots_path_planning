#include <ros/ros.h>
#include <math.h>
#include "perm_grid_search.hpp"
#include <Eigen/Dense>
#include "timer.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "termcolor.hpp"

using namespace termcolor;
using namespace Eigen;
typedef neptimers::Timer MyTimer;
ros::Timer SetpointpubCBTimer_;
ros::Publisher positions_pub_;

Vector2d grid_pos_origin_(0.0, 0.0);
Vector2d grid_size_(1.0, 1.0);
double max_vel_along_grid_ = 1.0; //this refer to the actual geometric unit, not grid unit
double max_acc_along_grid_ = 1.0;
std::unique_ptr<perm_grid_search> perm_search_;
std::vector<Eigen::Matrix<int, 2, Dynamic>> pos_path_;
int number_of_agents_ = 8;
Eigen::Matrix<double, Dynamic, 3> agent_pos_;
MyTimer setpoint_timer_;
bool publishing_setpoint_ = false;

void SetpointpubCB(const ros::TimerEvent& e);
std_msgs::ColorRGBA getColorJetInt(int id, int min, int max);

int main(int argc, char *argv[]) {
	// initialize ROS
	ros::init(argc, argv, "perm_grid_ros");
	ros::NodeHandle nh("~");

	perm_search_ = std::unique_ptr<perm_grid_search>(new perm_grid_search(number_of_agents_));
	Eigen::MatrixXi start_perm(2,number_of_agents_);
	start_perm<< 0, 1, 2, 3, 4, 5, 6, 7,
				 0, 1, 2, 3, 4, 5, 6, 7;
	Eigen::MatrixXi goal_perm(2,number_of_agents_);
	goal_perm<< 7, 6, 5, 4, 3, 2, 1, 0,
				 7, 6, 5, 4, 3, 2, 1, 0;	

	agent_pos_ = MatrixXd::Zero(number_of_agents_, 3);
	for (int i = 0; i < number_of_agents_; ++i)
	{
	 	agent_pos_(i,0) = grid_pos_origin_(0)+start_perm(i)*grid_size_(0);
	 	agent_pos_(i,1) = grid_pos_origin_(1)+start_perm(i)*grid_size_(1);
	}			

	Eigen::MatrixXi start_interaction = Eigen::MatrixXi::Zero(number_of_agents_, number_of_agents_);
	perm_search_ ->setGoal(goal_perm);
	int search_status = 0;
	perm_search_ ->run(start_perm, start_interaction, search_status);
	
	if (search_status ==1)
	{
	  pos_path_.clear();
	  perm_search_->getPosPath(pos_path_);
	  setpoint_timer_.Reset();
	  publishing_setpoint_ = true;
	}

  	positions_pub_ = nh.advertise<visualization_msgs::Marker>("agent_positions", 1);
  	SetpointpubCBTimer_ = nh.createTimer(ros::Duration(0.1), SetpointpubCB);  

  	ros::spin();
	ros::waitForShutdown();
 
		// ros::spinOnce();
	// }
	return 0;
}

void SetpointpubCB(const ros::TimerEvent& e)
{
	if (!publishing_setpoint_) return;
	double time_elapsed = setpoint_timer_.ElapsedMs()/1000.0;
	// std::cout<<green<<"time elapsed is "<<time_elapsed<<" s"<<std::endl;

	for (int i = 0; i < pos_path_.size()-1; ++i)
	{
		double time_for_this_seg = ((pos_path_[i+1]-pos_path_[i]).cast<double>().
									transpose()*grid_size_).norm()/max_vel_along_grid_;
		if (time_elapsed>time_for_this_seg)
		{
			time_elapsed -= time_for_this_seg;
			continue;
		}

		MatrixXd grid_pos = pos_path_[i].cast<double>() + 
						(pos_path_[i+1]-pos_path_[i]).cast<double>()*time_elapsed/time_for_this_seg;
		agent_pos_.col(0) = Eigen::MatrixXd::Ones(number_of_agents_,1) * grid_pos_origin_(0)+
							grid_pos.row(0).transpose()*grid_size_(0);
		agent_pos_.col(1) = Eigen::MatrixXd::Ones(number_of_agents_,1) * grid_pos_origin_(1)+
							grid_pos.row(1).transpose()*grid_size_(1);							
		std::cout<<green<<"current path segment is "<<i<<std::endl;
		std::cout<<green<<"agent pos path  is "<<std::endl;
		std::cout<<green<<agent_pos_<<std::endl;
		break;
	}


	visualization_msgs::Marker m;
	m.type = visualization_msgs::Marker::SPHERE_LIST;
	m.action = visualization_msgs::Marker::ADD;
	m.id = 1;  // % 3000;  // Start the id again after ___ points published (if not RVIZ goes very slow)
	m.ns = "Agents";

	m.scale.x = 1.0;
	m.scale.y = 1.0;
	m.scale.z = 1.0;
	m.header.stamp = ros::Time::now();
	m.header.frame_id = "world";
	// pose is actually not used in the marker, but if not RVIZ complains about the quaternion
	m.pose.position.x = 0.0;
	m.pose.position.y = 0.0;
	m.pose.position.z = 0.0;
	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;
	for (int i = 0; i < number_of_agents_; ++i)
	{
		geometry_msgs::Point p;
		p.x = agent_pos_(i, 0);
		p.y = agent_pos_(i, 1);
		p.z = agent_pos_(i, 2);
		m.points.push_back(p);
		m.colors.push_back(getColorJetInt(i, 0, number_of_agents_));
	}

	positions_pub_.publish(m);

}


std_msgs::ColorRGBA getColorJetInt(int id, int min, int max)
{
  std_msgs::ColorRGBA c;
  c.r = 1;
  c.g = 1;
  c.b = 1;
  c.a = 1;
  // white
  double dv;

  if (id < min)
    id = min;
  if (id > max)
    id = max;
  if (id-1 <= (double)min + (double)(max-min)/8*1-0.001)
  {
    c.r = (double)70/255;
    c.g = (double)240/255;
    c.b = (double)240/255;    

  }
  else if (id-1 <= (double)min + (double)(max-min)/8*2-0.001)
  {
    c.r = (double)240/255;
    c.g = (double)50/255;
    c.b = (double)230/255;
  }
  else if (id-1 <= (double)min + (double)(max-min)/8*3-0.001)
  {
    c.r = (double)60/255;
    c.g = (double)180/255;
    c.b = (double)75/255;   

  }
  else if (id-1 <= (double)min + (double)(max-min)/8*4-0.001)
  {
    c.r = (double)170/255;
    c.g = (double)110/255;
    c.b = (double)40/255;     
  }
  else if (id-1 <= (double)min + (double)(max-min)/8*5-0.001)
  {
    c.r = (double)255/255;
    c.g = (double)225/255;
    c.b = (double)25/255;     
  }
  else if (id-1 <= (double)min + (double)(max-min)/8*6-0.001)
  {
    c.r = (double)0/255;
    c.g = (double)130/255;
    c.b = (double)200/255;   

  }
  else if (id-1 <= (double)min + (double)(max-min)/8*7-0.001)
  {
    c.r = (double)230/255;
    c.g = (double)25/255;
    c.b = (double)75/255;   
  } 
  else if (id-1 <= (double)min + (double)(max-min)+0.001)
  {
    c.r = (double)255/255;
    c.g = (double)0/255;
    c.b = (double)0/255;        

  }

  return (c);
}