#include "planning_ros.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_ros");

  ros::NodeHandle nh1("~");
  ros::NodeHandle nh2("~");
  ros::NodeHandle nh3("~");

  // Concurrency and parallelism in ROS:
  // https://nicolovaligi.com/concurrency-and-parallelism-in-ros1-and-ros2-application-apis.html

  ros::CallbackQueue custom_queue1;
  ros::CallbackQueue custom_queue2;
  ros::CallbackQueue custom_queue3;

  nh1.setCallbackQueue(&custom_queue1);
  nh2.setCallbackQueue(&custom_queue2);
  nh3.setCallbackQueue(&custom_queue3);

  perm_search_ = std::unique_ptr<perm_grid_search>(new perm_grid_search(number_of_agents_));

  agent_pos_ = MatrixXd::Zero(number_of_agents_, 3);
  agent_pos_prev_ = MatrixXd::Zero(number_of_agents_, 3);

  std::vector<ros::Subscriber> sub_odom_vector_;
  for (int i = 0; i < number_of_agents_; ++i)
  {
    agent_pos_(i,0) = -999.0;
    agent_pos_prev_(i,0) = -999.0;

    Listener idListener;
    idListener.setId(i);
    ros::Subscriber sub_odom_ = nh1.subscribe<nav_msgs::Odometry>
                                    ("/firefly"+std::to_string(i)+"/ground_truth/odometry", 
                                      1, &Listener::odomCB, &idListener);
    sub_odom_vector_.push_back(sub_odom_);

    MyTimer timerr(true);
    timer_getting_odoms_.push_back(timerr);
  }

  ros::Timer SetpointpubCBTimer = nh2.createTimer(ros::Duration(0.1), SetpointpubCB);  

  ros::AsyncSpinner spinner1(1, &custom_queue1);  // 1 thread for the custom_queue1 // 0 means threads= # of CPU cores
  ros::AsyncSpinner spinner2(1, &custom_queue2);  // 1 thread for the custom_queue2 // 0 means threads= # of CPU cores
  ros::AsyncSpinner spinner3(1, &custom_queue3);  // 1 thread for the custom_queue3 // 0 means threads= # of CPU cores

  spinner1.start();  // start spinner of the custom queue 1
  spinner2.start();  // start spinner of the custom queue 2
  spinner3.start();  // start spinner of the custom queue 3

  ros::waitForShutdown();
  return 0;
}

void Listener::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (timer_getting_odoms_[agent_id].ElapsedMs() < 100.0)
    return;
  agent_pos_prev_.row(agent_id) = agent_pos_.row(agent_id);
  agent_pos_(agent_id,0) = msg->pose.pose.position.x;
  agent_pos_(agent_id,1) = msg->pose.pose.position.y;
  agent_pos_(agent_id,2) = msg->pose.pose.position.z;


  timer_getting_odoms_[agent_id].Reset();

}

void SetpointpubCB(const ros::TimerEvent& e)
{
  if (!gotten_all_odoms_)
  {
    gotten_all_odoms_ = true;
    for (int i = 0; i < number_of_agents_; ++i)
    {
      if (agent_pos_(i,0)<-500.0 && agent_pos_prev_(i,0)<-500.0)
      {
        gotten_all_odoms_ = false;
      }
      break;
    }
    if (gotten_all_odoms_)
    {
      getProjectionPlane();
    }
  }

  
}

void getProjectionPlane()
{
  bool setting_correct = false;  
  double theta = 0;
  MatrixXd agent_pos_projected = MatrixXd::Zero(number_of_agents_, 2);

  while (!setting_correct)
  {
    proj_vector_0_ << cos(theta), sin(theta);
    proj_vector_90_ << cos(theta+1.570796), sin(theta+1.570796);
    for (int i = 0; i < number_of_agents_; ++i)
    {
      Vector2d xy(agent_pos_(i,0), agent_pos_(i,1));
      agent_pos_projected(i, 0) = xy.dot(proj_vector_0_);
      agent_pos_projected(i, 1) = xy.dot(proj_vector_90_);
    }

    for (int i = 0; i < number_of_agents_; ++i)
    {
      for (int j = i+1; j < number_of_agents_; ++j)
      {
        if (fabs(agent_pos_projected(i, 0)-agent_pos_projected(j, 0))<tolerance_init_distance_ &&
            fabs(agent_pos_projected(i, 1)-agent_pos_projected(j, 1))<tolerance_init_distance_)
        {
          goto nextround;
        }
      }
    }

    setting_correct = true;
    break;    
    nextround:
    {
      theta = theta + 5.0/180.0 * 3.1415927;
    }
  }

  std::vector<double> projected_x;
  std::vector<double> projected_y;
  for (int i = 0; i < number_of_agents_; ++i)
  {
    projected_x.push_back(agent_pos_projected(i,0));
    projected_y.push_back(agent_pos_projected(i,1));
  }
  std::vector<int>  perm_x = sort_indexes(projected_x);

}