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
  agent_perm_ = MatrixXi::Zero(2, number_of_agents_);
  agent_interaction_ = Eigen::MatrixXi::Zero(number_of_agents_, number_of_agents_);

  std::vector<ros::Subscriber> sub_odom_vector_;
  for (int i = 0; i < number_of_agents_; ++i)
  {
    agent_pos_(i,0) = -999.0;
    agent_pos_prev_(i,0) = -999.0;

    Listener idListener;
    idListener.setId(i);
    ros::Subscriber sub_odom_ = nh1.subscribe<nav_msgs::Odometry>
                                    ("/firefly"+std::to_string(i+1)+"/ground_truth/odometry", 
                                      1, &Listener::odomCB, &idListener);
    ros::Publisher cmd_pub = nh1.advertise<trajectory_msgs::MultiDOFJointTrajectory>
                              ("/firefly"+std::to_string(i+1)+"/command/trajectory", 1);
    sub_odom_vector_.push_back(sub_odom_);
    cmd_pub_vec_.push_back(cmd_pub);

    MyTimer timerr(true);
    timer_getting_odoms_.push_back(timerr);
  }

  ros::Timer SetpointpubCBTimer = nh2.createTimer(ros::Duration(0.1), SetpointpubCB);  
  ros::Subscriber goal_sub = nh3.subscribe<std_msgs::Float32MultiArray>("goal", 
                              1, GoalCallback);

  ros::AsyncSpinner spinner1(1, &custom_queue1);  // 1 thread for the custom_queue1 // 0 means threads= # of CPU cores
  ros::AsyncSpinner spinner2(1, &custom_queue2);  // 1 thread for the custom_queue2 // 0 means threads= # of CPU cores
  ros::AsyncSpinner spinner3(1, &custom_queue3);  // 1 thread for the custom_queue3 // 0 means threads= # of CPU cores

  spinner1.start();  // start spinner of the custom queue 1
  spinner2.start();  // start spinner of the custom queue 2
  spinner3.start();  // start spinner of the custom queue 3

  ros::waitForShutdown();
  return 0;
}

void GoalCallback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
  if (msg->data.size()%3 !=0) ROS_WARN("Size of input message not correct!");
  int num_of_robot = msg->data.size()/3;
  if (num_of_robot!=number_of_agents_)
  {
     ROS_WARN("Size of input message not consistent with the number of robots!");
     return;
  }
  if (!found_projection_) 
  {
     ROS_WARN("Not getting a feasible projection, returnning!!");
     return;
  }

  Eigen::Matrix<double, 2, Dynamic> goals_proj;
  goals_proj = MatrixXd::Zero(2, number_of_agents_);

  std::vector<Eigen::Matrix<double, 3, 1>> EndPts;
  for (int i=0; i<number_of_agents_; i++)
  {
    Vector3d Endpt;
    Endpt(0) =  msg->data[i*3+0];
    Endpt(1) =  msg->data[i*3+1];
    Endpt(2) =  msg->data[i*3+2];
    EndPts.push_back(Endpt);

    goals_proj(0, i) = Endpt.head(2).dot(proj_vector_0_);
    goals_proj(1, i) = Endpt.head(2).dot(proj_vector_90_);
  }  

  Eigen::Matrix<int, 2, Dynamic> goal_perm = MatrixXi::Zero(2, number_of_agents_);  
  getAgentPerm(goals_proj, goal_perm); 

  perm_search_ ->setGoal(goal_perm);   
  int search_status = 0;
  perm_search_ ->run(agent_perm_, agent_interaction_, search_status);
  
  if (search_status ==1)
  {
    pos_path_.clear();
    perm_search_->getPosPath(pos_path_);
    setpoint_timer_.Reset();
    publishing_setpoint_ = true;
  }  
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
  if (!gotten_all_odoms_) //initialize projection plane
  {
    gotten_all_odoms_ = true;
    for (int i = 0; i < number_of_agents_; ++i)
    {
      if (agent_pos_(i,0)<-500.0 && agent_pos_prev_(i,0)<-500.0)
      {
        gotten_all_odoms_ = false;
        return;
      }
    }

    getProjectionPlane();
  }

  if (found_projection_ && update_timer_.ElapsedMs()>100.0) //update agent status
  {
    agent_pos_proj_prev_ = agent_pos_proj_;
    for (int i = 0; i < number_of_agents_; ++i)
    {
      Vector2d xy(agent_pos_(i,0), agent_pos_(i,1));
      agent_pos_proj_(0, i) = xy.dot(proj_vector_0_);
      agent_pos_proj_(1, i) = xy.dot(proj_vector_90_);
    }

    for (int dim: {0,1})
    {
      for (int i = 0; i < number_of_agents_; ++i)
      {
        for (int j = i+1; j < number_of_agents_; ++j)
        {
          if ((agent_pos_proj_(0, i)-agent_pos_proj_(0, j))*
            (agent_pos_proj_prev_(0, i)-agent_pos_proj_prev_(0, j))<0) //there is intersection
          {
            //update interaction
          }
        }    
      }
    }
    getAgentPerm(agent_pos_proj_, agent_perm_);
    update_timer_.Reset();
  }

  MatrixXd agents_cmd_pva = MatrixXd::Zero(number_of_agents_, 9);
  MatrixXd projection_matrix;
  projection_matrix << proj_vector_0_, proj_vector_90_;

  if (!publishing_setpoint_) //publish initial positions
  {
    Eigen::Matrix<double, 2, Dynamic> agents_pos_projected = MatrixXd::Zero(2, number_of_agents_);
    getAgentPosFromPerm(agents_pos_projected, agent_perm_);
    agents_pos_projected.row(0) = agents_pos_projected.row(0) * grid_size_(0);
    agents_pos_projected.row(1) = agents_pos_projected.row(1) * grid_size_(1);    
    Eigen::Matrix<double, 2, Dynamic> agents_pos = projection_matrix * agents_pos_projected;

    for (int i = 0; i < number_of_agents_; ++i)
    {
      agents_cmd_pva(i,0) = grid_pos_origin_(0)+agents_pos(0,i);
      agents_cmd_pva(i,1) = grid_pos_origin_(1)+agents_pos(1,i);
      agents_cmd_pva(i,2) = 2.0;
    }
  }
  else
  {
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
      grid_pos.row(0) = grid_pos.row(0) * grid_size_(0);
      grid_pos.row(1) = grid_pos.row(1) * grid_size_(1);   
      Eigen::Matrix<double, 2, Dynamic> agents_pos = projection_matrix * grid_pos;
         
      for (int i = 0; i < number_of_agents_; ++i)
      {
        agents_cmd_pva(i,0) = grid_pos_origin_(0)+agents_pos(0,i);
        agents_cmd_pva(i,1) = grid_pos_origin_(1)+agents_pos(1,i);
        agents_cmd_pva(i,2) = 2.0;
      }
      break;
    }

  }

  publishTrajCmd(agents_cmd_pva);
}

void getProjectionPlane()
{
  double theta = 0;
  MatrixXd agent_pos_projected = MatrixXd::Zero(2, number_of_agents_);

  while (!found_projection_)
  {
    proj_vector_0_ << cos(theta), sin(theta);
    proj_vector_90_ << cos(theta+1.570796), sin(theta+1.570796);
    for (int i = 0; i < number_of_agents_; ++i)
    {
      Vector2d xy(agent_pos_(i,0), agent_pos_(i,1));
      agent_pos_projected(0, i) = xy.dot(proj_vector_0_);
      agent_pos_projected(1, i) = xy.dot(proj_vector_90_);
    }

    for (int i = 0; i < number_of_agents_; ++i)
    {
      for (int j = i+1; j < number_of_agents_; ++j)
      {
        if (fabs(agent_pos_projected(0, i)-agent_pos_projected(0, j))<tolerance_init_distance_ &&
            fabs(agent_pos_projected(1, i)-agent_pos_projected(1, j))<tolerance_init_distance_)
        {
          goto nextround;
        }
      }
    }

    found_projection_ = true;
    ROS_WARN("Finished setting up projection plane!");
    break;    
    nextround:
    {
      theta = theta + 5.0/180.0 * 3.1415927;
    }
  }

  agent_pos_proj_ = agent_pos_projected;
  agent_pos_proj_prev_ = agent_pos_projected;

  getAgentPerm(agent_pos_proj_, agent_perm_);
  
  update_timer_.Reset();
}

void getAgentPerm(Matrix<double, 2, Dynamic>& agent_pos_projected, 
                        Matrix<int, 2, Dynamic>& agent_perm)
{
  std::vector<double> projected_x;
  std::vector<double> projected_y;
  for (int i = 0; i < number_of_agents_; ++i)
  {
    projected_x.push_back(agent_pos_projected(0, i));
    projected_y.push_back(agent_pos_projected(1, i));
  }
  auto perm_x = sort_indexes(projected_x);
  auto perm_y = sort_indexes(projected_y);

  for (int i = 0; i < number_of_agents_; ++i)
  {
    agent_perm(0,i) = perm_x[i];
    agent_perm(1,i) = perm_y[i];
  }
}

void getAgentPosFromPerm(Matrix<double, 2, Dynamic>& agent_pos_projected, 
                        Matrix<int, 2, Dynamic>& agent_perm)
{
  for (int dim: {0,1})
  {
    for (int i = 0; i < number_of_agents_; ++i)
    {
      for (int j = 0; j < number_of_agents_; ++j)
      {
        if (i==agent_perm(dim,j))
        {
          agent_pos_projected(dim, i) = (double)j;
        }
      }
    }
  }  
}

void publishTrajCmd(MatrixXd& agents_cmd_pva)
{
  for (int i = 0; i < number_of_agents_; ++i)
  {
    trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
    trajset_msg.header.stamp = ros::Time::now();
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
    geometry_msgs::Transform transform_msg;
    geometry_msgs::Twist accel_msg, vel_msg;
    transform_msg.translation.x = agents_cmd_pva(i,0);
    transform_msg.translation.y = agents_cmd_pva(i,1);
    transform_msg.translation.z = agents_cmd_pva(i,2);
    transform_msg.rotation.x = 0;
    transform_msg.rotation.y = 0;
    transform_msg.rotation.z = 0.0;
    transform_msg.rotation.w = 1.0;
    trajpt_msg.transforms.push_back(transform_msg);
    vel_msg.linear.x = agents_cmd_pva(i,3);
    vel_msg.linear.y = agents_cmd_pva(i,4);
    vel_msg.linear.z = agents_cmd_pva(i,5);
    accel_msg.linear.x = agents_cmd_pva(i,6);
    accel_msg.linear.y = agents_cmd_pva(i,7);
    accel_msg.linear.z = agents_cmd_pva(i,8);
    trajpt_msg.velocities.push_back(vel_msg);
    trajpt_msg.accelerations.push_back(accel_msg);
    trajset_msg.points.push_back(trajpt_msg);    
    cmd_pub_vec_[i].publish(trajset_msg);    
  }
}