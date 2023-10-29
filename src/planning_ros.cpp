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
  agent_target_ = MatrixXd::Zero(number_of_agents_, 3);
  agent_start_ = MatrixXd::Zero(number_of_agents_, 3);
  agent_start_on_grid_ = MatrixXd::Zero(number_of_agents_, 3);
  agent_perm_ = MatrixXi::Zero(2, number_of_agents_);
  agent_interaction_ = Eigen::MatrixXi::Zero(number_of_agents_, number_of_agents_);
  agent_interact_3d_ = vector4d<std::vector<Eigen::Vector2i>>(2, number_of_agents_, 
                                  number_of_agents_, number_of_agents_);
  std::vector<ros::Subscriber> sub_odom_vector_;
  for (int i = 0; i < number_of_agents_; ++i)
  {
    Listener idListener(i);
    idListener_.push_back(idListener);
  }

    for (int i = 0; i < number_of_agents_; ++i)
  {
    agent_pos_(i,0) = -999.0;
    agent_pos_prev_(i,0) = -999.0;

    ros::Subscriber sub_odom_ = nh1.subscribe<nav_msgs::Odometry>
                                    ("/firefly"+std::to_string(i+1)+"/unity/odom", 
                                      1, &Listener::odomCB, &idListener_[i]);
    ros::Subscriber sub_state_ = nh1.subscribe<snapstack_msgs::State>
                                    ("/firefly"+std::to_string(i+1)+"/state", 
                                      1, &Listener::stateCB, &idListener_[i]);
    ros::Publisher cmd_pub = nh1.advertise<trajectory_msgs::MultiDOFJointTrajectory>
                              ("/firefly"+std::to_string(i+1)+"/command/trajectory", 1);
    sub_odom_vector_.push_back(sub_odom_);
    sub_odom_vector_.push_back(sub_state_);
    cmd_pub_vec_.push_back(cmd_pub);

    MyTimer timerr(true);
    timer_getting_odoms_.push_back(timerr);
  }

  ros::Timer SetpointpubCBTimer = nh2.createTimer(ros::Duration(0.1), SetpointpubCB);  
  ros::Subscriber goal_sub = nh3.subscribe<std_msgs::Float32MultiArray>("goals", 
                              1, GoalCallback);
  pub_log_ = nh1.advertise<nav_msgs::Odometry>("/firefly/log_for_plot", 1);
  pub_permsequence_ = nh1.advertise<neptune2::PermSequence>("/permsequence", 1);

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
     ROS_WARN("num_of_robot is %d !", num_of_robot);
     return;
  }
  if (!found_projection_) 
  {
     ROS_WARN("Not getting a feasible projection, returnning!!");
     return;
  }

  Eigen::Matrix<double, 2, Dynamic> goals_proj;
  goals_proj = MatrixXd::Zero(2, number_of_agents_);

  for (int i=0; i<number_of_agents_; i++)
  {
    Vector3d Endpt;
    Endpt(0) =  msg->data[i*3+0];
    Endpt(1) =  msg->data[i*3+1];
    Endpt(2) =  msg->data[i*3+2];
    agent_target_.row(i) = Endpt.transpose();

    goals_proj(0, i) = Endpt.head(2).dot(proj_vector_0_);
    goals_proj(1, i) = Endpt.head(2).dot(proj_vector_90_);
    agent_start_on_grid_(i, 2) = Endpt(2);
  }  

  agent_start_ = agent_pos_;
  Eigen::Matrix<double, 2, Dynamic> agents_pos_grid = MatrixXd::Zero(2, number_of_agents_);
  getAgentPosFromPerm(agents_pos_grid, agent_perm_);
  agents_pos_grid.row(0) = agents_pos_grid.row(0) * grid_size_(0);
  agents_pos_grid.row(1) = agents_pos_grid.row(1) * grid_size_(1);  
  MatrixXd projection_matrix(2,2);
  projection_matrix << proj_vector_0_, proj_vector_90_;    
  Eigen::Matrix<double, 2, Dynamic> agents_pos = projection_matrix * agents_pos_grid;
  agent_start_on_grid_.block(0,0,number_of_agents_,2) = 
      (agents_pos.colwise()+grid_pos_origin_).transpose();

  setpoint_timer_.Reset();  
  Eigen::Matrix<int, 2, Dynamic> goal_perm = MatrixXi::Zero(2, number_of_agents_);  
  getAgentPerm(goals_proj, goal_perm); 
  final_goal_perm_ = goal_perm;
  goal_perm_ = goal_perm;

  perm_search_ ->setGoal(goal_perm);   
  int search_status = 0;
  perm_search_ ->run(agent_perm_, agent_interaction_, agent_interact_3d_, search_status);
  
  if (search_status ==1)
  {
    pos_path_.clear();
    perm_search_->getPosPath(pos_path_);
    
    neptune2::PermSequence permsequence;
    perm_search_->getPermSequence(permsequence);
    perm_search_->getPermSequenceVector(sequence_vector_);
    planner_status_ = PlannerStatus::MOVING_START;
    publishing_setpoint_ = true;
    current_leg_ = 0;
  }  
  double runtime_this_round;
  int node_used_num;
  perm_search_ ->getRuntime(runtime_this_round, node_used_num);
  nav_msgs::Odometry log_msg_;
  log_msg_.header.stamp = ros::Time::now();

  double totaldistance = 0.0;
  double totaltime = 0.0;
  if (benchmark_mode_)
  {
    MatrixXd grid_pos_end = pos_path_.back().cast<double>();
    grid_pos_end.row(0) = grid_pos_end.row(0) * grid_size_(0);
    grid_pos_end.row(1) = grid_pos_end.row(1) * grid_size_(1);   
    Eigen::Matrix<double, 2, Dynamic> agents_pos_end = projection_matrix * grid_pos_end;
       
    for (int i = 0; i < number_of_agents_; ++i)
    {
      agents_pos_end(0,i) = grid_pos_origin_(0)+agents_pos_end(0,i);
      agents_pos_end(1,i) = grid_pos_origin_(1)+agents_pos_end(1,i);
    }

    double max_dist_start = 0.0, max_dist_end = 0.0;
    for (int i = 0; i < number_of_agents_; ++i)
    {
      if (max_dist_start<(agent_start_.row(i) - agent_start_on_grid_.row(i)).norm())
      {
        max_dist_start = (agent_start_.row(i) - agent_start_on_grid_.row(i)).norm();
      }
      if (max_dist_end<(agents_pos_end.col(i)- agent_target_.block(i,0,1,2).transpose()).norm())
      {
        max_dist_end = (agents_pos_end.col(i)- agent_target_.block(i,0,1,2).transpose()).norm();
      }
      totaldistance += (agent_start_.row(i) - agent_start_on_grid_.row(i)).norm();
      totaldistance += (agents_pos_end.col(i)- agent_target_.block(i,0,1,2).transpose()).norm();
    }
    totaldistance += (double)pos_path_.size()*grid_size_(0)*2;
    ROS_WARN("max_dist_start is %f", max_dist_start);   
    ROS_WARN("max_dist_end is %f", max_dist_end);   

    ROS_WARN("agent avg distance is %f", totaldistance/number_of_agents_);    
    totaltime = (max_dist_end+max_dist_start+(double)pos_path_.size()*grid_size_(0))/max_vel_along_grid_;
    perm_search_ ->retrieveAllthese(agent_perm_, agent_interaction_, agent_interact_3d_);
    agent_pos_ = agent_target_;
  }

  // log_msg_.header.frame_id = world_name_;
  log_msg_.pose.pose.position.x = search_status ==1? 1.0:0.0;
  log_msg_.pose.pose.position.y = runtime_this_round; //prevent large number when undefined
  log_msg_.pose.pose.position.z = (float) node_used_num;    
  log_msg_.pose.pose.orientation.x = totaldistance/number_of_agents_;    
  log_msg_.pose.pose.orientation.y = totaltime;  
  log_msg_.pose.pose.orientation.z = 0.0;    
  pub_log_.publish(log_msg_);     
}

void Listener::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  // if (timer_getting_odoms_[agent_id].ElapsedMs() < 100.0)
  //   return;
  if (gotten_all_odoms_ && benchmark_mode_)
  {
    return;
  }
  agent_pos_prev_.row(getId()) = agent_pos_.row(getId());
  agent_pos_(getId(),0) = msg->pose.pose.position.x;
  agent_pos_(getId(),1) = msg->pose.pose.position.y;
  agent_pos_(getId(),2) = msg->pose.pose.position.z;

  // timer_getting_odoms_[agent_id].Reset();

}

void Listener::stateCB(const snapstack_msgs::State::ConstPtr& msg)
{

  if (gotten_all_odoms_ && benchmark_mode_)
  {
    return;
  }
  agent_pos_prev_.row(getId()) = agent_pos_.row(getId());
  agent_pos_(getId(),0) = msg->pos.x;
  agent_pos_(getId(),1) = msg->pos.y;
  agent_pos_(getId(),2) = msg->pos.z;

}

void SetpointpubCB(const ros::TimerEvent& e)
{
  if (!gotten_all_odoms_) //initialize projection plane
  {
    // if (benchmark_mode_)
    // {

    //   double one_slice = 3.1415927*2/number_of_agents_;
    //   double circle_init_radius = 10;

    //   for (int i=0; i<number_of_agents_; i++)
    //   {
    //     double theta = one_slice*i;
    //     agent_pos_(i,0) = -circle_init_radius*cosf(theta);
    //     agent_pos_(i,1) = -circle_init_radius*sinf(theta);                      
    //     agent_pos_prev_(i,0) = -circle_init_radius*cosf(theta);
    //     agent_pos_prev_(i,1) = -circle_init_radius*sinf(theta);        }
    //   /* code */
    // }

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

  if (found_projection_ && update_timer_.ElapsedMs()>100.0 && !benchmark_mode_) //update agent status
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
          if ((agent_pos_proj_(dim, i)-agent_pos_proj_(dim, j))*
            (agent_pos_proj_prev_(dim, i)-agent_pos_proj_prev_(dim, j))<0) //there is intersection
          {
            int agent_current, agent_to_exchange;
            if (agent_pos_proj_(dim, i)<agent_pos_proj_(dim, j))
            {
              agent_current = j;
              agent_to_exchange = i;
            }
            else
            {
              agent_current = i;
              agent_to_exchange = j;
            }
            int infront_or_behind = (agent_pos_proj_(1-dim, agent_current) 
                                    < agent_pos_proj_(1-dim, agent_to_exchange))?1:-1;

            if (dim==0) //first axis
            {
              if (agent_current< agent_to_exchange) //only keeping the upper diagonal 
              {
                agent_interaction_(agent_current, agent_to_exchange) += 
                  infront_or_behind;
              }
              else
              {
                agent_interaction_(agent_to_exchange, agent_current) += 
                  infront_or_behind;
              }        
            }
            else //second axis
            {
              if (agent_current< agent_to_exchange) //LOWER diagonal
              {
                agent_interaction_(agent_to_exchange, agent_current) += 
                  infront_or_behind;
              }
              else
              {
                agent_interaction_(agent_current, agent_to_exchange) += 
                  infront_or_behind;
              }          
            }

            for (int k = 0; k < number_of_agents_; ++k) //update interaction among 3 robots
            {
              if (k==agent_current ||k==agent_to_exchange) continue;
              Vector2i to_add;
              if (agent_pos_proj_(dim, k)<agent_pos_proj_(dim, agent_current))
              {
                to_add << 2, infront_or_behind;
              }
              else 
              {
                to_add << 1, infront_or_behind;
              }
              std::vector<int> agents_id {k, agent_current, agent_to_exchange};
              std::sort(agents_id.begin(), agents_id.end());
              perm_search_->check3robotEnt(agent_interact_3d_(dim,
                              agents_id[0], agents_id[1], agents_id[2]), to_add);

            }            
          }          
        }    
      }
    }
    getAgentPerm(agent_pos_proj_, agent_perm_);
    // std::cout<<"current agent positions in projected:"<<std::endl;
    // std::cout<<agent_pos_proj_<<std::endl;
    // std::cout<<"current agent perm"<<std::endl;
    // std::cout<<agent_perm_<<std::endl;
    // std::cout<<"\033[32mcurrent GOAL perm\033[0m"<<std::endl;
    // std::cout<<"\033[32m"<<goal_perm_<<"\033[0m"<<std::endl;    
    update_timer_.Reset();
  }

  MatrixXd agents_cmd_pva = MatrixXd::Zero(number_of_agents_, 9); //publish commands
  MatrixXd projection_matrix(2,2);
  projection_matrix << proj_vector_0_, proj_vector_90_;

  double time_elapsed = setpoint_timer_.ElapsedMs()/1000.0;  
  if (planner_status_ == PlannerStatus::MOVING_START)
  {
    neptune2::PermSequence permsequence = sequence_vector_[current_leg_];
    if (sequence_vector_.size()!=current_leg_+1) //is not the last sequence to be executed
    {
      neptune2::PermAction fake_action;
      fake_action.perm_id = -100;
      fake_action.axis = -100;
      fake_action.action = -100;
      permsequence.actions.push_back(fake_action);      
      for (size_t i = 0; i < 2; i++) //the current goal perm is the start perm of next leg
      {
        for (size_t j = 0; j < number_of_agents_; j++)
        {
          goal_perm_(i,j) = sequence_vector_[current_leg_+1].perm[i*number_of_agents_ + j];
        }
      }
    }
    else
    {
      goal_perm_ = final_goal_perm_;
    }

    if (agent_perm_.row(permsequence.actions[0].axis) == goal_perm_.row(permsequence.actions[0].axis))
    {
      ROS_WARN("[PLANNING ROS] Strange thing happens!!");
    }
    
    std::cout<<"current agent perm"<<std::endl;
    std::cout<<agent_perm_<<std::endl;
    std::cout<<"\033[32mcurrent GOAL perm\033[0m"<<std::endl;
    std::cout<<"\033[32m"<<goal_perm_<<"\033[0m"<<std::endl;    
    // std::getchar();
    pub_permsequence_.publish(permsequence);
    planner_status_ = PlannerStatus::FOLLOWING_PLAN;
    setpoint_timer_.Reset();
  }
  else if (planner_status_ == PlannerStatus::FOLLOWING_PLAN)
  {
    if (agent_perm_ == goal_perm_)
    {
      goal_count_ ++;
    }
    else goal_count_ = 0;

    if (goal_count_>5)
    {
      if (sequence_vector_.size()!=current_leg_+1) //it is not the end
      {
        current_leg_++;
        planner_status_ = PlannerStatus::MOVING_START;
      }
      else
      {
        planner_status_ = PlannerStatus::IDLE;
      } 
      goal_count_ = 0;
    }
  }
  else if (planner_status_ == PlannerStatus::IDLE) //publish initial position with height
  {

  }  
}

void reactiveController(bool& controller_enabled, Eigen::Vector2d& tarPos, int& itr, int agent_id)
{
  if (itr>100)
  {
    controller_enabled = false;
    return;
  }
  for (int i=0; i<number_of_agents_; i++)
  {
    if (i==agent_id) continue;
    Eigen::Vector2d agent_i_pos = agent_pos_.block(i,0,1,2).transpose();
    if (agent_i_pos(0)<-500) continue;

    if ((tarPos-agent_i_pos).norm()<0.7*grid_size_(0)-0.01)
    {
      controller_enabled = true;
      itr ++;
      // std::cout<<bold<<red<<"itr is" <<itr<<std::endl;
      // std::cout<<bold<<red<<"tarPos original is" <<tarPos<<std::endl; 
      tarPos = agent_i_pos + (tarPos-agent_i_pos) *
              1.2*grid_size_(0)/(tarPos-agent_i_pos).norm();

      // std::cout<<bold<<red<<"latestCheckingPosAgent_[i] is" <<latestCheckingPosAgent_[i]<<std::endl;  
      // std::cout<<bold<<red<<"tarPos new is" <<tarPos<<std::endl;        
      reactiveController(controller_enabled, tarPos, itr, agent_id);
      return;
    }
  }  
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
  std::cout<<red<<"projection plane 0 is "<< std::endl;
  std::cout<<red<<proj_vector_0_<< std::endl;
  std::cout<<red<<"projection plane 90 is "<< std::endl;
  std::cout<<red<<proj_vector_90_<< std::endl;
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