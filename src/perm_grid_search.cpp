/* ----------------------------------------------------------------------------
 * Nanyang Technological University
 * Authors: Cao Muqing, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */
#include <vector>
#include <random>
#include "ros/ros.h"

#include "timer.hpp"
#include "termcolor.hpp"
#include "perm_grid_search.hpp"

using namespace termcolor;
using namespace Eigen;

typedef neptimers::Timer MyTimer;
typedef neptimers::ROSTimer MyROSTimer;

// template <typename T>
// int mu::sgn(T val)
// {
//   return (T(0) < val) - (val < T(0));
// }

perm_grid_search::perm_grid_search(int num_agent)
{
  number_of_agents_ = num_agent;
  node_num_max_ = node_num_max_< power_int(4, number_of_agents_+4)?node_num_max_:
                  power_int(4, number_of_agents_+4); 
  
  node_pool_.resize(node_num_max_);
  for (int i = 0; i < node_num_max_; i++) //pre-allocate the memory for the nodes
  {
    node_pool_[i] = new Node;
    node_pool_[i]->perm = MatrixXi::Zero(2, number_of_agents_);
    node_pool_[i]->interaction = MatrixXi::Zero(number_of_agents_, number_of_agents_);
    node_pool_[i]->agent_positions = MatrixXi::Zero(2, number_of_agents_);
  }  
  std::cout << bold << blue << "Setting Up permutation search! " << reset << std::endl;

  goalPos_ = MatrixXi::Zero(2, number_of_agents_);
}

perm_grid_search::~perm_grid_search()
{
}

void perm_grid_search::setUp()
{

}

int perm_grid_search::getIdxNode(NodePtr node)
{
  int idx = 0;
  int binary = 1;
  for (int i=0; i<2; i++)
  {
    for (int j=0; j<number_of_agents_; j++)
    {
      idx += binary * node->perm(i,j);
      binary  *= 2;
    }
  }
  for (int j=0; j<number_of_agents_; j++)
  {
    idx += binary * node->interaction(0,j);
    binary  *= 2;
  }
  return idx;
}


void perm_grid_search::setBias(double bias)
{
  bias_ = bias;
}

void perm_grid_search::setGoal(Eigen::Matrix<int, 2, Eigen::Dynamic> goalPerm)
{
  goalPerm_ = goalPerm;
  fromPermToPositions(goalPerm_, goalPos_);
  clearProcess();  
}

void perm_grid_search::setRunTime(double max_runtime)
{
  max_runtime_ = max_runtime;
}



double perm_grid_search::getH(NodePtr node)
{
  double h = 0.0;
  MatrixXd diff = (node->agent_positions - goalPos_).cast<double>();
  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < number_of_agents_; ++j)
    {
      h += fabs(diff(i,j));
    }
  }

  // h = (node->agent_positions - goalPos_).norm();
  return h;
}

double perm_grid_search::getG(NodePtr node)
{
  // return node->previous->g + T_span_;
  return node->previous->g + 1.0;

}

double perm_grid_search::h(Eigen::Vector2d pos)
{
  return 0;
}


void perm_grid_search::recoverPath(NodePtr result_ptr)
{
  perm_path_.clear();
  pos_path_.clear();
  std::vector<Eigen::Matrix<int, 2, Dynamic>> perm_path, pos_path;
  if (result_ptr == NULL)
  {
    return;
  }

  NodePtr tmp = result_ptr;

  while (tmp != NULL)
  {
    // std::cout <<bold<<green<< "feasible permutation path is" << std::endl;
    // std::cout <<bold<<green<< tmp->perm << std::endl;
    std::cout <<bold<<green<< "feasible agent positions path is" << std::endl;
    std::cout <<bold<<green<< tmp->agent_positions << std::endl;    
    std::cout <<bold<<green<< "feasible agent interaction is" << std::endl;
    std::cout <<bold<<green<< tmp->interaction << std::endl;     
    perm_path.push_back(tmp->perm);
    pos_path.push_back(tmp->agent_positions);
    tmp = tmp->previous;
  }

  std::reverse(std::begin(perm_path), std::end(perm_path));  
  std::reverse(std::begin(pos_path), std::end(pos_path));  
  perm_path_ = perm_path;
  pos_path_ = pos_path;
}

void perm_grid_search::getPermPath(std::vector<Eigen::Matrix<int, 2, Dynamic>>& result)
{
  result.clear();

  result = perm_path_;

}

void perm_grid_search::getPosPath(std::vector<Eigen::Matrix<int, 2, Dynamic>>& result)
{
  result.clear();

  result = pos_path_;

}

void perm_grid_search::recoverPwpOut(NodePtr result_ptr)
{


}


void perm_grid_search::generatePwpOut()
{


}


void perm_grid_search::getCurrentSamplePath(const NodePtr current, std::vector<Eigen::Vector3d>& output)
{


  std::vector<Eigen::Vector3d> output_tmp;

  output = output_tmp;
}

bool perm_grid_search::entanglesWithOtherAgents(NodePtr current, double& arc_length)
{


  return false;
}

void perm_grid_search::expandAndAddToQueue2(NodePtr current)
{
  for (int j=0; j<number_of_agents_-1; j++)
  {
    for (int i: {0, 1})
    {
      if (node_used_num_ == node_num_max_-1)
      {
        std::cout <<bold<<red<< "run out of memory." << std::endl;
        return;
      }

      NodePtr neighbor = node_pool_[node_used_num_];
      neighbor->perm = current->perm;
      neighbor->agent_positions = current->agent_positions;
      neighbor->interaction = current->interaction;        

      int agent_current = current->perm(i, j);      
      int agent_to_exchange = current->perm(i, j+1);
      neighbor->perm(i, j) = agent_to_exchange;
      neighbor->perm(i, j+1) = agent_current;         
      neighbor->agent_positions(i, agent_current) += 1;
      neighbor->agent_positions(i, agent_to_exchange) -= 1;    

      for (int k=0; k<number_of_agents_; k++) //get agent start positions
      {
        for (int kk:{0,1})
        {
            if(neighbor->agent_positions(kk,neighbor->perm(kk,k)) != k)
            {
                std::cout << red << "wrong!!!!!!!!! " <<reset << std::endl;      
                exit(-1);
            }
        }
      }

      int infront_or_behind = (current->agent_positions(1-i, agent_current) 
                              < current->agent_positions(1-i, agent_to_exchange))?1:-1;

      if (i==0) //first axis
      {
        // infront_or_behind = -infront_or_behind; //just to keep meaning consistent, 
                                                          //should not affect algorithm
        if (agent_current< agent_to_exchange) //only keeping the upper diagonal 
        {
          neighbor->interaction(agent_current, agent_to_exchange) += 
            infront_or_behind;
        }
        else
        {
          neighbor->interaction(agent_to_exchange, agent_current) += 
            infront_or_behind;
        }        
      }
      else //second axis
      {
        if (agent_current< agent_to_exchange) //LOWER diagonal
        {
          neighbor->interaction(agent_to_exchange, agent_current) += 
            infront_or_behind;
        }
        else
        {
          neighbor->interaction(agent_current, agent_to_exchange) += 
            infront_or_behind;
        }          
      }

      if (abs(neighbor->interaction(agent_current, agent_to_exchange))>=2 || //does not satisfy condition
        abs(neighbor->interaction(agent_to_exchange, agent_current))>=2)
      {
        continue;
      }
      neighbor-> previous = current;
      neighbor->h = getH(neighbor); 
      neighbor->g = getG(neighbor);
      MatrixXi idx(3, number_of_agents_);
      idx << neighbor->perm, neighbor->interaction.row(0);

      NodePtr nodeptr;
      nodeptr = generated_nodes_.find(idx);

      if (nodeptr != NULL) //same node has been added to open list before
      {
        if (nodeptr->state == 1) //in open list
        {
          //update the node if the new one is better
          // if (neighbor->g + bias_*neighbor->h < nodeptr->g + bias_*nodeptr->h)
          // {
          //   nodeptr->previous = current;
          //   nodeptr->g = neighbor->g;
          //   nodeptr->h = neighbor->h;
          // }
          continue;
        }
        else //already expanded, in closed list
        {
          continue;
        }
      }
      else generated_nodes_.insert(idx, neighbor); //node generated yet

      neighbor-> state = 1;
      openList_.push(neighbor);
      std::cout << red << "pushing into open list!" <<reset << std::endl;      
      node_used_num_ += 1;   
      // std::cout<<" "<<node_used_num_<<" ";

      cont:;
    }
  }
}

void perm_grid_search::expandAndAddToQueue(NodePtr current)
{
  MyTimer timer_expand(true);

  std::cout << green << "current permutation is " <<reset << std::endl;      
  std::cout << green << current->perm <<reset << std::endl;      
  std::cout << green << current->agent_positions <<reset << std::endl;   

  for (int i :{0,1})
  {
    for (int j: {-1, 0, 1})
    {
      if (j==0 && i==1) continue; //j=0 is when robot does not move
      if (node_used_num_ == node_num_max_-1)
      {
        std::cout <<bold<<red<< "run out of memory." << std::endl;
        return;
      } 

      NodePtr neighbor = node_pool_[node_used_num_];
      neighbor->perm = current->perm;
      neighbor->agent_positions = current->agent_positions;
      neighbor->interaction = current->interaction;


      if (current->agent_positions(i, current->next_agent)+j >= number_of_agents_ ||
          current->agent_positions(i, current->next_agent)+j < 0)
      {
         continue;
      }     

      int agent_to_exchange = 
        current->perm(i, current->agent_positions(i, current->next_agent)+j);
      neighbor->perm(i, current->agent_positions(i, current->next_agent))
        = agent_to_exchange;
      neighbor->perm(i, current->agent_positions(i, current->next_agent)+j)
        = current->next_agent;         
      neighbor->agent_positions(i, current->next_agent) += j;
      neighbor->agent_positions(i, agent_to_exchange) -= j;       
      
      std::cout << red << "neighbor permutation is " <<reset << std::endl;      
      std::cout << red << neighbor->perm <<reset << std::endl;      
      std::cout << red << neighbor->agent_positions <<reset << std::endl;      

      for (int i=0; i<number_of_agents_; i++) //get agent start positions
      {
        for (int j:{0,1})
        {
            if(neighbor->agent_positions(j,neighbor->perm(j,i)) != i)
            {
                std::cout << red << "wrong!!!!!!!!! " <<reset << std::endl;      
                exit(-1);
            }
        }
      }

      int infront_or_behind = (current->agent_positions(1-i, current->next_agent) 
                              < current->agent_positions(1-i, agent_to_exchange))?1:-1;

      if (i==0) infront_or_behind = -infront_or_behind; //just to keep meaning consistent, 
                                                        //should not affect algorithm
      if (current->next_agent< agent_to_exchange) //only keeping the upper diagonal
      {
        neighbor->interaction(current->next_agent, agent_to_exchange) += 
          infront_or_behind*j;
      }
      else
      {
        neighbor->interaction(agent_to_exchange, current->next_agent) += 
          infront_or_behind*j;
      }

      neighbor->h = getH(neighbor);
      neighbor->g = current->g + 2.0;
      neighbor->next_agent = (neighbor->next_agent+1)%number_of_agents_;
      if (neighbor->next_agent == 0) neighbor-> index +=1;

      if (neighbor->next_agent == 0) //only check if a node is already expanded if
                                      // it is a standard node (not intermediate)
      {
        MatrixXi idx(3, number_of_agents_);
        idx << neighbor->perm, neighbor->interaction.row(0);

        NodePtr nodeptr;
        nodeptr = generated_nodes_.find(idx);

        if (nodeptr != NULL) //same node has been added to open list before
        {
          if (nodeptr->state == 1) //in open list
          {
            //update the node if the new one is better
            // if (neighbor->g + bias_*neighbor->h < nodeptr->g + bias_*nodeptr->h)
            // {
            //   nodeptr->previous = current;
            //   nodeptr->g = neighbor->g;
            //   nodeptr->h = neighbor->h;
            // }
            continue;
          }
          else //already expanded, in closed list
          {
            continue;
          }
        }
        else generated_nodes_.insert(idx, neighbor); //node generated yet

      }

      // std::cout << green << neighbor->qi.transpose() << " cost=" << neighbor->g + bias_ * neighbor->h << reset <<
      // std::endl;
      neighbor-> previous = current;
      neighbor-> state = 1;
      openList_.push(neighbor);
      std::cout << red << "pushing into open list!" <<reset << std::endl;      
      node_used_num_ += 1;   
      // std::cout<<" "<<node_used_num_<<" ";

      cont:;
    }    
  }
}


void perm_grid_search::clearProcess()
{

  best_node_ptr_ = NULL;
  generated_nodes_.clear();


  CompareCost comparer;
  comparer.bias = bias_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, CompareCost> openList_tmp(comparer);
  openList_.swap(openList_tmp);  

  // for (int i = 0; i < node_used_num_; i++)
  // {
  //   NodePtr node = node_pool_[i];
  //   node->previous = NULL;
  // }
  node_used_num_ = 0;  


  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
}


bool perm_grid_search::run(Eigen::Matrix<int, 2, Eigen::Dynamic> start_perm,  
                           Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& interaction, 
                           int& status)
{
  std::cout << "[A*] Running..." << std::endl;

  MyTimer timer_astar(true);

  NodePtr startPtr = node_pool_[0];
  startPtr->next_agent = 0;
  startPtr->index = 0;
  startPtr->perm = start_perm;
  startPtr->interaction = interaction;
  startPtr-> g =0.0;
  fromPermToPositions(start_perm, startPtr->agent_positions);
  // for (int i=0; i<number_of_agents_; i++) //get agent start positions
  // {
  //   for (int j:{0,1})
  //   {
  //       startPtr->agent_positions(j,start_perm(j,i)) = i;
  //   }
  // }

  MatrixXi idx(3, number_of_agents_);
  idx << start_perm, interaction.row(0);
  generated_nodes_.insert(idx, startPtr); 
  openList_.push(startPtr);
  node_used_num_ += 1;

  int RUNTIME_REACHED = 0;
  int GOAL_REACHED = 1;
  int EMPTY_OPENLIST = 2;
  int OUT_OF_MEMORY = 3;
  NodePtr current_ptr;

  while (openList_.size() > 0)
  {
    // std::cout << "[A*] current open list size is "<<openList_.size() << std::endl;
    // Check if runtime is over
    if (timer_astar.ElapsedMs() > (max_runtime_ * 1000))
    {
      std::cout << "[A*] Max Runtime was reached" << std::endl;
      std::cout << "[A*] Run time is " << timer_astar.ElapsedMs()<<std::endl;      
      status = RUNTIME_REACHED;
      goto exitloop;
    }

    current_ptr = openList_.top();  // copy the first element onto current_ptr
    openList_.pop();                 // remove it from the list
    current_ptr-> state = -1;

    double dist = (goalPos_ - current_ptr->agent_positions).norm();

    /////////////////////

    // check if we are already in the goal
    if ((dist < 0.1))
    {

      runtime_this_round_ = (double)timer_astar.ElapsedUs()/1000.0;
      std::cout << "[A*] Goal was reached!" << std::endl;
      std::cout << bold << green <<"[A*] solving time is" << runtime_this_round_ << "ms !!"<<std::endl;
      std::cout << bold << green <<"[A*] num of expansion is" << node_used_num_ << " !!"<<std::endl;
      status = GOAL_REACHED;
      goto exitloop;
    
    }

    expandAndAddToQueue2(current_ptr);

    if (node_used_num_ == node_num_max_-1)
    {
      std::cout <<bold<<red<< "[A*] run out of memory." << std::endl;
      status = OUT_OF_MEMORY;
      goto exitloop;
    } 
  }

  std::cout << "[A*] openList_ is empty" << std::endl;
  std::cout << "[A*] Run time is " << timer_astar.ElapsedMs()<<std::endl;      
  status = EMPTY_OPENLIST;
  goto exitloop;

exitloop:

  // std::cout << "status= " << status << std::endl;
  // std::cout << "expanded_nodes_.size()= " << expanded_nodes_.size() << std::endl;
  // std::cout << "complete_closest_dist_so_far_= " << complete_closest_dist_so_far_ << std::endl;

  NodePtr best_node_ptr = NULL;

  bool have_a_solution = false;

  if (status == GOAL_REACHED)
  {
    std::cout << "[A*] choosing current_ptr as solution" << std::endl;
    best_node_ptr = current_ptr;
  }
  else if (status == RUNTIME_REACHED || status == EMPTY_OPENLIST
            ||status == OUT_OF_MEMORY)
  {
    std::cout << "[A*] not finding a feasible solution!" << std::endl;
    // best_node_ptr = closest_result_so_far_ptr_;
    if (status == RUNTIME_REACHED)
    {
      std::cout <<bold<<red<< "[A*] RUN TIME REACHED!" << std::endl;
    }
    std::cout << "[A*] num of nodes expanded is " <<node_used_num_<<std::endl;
    return false;
  }
  else
  {
    std::cout << red << "This state should never occur" << reset << std::endl;
    abort();
    return false;
  }


  recoverPath(best_node_ptr);
  // recoverPwpOut(best_node_ptr);
  best_node_ptr_ = best_node_ptr;

  return true;
}

bool perm_grid_search::runonce(std::vector<Eigen::Vector3d>& tmp_path, int &status)
{


  return true;
}

void perm_grid_search::getRuntime(double& runtime_this_round, double& time_spent_contact_pt,
                                  int& node_used_num)
{

}


unsigned int perm_grid_search::power_int(unsigned int base, unsigned int exponent)
{
  if (exponent == 0) { return 1;    }
  if (base < 2)      { return base; }

  unsigned int result = 1;

  for (unsigned int term = base; ; term = term * term)
  { 
    if (exponent % 2 != 0) { result *= term; }
    exponent /= 2;
    if (exponent == 0)     { break; }
  }

  return result;
}

void perm_grid_search::fromPermToPositions(Eigen::Matrix<int, 2, Eigen::Dynamic>& perm,
                                          Eigen::Matrix<int, 2, Eigen::Dynamic>& positions)
{

  for (int i=0; i<number_of_agents_; i++) //get agent start positions
  {
    for (int j:{0,1})
    {
        positions(j,perm(j,i)) = i;
    }
  }

}