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
#include "entangle_check.hpp"

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
  std::cout << bold << blue << "Setting Up permutation search! " << reset << std::endl;

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
    node_pool_[i]->interact_3d = vector4d<std::vector<Eigen::Vector2i>>(2, number_of_agents_, 
                                  number_of_agents_, number_of_agents_);

  }  
  std::cout << bold << blue << "Finish Setting Up permutation search! " << reset << std::endl;

  goalPos_ = MatrixXi::Zero(2, number_of_agents_);
  for (int i = 0; i < number_of_agents_-1; ++i)
  {
    list_of_agents_.push_back(i);
  }
}

perm_grid_search::~perm_grid_search()
{
}

void perm_grid_search::setUp()
{

}

void perm_grid_search::encode3dintoInteract(MatrixXi& interaction, 
  vector4d<std::vector<Eigen::Vector2i>>& interact_3d)
{
  for (auto dim: {0,1})
  {
    for (int i = 0; i < number_of_agents_; ++i)
    {
      interaction(i,i) = 0;
      for (int j = i+1; j < number_of_agents_; ++j)
      {
        for (int k = j+1; k < number_of_agents_; ++k)
        {
          interaction(i,i) += dim*number_of_agents_*power_int(10,3) +
            j*k *
            encode3dSepcific(interact_3d(dim, i,j,k));
        }
      }
    }
  }
}

int perm_grid_search::encode3dSepcific( 
  std::vector<Eigen::Vector2i>& braid)
{
  if (braid.size()==0)
  {
    return 0;
  }
  else if (braid.size()==1)
  {
    if (braid[0](0)==1)
    {
      if (braid[0](1)==1)
      {
        return 10;
      }
      else if (braid[0](1)==-1)
      {
        return 12;
      }
      else
      {
        std::cout<<"something wrong 001001!"<<std::endl;
        exit(-1);
      }      
    }
    else if (braid[0](0)==2)
    {
      if (braid[0](1)==1)
      {
        return 14;
      }
      else if (braid[0](1)==-1)
      {
        return 18;
      }
      else
      {
        std::cout<<"something wrong 001002!"<<std::endl;
        exit(-1);
      }      
    }
    else
    {
      std::cout<<"something wrong 001003!"<<std::endl;
      exit(-1);
    }
  }
  else if (braid.size()==2)
  {

    switch (braid[0](0)){
    case 1: //12
      switch (braid[0](1)){
      case 1:
        switch (braid[1](1)){ 
        case 1: return 21;
        break;
        case -1: return 22;
        break;
        default:
        std::cout<<"something wrong 002001!"<<std::endl;
        exit(-1);  
        }
      break;
      case -1:
        switch (braid[1](1)){
        case 1: return 23;
        break;
        case -1: return 24;
        break;
        default:
        std::cout<<"something wrong 002002!"<<std::endl;
        exit(-1);  
        }
      break;
      default:
        std::cout<<"something wrong 002003!"<<std::endl;
        exit(-1);  
      }
    break;
    case 2: //21
    switch (braid[0](1)){
      case 1:
        switch (braid[1](1)){
        case 1: return 25;
        break;
        case -1: return 26;
        break;
        default:
        std::cout<<"something wrong 002004!"<<std::endl;
        exit(-1);  
        }
      break;
      case -1:
        switch (braid[1](1)){
        case 1: return 27;
        break;
        case -1: return 28;
        break;
        default:
        std::cout<<"something wrong 002005!"<<std::endl;
        exit(-1);  
        }
      break;
      default:
        std::cout<<"something wrong 002006!"<<std::endl;
        exit(-1);  
      }
    break;
    default:
      std::cout<<"something wrong 002007!"<<std::endl;
      for (int ii = 0; ii < braid.size(); ++ii)
      {
          std::cout<<red<<braid[ii]<<std::endl;

      }        
      exit(-1); 
    }
  }
  else if (braid.size()==3)
  {
    switch (braid[0](0)){
    case 1: //121
      switch (braid[0](1)){
      case 1:
        switch (braid[1](1)){ 
        case 1: 
          switch (braid[2](1)){   
            case 1: return 31; //121
            break;
            case -1: return 32; //121^-1
            break;
            default:
            std::cout<<"something wrong 003001!"<<std::endl;
            exit(-1); 
          }      
        break;
        case -1: 
          switch (braid[2](1)){   
            case 1: return 37; //12^-1 1
            break;
            case -1: return 33; //12^-1 1^-1
            break;
            default:
            std::cout<<"something wrong 003002!"<<std::endl;
            exit(-1); 
          }         
        break;
        default:
        std::cout<<"something wrong 003003!"<<std::endl;
        exit(-1);  
        }
      break;
      case -1:
        switch (braid[1](1)){
        case 1: 
          switch (braid[2](1)){   
            case 1: return 36; //1^-1 2 1
            break;
            case -1: return 39; //1^-1 2 1^-1
            break;
            default:
            std::cout<<"something wrong 003004!"<<std::endl;
            exit(-1); 
          }         
        break;
        case -1: 
          switch (braid[2](1)){   
            case 1: return 35; //1^-1 2^-1 1
            break;
            case -1: return 34; //1^-1 2^-1 1^-1
            break;
            default:
            std::cout<<"something wrong 003005!"<<std::endl;
            exit(-1); 
          }         
        break;
        default:
        std::cout<<"something wrong 003006!"<<std::endl;
        exit(-1);  
        }
      break;
      default:
        std::cout<<"something wrong 003007!"<<std::endl;
        exit(-1);  
      }
    break;
    case 2: //212
    switch (braid[0](1)){
      case 1:
        switch (braid[1](1)){
        case 1: 
          switch (braid[2](1)){   
            case 1: return 31; //212
            break;
            case -1: return 36; //212^-1
            break;
            default:
            std::cout<<"something wrong 003008!"<<std::endl;
            exit(-1); 
          }         
        break;
        case -1: 
          switch (braid[2](1)){   
            case 1: return 38; //21^-1 2
            break;
            case -1: return 35; //21^-1 2^-1
            break;
            default:
            std::cout<<"something wrong 003009!"<<std::endl;
            exit(-1); 
          }         
        break;
        default:
        std::cout<<"something wrong 003000!"<<std::endl;
        exit(-1);  
        }
      break;
      case -1:
        switch (braid[1](1)){
        case 1: 
          switch (braid[2](1)){   
            case 1: return 32; //2^-1 1 2
            break;
            case -1: return 30; //2^-1 1 2^-1
            break;
            default:
            std::cout<<"something wrong 0030011!"<<std::endl;
            exit(-1); 
          }         
        break;
        case -1: 
          switch (braid[2](1)){   
            case 1: return 33; //2^-1 1^-1 2
            break;
            case -1: return 34; //2^-1 1^-1 2^-1
            break;
            default:
            std::cout<<"something wrong 0030012!"<<std::endl;
            exit(-1); 
          }         
        break;
        default:
        std::cout<<"something wrong 0030013!"<<std::endl;
        exit(-1);  
        }
      break;
      default:
        std::cout<<"something wrong 0030014!"<<std::endl;
        exit(-1);  
      }
    break;
    default:
      std::cout<<"something wrong 0030015!"<<std::endl;
      for (int ii = 0; ii < braid.size(); ++ii)
      {
          std::cout<<red<<braid[ii]<<std::endl;

      }        
      exit(-1); 
    }
  }
  std::cout<<"something wrong 005001!"<<std::endl;
  for (int ii = 0; ii < braid.size(); ++ii)
  {
      std::cout<<red<<braid[ii]<<std::endl;

  }    
  // exit(-1);  
  return -1;
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
  std::vector<neptune2::PermSequence> sequence_vector;
  neptune2::PermSequence permsequence;
  int prev_axis = tmp->action[1];
  std::vector<int> prev_perm;
  
  while (tmp != NULL)
  {
    // std::cout <<bold<<green<< "feasible permutation path is" << std::endl;
    // // std::cout <<bold<<green<< tmp->perm << std::endl;
    // std::cout <<bold<<green<< "feasible agent positions path is" << std::endl;
    // std::cout <<bold<<green<< tmp->agent_positions << std::endl;    
    // std::cout <<bold<<green<< "feasible agent interaction is" << std::endl;
    // std::cout <<bold<<green<< tmp->interaction << std::endl;     
    perm_path.push_back(tmp->perm);
    pos_path.push_back(tmp->agent_positions);

    if (tmp->previous != NULL) //not the first node
    {
      neptune2::PermAction action1;
      action1.perm_id = tmp->action[0];
      action1.axis = tmp->action[1];
      action1.action = tmp->action[2];
      permsequence.actions.push_back(action1);
    }
    else
    { //set up the perm vector of the initial node
      permsequence.perm.clear();
      for (size_t i = 0; i < 2; i++)
      {
        for (size_t j = 0; j < tmp->previous->perm.cols(); j++)
        {
          permsequence.perm.push_back(tmp->previous->perm(i,j));
        }
      }
    }

    tmp = tmp->previous;
  }

  std::reverse(std::begin(perm_path), std::end(perm_path));  
  std::reverse(std::begin(pos_path), std::end(pos_path));  
  std::reverse(std::begin(permsequence.actions), std::end(permsequence.actions));  
  perm_path_ = perm_path;
  pos_path_ = pos_path;
  permSequence_ = permsequence;
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

void perm_grid_search::getPermSequence(neptune2::PermSequence& result)
{
  result.actions.clear();

  result = permSequence_;

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
  for (auto j:list_of_agents_)
  {
    for (int dim: {0, 1})
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
      neighbor->interact_3d = current->interact_3d; 
      
      int agent_current = current->perm(dim, j);      
      int agent_to_exchange = current->perm(dim, j+1);
      neighbor->perm(dim, j) = agent_to_exchange;
      neighbor->perm(dim, j+1) = agent_current;         
      neighbor->agent_positions(dim, agent_current) += 1;
      neighbor->agent_positions(dim, agent_to_exchange) -= 1;    

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

      int infront_or_behind = (current->agent_positions(1-dim, agent_current) 
                              < current->agent_positions(1-dim, agent_to_exchange))?1:-1;
      neighbor->action[0] = j;
      neighbor->action[1] = dim;
      neighbor->action[2] = -infront_or_behind; // +1 is crossing on the + side

      if (dim==0) //first axis
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

      bool satisfy_condition = true;
      for (int i = 0; i < number_of_agents_; ++i)
      {
        if (i==agent_current ||i==agent_to_exchange) continue;
        Vector2i to_add;
        if (neighbor->agent_positions(dim, i)<neighbor->agent_positions(dim, agent_current))
        {
          to_add << 2, infront_or_behind;
        }
        else 
        {
          to_add << 1, infront_or_behind;
        }
        std::vector<int> agents_id {i, agent_current, agent_to_exchange};
        std::sort(agents_id.begin(), agents_id.end());
        satisfy_condition = check3robotEnt(neighbor->interact_3d(dim,
                                agents_id[0], agents_id[1], agents_id[2]), to_add);
        if (!satisfy_condition) break;
        if (neighbor->interact_3d(dim,agents_id[0], agents_id[1], agents_id[2]).size()==3)
        {
          if (neighbor->interact_3d(dim,agents_id[0], agents_id[1], agents_id[2])[0]==
            neighbor->interact_3d(dim,agents_id[0], agents_id[1], agents_id[2])[1] ||
            neighbor->interact_3d(dim,agents_id[0], agents_id[1], agents_id[2])[1]==
            neighbor->interact_3d(dim,agents_id[0], agents_id[1], agents_id[2])[2])
          {
            std::cout<<red<<"strange 101!!"<<std::endl;

            std::cout<<red<<"id0 : "<<agents_id[0]<<" id1 : "<<agents_id[1]<<" id2: "<<agents_id[2]<<std::endl; 
            std::cout<<red<<"dim : "<<dim<<"agent : "<<agent_current<<"agent_to_exchange: "<<agent_to_exchange<<std::endl; 


            NodePtr tmp = neighbor;

            while (tmp != NULL)
            {
              std::cout<<red<<"================================================"<<std::endl;          

              std::cout<<red<<"perms:"<<std::endl<<tmp->perm.row(dim)<<std::endl;          
              std::cout<<red<<"the other dim perms:"<<std::endl<<tmp->perm.row(1-dim)<<std::endl;          
               std::cout<<red<<"interaction:"<<std::endl<<tmp->interaction<<std::endl;   
              for (int ii = 0; ii < 
                tmp->interact_3d(dim,agents_id[0], agents_id[1], agents_id[2]).size(); ++ii)
              {
                  std::cout<<red<<tmp->interact_3d(dim,agents_id[0], agents_id[1], agents_id[2])[ii]<<std::endl;

              }   
              tmp = tmp->previous;
            }                  
            // exit(-1);
          }

        }

      }
      if (!satisfy_condition) continue;
      encode3dintoInteract(neighbor->interaction, neighbor->interact_3d);
      neighbor-> previous = current;
      neighbor->h = getH(neighbor); 
      neighbor->g = getG(neighbor);
      MatrixXi idx(number_of_agents_+2, number_of_agents_);
      idx << neighbor->perm, neighbor->interaction;

      NodePtr nodeptr;
      nodeptr = generated_nodes_.find(idx);

      if (nodeptr != NULL) //same node has been added to open list before
      {

        // if (neighbor->interaction == nodeptr->interaction)
        // {
        //   for (int dim :{0,1})
        //   {
        //     for (int i = 0; i < number_of_agents_; ++i)
        //     {
        //       for (int j = i+1; j < number_of_agents_; ++j)
        //       {
        //         for (int k = j+1; k < number_of_agents_; ++k)
        //         {
        //           if (encode3dSepcific(nodeptr->interact_3d(dim, i, j, k))!=
        //               encode3dSepcific(neighbor->interact_3d(dim, i, j, k)))
        //           {
        //             std::cout<<red<<"found such case!!"<<std::endl;
        //             std::cout<<red<<"ijk:: "<<i<<j<<k<<std::endl;  
        //             std::cout<<red<<"interaction:: "<<neighbor->interaction<<std::endl;
        //             std::cout<<red<<"nodeptr->interact_3d(dim, i, j, k) is "<<std::endl;
        //             for (int ii = 0; ii < nodeptr->interact_3d(dim, i, j, k).size(); ++ii)
        //             {
        //                 std::cout<<red<<nodeptr->interact_3d(dim, i, j, k)[ii]<<std::endl;

        //             }                
        //             std::cout<<green<<"neighbor->interact_3d(dim, i, j, k) is "<<std::endl;
        //             for (int ii = 0; ii < neighbor->interact_3d(dim, i, j, k).size(); ++ii)
        //             {
        //                 std::cout<<green<<neighbor->interact_3d(dim, i, j, k)[ii]<<std::endl;

        //             } 
        //             // exit(-1);
        //           }
        //         }
        //       }

        //     }
        //   }

        //   // std::cout<<red<<"interaction the same, but 3d interaction also same!!"<<std::endl;
        // }

        if (nodeptr->state == 1) //in open list
        {
          //update the node if the new one is better
          if (neighbor->g + bias_*neighbor->h < nodeptr->g + bias_*nodeptr->h)
          {
            nodeptr->previous = current;
            nodeptr->g = neighbor->g;
            nodeptr->h = neighbor->h;
            // std::cout<<blue<<"updating new parrent!!"<<std::endl;
          }
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
      // std::cout << red << "pushing into open list!" <<reset << std::endl;      
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
        idx << neighbor->perm, neighbor->interaction.row(0), neighbor->interaction.row(1),
               neighbor->interaction.row(2);

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


}


bool perm_grid_search::run(Eigen::Matrix<int, 2, Eigen::Dynamic> start_perm,  
                           Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& interaction, 
                           vector4d<std::vector<Eigen::Vector2i>>& interact_3d, int& status,
                           bool is_second_try)
{
  std::cout << "[A*] Running..." << std::endl;

  MyTimer timer_astar(true);

  NodePtr startPtr = node_pool_[0];
  startPtr->next_agent = 0;
  startPtr->index = 0;
  startPtr->perm = start_perm;
  startPtr->interaction = interaction;
  startPtr->interact_3d = interact_3d;
  startPtr-> g =0.0;
  fromPermToPositions(start_perm, startPtr->agent_positions);
  if (pastGoalPos_.empty())
  {
    pastGoalPos_.push_back(startPtr->agent_positions);
  }    

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

    if (!is_second_try)
    {
      Eigen::Matrix<int, 2, Eigen::Dynamic> goalpos_tmp = goalPos_;
      bool found_secondary_soln = false;
      for (int i = pastGoalPos_.size()-2; i >= 0; --i)
      {
        std::cout <<green<< "==========[A*] start a new search round!!========= " <<std::endl;        
        clearProcess();
        goalPos_ = pastGoalPos_[i];
        int res;
        run(start_perm,  interaction, interact_3d, res, true);        
        if (res == GOAL_REACHED) 
        {
          std::cout <<green<< "==========[A*] found itermediate goal, now try reaching the terminal goal!!========= " <<std::endl;    
          std::vector<Eigen::Matrix<int, 2, Eigen::Dynamic>> pos_path_inter = pos_path_;
          goalPos_ = goalpos_tmp;
          Eigen::Matrix<int, 2, Eigen::Dynamic> start_perm_tmp = best_node_ptr_->perm;  
          Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> interaction_tmp = best_node_ptr_->interaction; 
          vector4d<std::vector<Eigen::Vector2i>> interact_3d_tmp = best_node_ptr_->interact_3d;
          int node_used_num_1 = node_used_num_;
          clearProcess();
          int sta;
          run(start_perm_tmp,  interaction_tmp, interact_3d_tmp, sta, true);
          if (sta == GOAL_REACHED)
          {
            found_secondary_soln = true;
            for (int ii = pos_path_inter.size()-1; ii >= 0; --ii) //recover the overall path
            {
              pos_path_.insert(pos_path_.begin(), pos_path_inter[ii]);
            }
            status = GOAL_REACHED;
            runtime_this_round_ = (double)timer_astar.ElapsedUs()/1000.0;
            node_used_num_ += node_used_num_1;
            return true;
          }
        }
      }
    }

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

  if (!is_second_try)
  {
    pastGoalPos_.push_back(goalPos_);
  }

  return true;
}

bool perm_grid_search::runonce(std::vector<Eigen::Vector3d>& tmp_path, int &status)
{


  return true;
}

bool perm_grid_search::retrieveAllthese(Eigen::Matrix<int, 2, Eigen::Dynamic>& perm_tmp, 
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& interaction_tmp,
  vector4d<std::vector<Eigen::Vector2i>>& interact_3d_tmp)
{
  perm_tmp = best_node_ptr_->perm;  
  interaction_tmp = best_node_ptr_->interaction; 
  interact_3d_tmp = best_node_ptr_->interact_3d;
}

void perm_grid_search::getRuntime(double& runtime_this_round,
                                  int& node_used_num)
{
  runtime_this_round = runtime_this_round_;
  node_used_num = node_used_num_;
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

bool perm_grid_search::check3robotEnt(std::vector<Eigen::Vector2i>& v, Eigen::Vector2i to_add)
{
  if (v.empty())
  {
    v.push_back(to_add);
    return true;
  }

  if (v.back()(0)==to_add(0) && v.back()(1)!=to_add(1)) //cancellation
  {
    v.pop_back();
    return true;
  }
  v.push_back(to_add);
  if (v.size()==4)
  {
    if (v[0](1)==v[1](1) && v[1](1)==v[2](1)) //same sign
    {
      if (v[1](1) != v[3](1))  //this may be always true
      {
        v.erase(v.begin());
        v.pop_back();
        return true;
      }
    }
    else if (v[1](1)==v[2](1) && v[2](1)==v[3](1))
    {
      if (v[0](1) != v[2](1)) //this may be always true
      {
        v.erase(v.begin());
        v.pop_back();
        return true;
      }      
    }
    else if (v[0](1)==v[1](1) && v[2](1)==v[3](1) && v[1](1) != v[2](1))
    {
        v.pop_back();
        v.pop_back();
        v[0](1) = v[0](1) * (int)(-1); //reverse sign
        v[0](0) = (v[0](0)==2? 1 :2); //reverse braid incex
        v[1](0) = (v[1](0)==2? 1 :2);  
        return true;  
    }
    return false;
  }
  return true;
}