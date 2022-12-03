#include <ros/ros.h>
#include <math.h>
#include "perm_grid_search.hpp"
#include <Eigen/Dense>

int main(int argc, char *argv[]) {
	// initialize ROS
	ros::init(argc, argv, "perm_grid_ros");
	ros::NodeHandle nh("~");

	std::unique_ptr<perm_grid_search> perm_search = std::unique_ptr<perm_grid_search>(
													new perm_grid_search(5));
	Eigen::MatrixXi start_perm(2,5);
	start_perm<< 0, 1, 2, 3, 4,
				 0, 1, 2, 3, 4;
	Eigen::MatrixXi goal_perm(2,5);
	goal_perm<< 4, 3, 2, 1, 0,
				 4, 3, 2, 1, 0;	
	Eigen::MatrixXi start_interaction = Eigen::MatrixXi::Zero(5, 5);
	perm_search ->setGoal(goal_perm);
	int search_status = 0;
	perm_search ->run(start_perm, start_interaction, search_status);
			 			 
  	ros::spin();
	ros::waitForShutdown();
 
		// ros::spinOnce();
	// }
	return 0;
}