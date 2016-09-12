/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <iostream>
#include <fstream>
#include <string>
//#include <sstream>
#include <Inventor/SoDB.h>

#include "Kauthamplanner.h"

#include "ros/ros.h"

using namespace std;

int main(int argc, char* argv[]){

	if( argv[1] == "-h" ){
	    std::cout << "HELP!!! \n";
	    return 0;
	  }

	//=====================
	SoDB::init();

	ros::init(argc, argv, "kautham_planner_ros");
	ros::NodeHandle n;

	//string absPath = argv[1];
  
	Kauthamplanner* problem = new Kauthamplanner( );

	//if (problem->solveProblem()){

	ros::ServiceServer service_plan = n.advertiseService("Request_Plan", &Kauthamplanner::SrvReqPlan, problem);
	ros::ServiceServer service_collision = n.advertiseService("Request_Collision_Check", &Kauthamplanner::SrvCollisionCheck, problem);
	ros::ServiceServer service_move = n.advertiseService("Move_Robots", &Kauthamplanner::SrvMoveRobots, problem);

	ros::ServiceServer service_setup = n.advertiseService("Problem_Setup", &Kauthamplanner::SrvProblemSetup, problem);
	ros::ServiceServer service_forbidden = n.advertiseService("Set_Forbidden", &Kauthamplanner::SrvSetForbidden, problem);
	ros::ServiceServer service_solve_problem = n.advertiseService("Solve_Problem", &Kauthamplanner::SrvSolveProblem, problem);
	ros::ServiceServer service_query = n.advertiseService("Request_Query", &Kauthamplanner::SrvSolveQuery, problem);
	ros::ServiceServer service_attach = n.advertiseService("Attach_Object", &Kauthamplanner::SrvAttachObject, problem);
	ros::ServiceServer service_detach = n.advertiseService("Detach_Object", &Kauthamplanner::SrvDetachObject, problem);



	ROS_INFO("ROS Node Kautham_Planner launched. Waiting for a request.");

	// Example with the SDH gripper and can
	// Move robot to a position


	ros::spin();

	return 0;
}

