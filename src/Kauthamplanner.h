/*
 * Kauthamplanner.h
 *
 *  Created on: 02/11/2011
 *      Author: andres
 */

#ifndef KAUTHAMPLANNER_H_
#define KAUTHAMPLANNER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <Inventor/SoDB.h>

#include "libproblem/problem.h"
#include "libplanner/kthquery.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include "kautham_ros/ReqPlan.h"
#include "kautham_ros/ReqCollisionCheck.h"
#include "kautham_ros/ReqQuery.h"
#include "kautham_ros/MoveRobots.h"
#include "kautham_ros/Forbidden.h"
#include "kautham_ros/ProblemSetup.h"
#include "kautham_ros/SolveProblem.h"
#include "kautham_ros/AttObj.h"
#include "kautham_ros/DetObj.h"


#define SDH 0
#define SDH1 3

#define SAH 1

class Kauthamplanner {
public:
	Kauthamplanner();
	Kauthamplanner( string path );
	virtual ~Kauthamplanner();

	bool solveProblem( std::string path );

	Problem* _problem;
	Planner* _planner;
	SampleSet* _samples;
	unsigned int _dim;

	vector<trajectory_msgs::JointTrajectoryPoint> _points;
	vector<trajectory_msgs::JointTrajectoryPoint> Trajectory;

	bool SrvProblemSetup(kautham_ros::ProblemSetup::Request &req,
	                     kautham_ros::ProblemSetup::Response &res);

	bool SrvReqPlan(kautham_ros::ReqPlan::Request  &req,
	                kautham_ros::ReqPlan::Response &res);

	//ToDo kautham_ros::ReqCollisionCheck and kautham_ros::MoveRobots are the same type.
	bool SrvCollisionCheck(kautham_ros::ReqCollisionCheck::Request  &req,
	                       kautham_ros::ReqCollisionCheck::Response &res);

	bool SrvSolveQuery(kautham_ros::ReqQuery::Request &req,
	                   kautham_ros::ReqQuery::Response &res);

	bool SrvMoveRobots(kautham_ros::MoveRobots::Request  &req,
	                   kautham_ros::MoveRobots::Response &res);

	bool SrvSetForbidden(kautham_ros::Forbidden::Request  &req,
	                     kautham_ros::Forbidden::Response &res);

	bool SrvSolveProblem(kautham_ros::SolveProblem::Request &req,
	                     kautham_ros::SolveProblem::Response &res);

	bool SrvAttachObject(kautham_ros::AttObj::Request &req,
	                     kautham_ros::AttObj::Response &res);

	bool SrvDetachObject(kautham_ros::DetObj::Request  &req,
	                     kautham_ros::DetObj::Response &res);

	bool  solveProblem( unsigned Obstgoal );
	bool  problemSetup(string path);

	bool getTransformation( void );
	bool setObstaclesNames( void );
	bool isGraspableObject( int numObject, std::vector<float> Position, Sample &smp );
	bool testGrasp( unsigned Obstgoal );

	bool ArmInverseKinematics( unsigned int Robot, vector<KthReal> &carm );

	struct graspObstacles {
	  std::vector<int> numCollision;
	  std::vector<Sample*> graspSmp;
	  std::vector<float> distance;
	};

	std::vector< graspObstacles > graspInfo;

	unsigned char handType;

	vector<Sample*>* _SolutionPlan;
	vector<unsigned> collobs;

	struct Solution {

	  vector< vector < KthReal > >  SimPath;
	  vector<unsigned> ObsColl;
	  //vector<Sample*>  Path;
	  //vector<Sample*>  SmpColl;

	};

	  std::vector< Solution > SolutionsInfo;

        unsigned      it_path;

        vector<KthReal> init_sample;


};

#endif /* KAUTHAMPLANNER_H_ */
