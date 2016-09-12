/*
 * Kauthamplanner.cpp
 *
 *  Created on: 02/11/2011
 *      Author: andres
 */

#include "Kauthamplanner.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <Inventor/SoDB.h>
#include <mt/transform.h>

#include "libproblem/problem.h"
#include "libplanner/kthquery.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include "kautham_ros/ReqPlan.h"
#include "kautham_ros/ReqCollisionCheck.h"
#include "kautham_ros/ReqQuery.h"
#include "kautham_ros/MoveRobots.h"
#include "kautham_ros/ProblemSetup.h"
#include "kautham_ros/Forbidden.h"
#include "kautham_ros/SolveProblem.h"
#include "kautham_ros/AttObj.h"
#include "kautham_ros/DetObj.h"

#include "kautham_ros/empty.h"

Kauthamplanner::Kauthamplanner()
{
  // TODO Auto-generated constructor stub
}

Kauthamplanner::Kauthamplanner(string path)
{

  _problem = new Problem();

  // Opening the problem from file
  if (_problem->setupFromFile(path))
  {
    cout << "The problem file has been loaded successfully.\n";

    // Configuring the problem from information on file
    _planner = _problem->getPlanner();
    _samples = _problem->getSampleSet();
    _dim = _samples->getSampleAt(0)->getDim();

  }
  else
  {
    std::cout << "The problem file has not been loaded successfully. "
        << "Please take care with the problem definition.\n";
  }

  setObstaclesNames();
}

Kauthamplanner::~Kauthamplanner()
{

  delete _problem;
}

bool Kauthamplanner::SrvProblemSetup(kautham_ros::ProblemSetup::Request &req, kautham_ros::ProblemSetup::Response &res)
{

  if (problemSetup(req.problem.data))
    res.status = true;
  else
    res.status = false;

  handType = req.hand;

  setObstaclesNames();

  return true;
}

bool Kauthamplanner::problemSetup(string path)
{

  _problem = new Problem();

  if (_problem->setupFromFile(path))
  {
    cout << "The problem file has been loaded successfully.\n";

    // Configuring the problem from information on file
    _planner = _problem->getPlanner();
    _samples = _problem->getSampleSet();
    _dim = _samples->getSampleAt(0)->getDim();

    return true;
  }
  else
    cout << "Problem loading file..." << endl;

  return false;
}

bool Kauthamplanner::solveProblem(std::string path)
{

  if (_problem != NULL)
  {
    delete _problem;
  }

  _problem = new Problem();

  // Opening the problem from file
  if (_problem->setupFromFile(path))
  {
    cout << "The problem file has been loaded successfully.\n";

    // Configuring the problem from information on file
    _planner = _problem->getPlanner();
    _samples = _problem->getSampleSet();
    _dim = _samples->getSampleAt(0)->getDim();

    vector < KthReal > init(_samples->getSampleAt(0)->getCoords());
    vector < KthReal > goal(_samples->getSampleAt(1)->getCoords());

    _samples->clear();

    Sample* smp = new Sample(_dim);
    smp->setCoords(init);
    _samples->add(smp);
    smp = new Sample(_dim);
    smp->setCoords(goal);
    _samples->add(smp);

    _planner->setInitSamp(_samples->getSampleAt(0));
    _planner->setGoalSamp(_samples->getSampleAt(1));
  }
  else
  {
    std::cout << "The problem file has not been loaded successfully. "
        << "Please take care with the problem definition.\n";
    ROS_INFO(" Error in problem file ");
    return false;
  }

  if (_planner->solveAndInherit())
  {
    //KthQuery& tmp = _planner->getQueries().at( _planner->getQueries().size() - 1 );

    _SolutionPlan = _planner->getSimulationPath();

    vector<double> positions(_dim, 0);

    _points.resize(_SolutionPlan->size());

    for (unsigned j = 0; j < _SolutionPlan->size(); j++)
    {
      //std::cout << "Sample " << j << ": ";
      for (unsigned i = 0; i < _dim; i++)
      {
        positions[i] = _SolutionPlan->at(j)->getCoords().at(i);
        //std::cout << positions[i] << " ";
      }

      //std::cout << std::endl;
      _points[j].positions = positions;
    }
    return true;
  }
  else
    return false;

}

bool Kauthamplanner::SrvReqPlan(kautham_ros::ReqPlan::Request &req, kautham_ros::ReqPlan::Response &res)
{
  ROS_INFO("SrvReqPlan launched for %s", req.problem.data.c_str());

  solveProblem(req.problem.data);

  res.plan.points = _points;

  return true;
}

bool Kauthamplanner::SrvCollisionCheck(kautham_ros::ReqCollisionCheck::Request &req,
                                       kautham_ros::ReqCollisionCheck::Response &res)
{

  //ROS_INFO("SrvCollisionCheck launched. Problem dimension: %d", _dim);
  // Move inactive Robots to Pose for collision check
  if (req.inactiveRobots > 0)
  {

    // ToDo: now just for only one robot

    vector < KthReal > inactiveRobotPose;
    inactiveRobotPose.resize(_dim, 0);

    for (unsigned i = 0; i < req.inactiveRobotsPose.positions.size(); i++)
      inactiveRobotPose[i] = req.inactiveRobotsPose.positions[i];

    _problem->wSpace()->moveObstacleTo(0, inactiveRobotPose);
  }

  vector < KthReal > pose;
  pose.resize(_dim, 0);

  for (unsigned i = 0; i < _dim; i++)
    pose[i] = req.pose.positions[i];

  Sample* smp = new Sample(_dim);
  smp->setCoords(pose);

  if (_problem->wSpace()->collisionCheck(smp))
    res.collision = true;
  else
  {
    //_problem->wSpace()->moveTo( smp );
    res.collision = false;
  }

  return true;
}

/****************/

bool Kauthamplanner::SrvSolveQuery(kautham_ros::ReqQuery::Request &req, kautham_ros::ReqQuery::Response &res)
{
  ROS_INFO("SrvSolveQuery launched");

  _samples->clear();

  vector < KthReal > samp;
  samp.resize(_dim);

  // Copy Init sample from request
  cout << "Init sample: ";
  for (unsigned i = 0; i < _dim; i++)
  {
    samp[i] = req.Init.positions[i];
    cout << samp[i] << " ";
  }
  cout << endl;

  // Set Init Sample to planner
  Sample* smp = new Sample(_dim);
  smp->setCoords(samp);

  /***************/
  cout << "Index of sample init: " << _samples->indexOf(smp) << endl;
  if (_samples->indexOf(smp) == (unsigned)_samples->getSize())
  {
    _samples->add(smp);

    cout << "Sample: ";
    for (unsigned s = 0; s < smp->getCoords().size(); s++)
    {
      cout << smp->getCoords().at(s) << " ";
    }
    cout << endl;

    //sampleschanged = true;
  }
  /***************/

  //_samples->add( smp );
  _planner->setInitSamp(smp);

  // Copy goal sample from request
  cout << "Goal sample: ";
  for (unsigned i = 0; i < _dim; i++)
  {
    samp[i] = req.Goal.positions[i];
    cout << samp[i] << " ";
  }
  cout << endl;

  // Set goal sample to planner
  smp = new Sample(_dim);
  smp->setCoords(samp);

  /*****************/
  cout << "Index of sample goal: " << _samples->indexOf(smp) << endl;
  if (_samples->indexOf(smp) == (unsigned)_samples->getSize())
  {
    _samples->add(smp);

    cout << "Sample: ";
    for (unsigned s = 0; s < smp->getCoords().size(); s++)
    {
      cout << smp->getCoords().at(s) << " ";
    }
    cout << endl;
    //sampleschanged = true;
  }
  /*****************/

  //_samples->add( smp );
  _planner->setGoalSamp(smp);

  // Call to trysolve
  if (_planner->solveAndInherit())
  {

    // Copy simulation plan solution
    _SolutionPlan = _planner->getSimulationPath();

    cout << "Plan for Query with " << _SolutionPlan->size() << " points" << endl;
    _points.resize(_SolutionPlan->size());

    vector<double> positions(_dim, 0);

    // Copy all the points in solution
    for (unsigned j = 0; j < _SolutionPlan->size(); j++)
    {
      for (unsigned i = 0; i < _dim; i++)
      {
        positions[i] = _SolutionPlan->at(j)->getCoords().at(i);
      }
      _points[j].positions = positions;
    }

    // Set response to service call
    res.plan.points = _points;
    //res.status = true;
    return true;

  }
  else
  {
    ROS_INFO("Plan without solution...");
    //res.status=false;
  }

  return false;

}

/*********************/

/*bool Kauthamplanner::SrvSolveQuery(kautham_ros::ReqQuery::Request &req,
 kautham_ros::ReqQuery::Response &res)
 {
 ROS_INFO("SrvSolveQuery launched");

 _samples->clear();

 vector<KthReal> samp;
 samp.resize(_dim);

 // Copy Init sample from request
 cout << "Init sample: ";
 for (unsigned i = 0; i < _dim; i++){
 samp[i] = req.Init.positions[i];
 cout << samp[i] << " ";
 }
 cout << endl;

 // Set Init Sample to planner
 Sample* smp = new Sample(_dim);
 smp->setCoords( samp );
 _samples->add( smp );
 _planner->setInitSamp( smp );

 // Copy goal sample from request
 cout << "Goal sample: ";
 for (unsigned i = 0; i < _dim; i++){
 samp[i] = req.Goal.positions[i];
 cout << samp[i] << " ";
 }
 cout << endl;

 // Set goal sample to planner
 smp = new Sample(_dim);
 smp->setCoords( samp );
 _samples->add( smp );
 _planner->setGoalSamp( smp );

 // Call to trysolve
 if(_planner->solveAndInherit()){

 // Copy simulation plan solution
 _SolutionPlan = _planner->getSimulationPath();

 cout << "Plan for Query with " << _SolutionPlan->size() << " points" << endl;
 _points.resize(_SolutionPlan->size());

 vector<double> positions(_dim, 0);

 // Copy all the points in solution
 for ( unsigned j = 0; j < _SolutionPlan->size() ; j++ ){
 for ( unsigned i = 0; i < _dim; i++ ){
 positions[i] = _SolutionPlan->at(j)->getCoords().at(i);
 }
 _points[j].positions = positions;
 }

 // Set response to service call
 res.plan.points = _points;

 return true;
 }
 else{
 ROS_INFO("Plan without solution...");
 }

 return false;

 }
 */
bool Kauthamplanner::SrvMoveRobots(kautham_ros::MoveRobots::Request &req, kautham_ros::MoveRobots::Response &res)
{

  vector < KthReal > pose(_dim);

  for (unsigned i = 0; i < _dim; i++)
  {
    pose[i] = req.Pose.positions[i];
  }

  Sample* smp = new Sample(_dim);
  smp->setCoords(pose);
  _problem->wSpace()->moveRobotsTo(smp);

  //getTransformation();

  //ROS_INFO("Moving Robots...");
  return true;
}

bool Kauthamplanner::getTransformation()
{

  cout << _problem->wSpace()->getRobot(0)->getHomeTransform();

  return true;
}

bool Kauthamplanner::setObstaclesNames(void)
{

  // Configure the obstacles names. Necessary fir Inverse Kinematic.
  unsigned int itObst = 0;

  //_problem->wSpace()->_forbiddenObstacles.push_back(itObst);
  _problem->wSpace()->getObstacle(itObst++)->setName("ioc");
  //_problem->wSpace()->_forbiddenObstacles.push_back(itObst);
  _problem->wSpace()->getObstacle(itObst++)->setName("table1");
  _problem->wSpace()->getObstacle(itObst++)->setName("box");
  //_problem->wSpace()->_forbiddenObstacles.push_back(itObst);
  //_problem->wSpace()->getObstacle(itObst++)->setName("table2");
  //_problem->wSpace()->_forbiddenObstacles.push_back(itObst);
  //_problem->wSpace()->getObstacle(itObst++)->setName("table3");
  //_problem->wSpace()->_forbiddenObstacles.push_back(itObst);
  _problem->wSpace()->getObstacle(itObst++)->setName("cangoal");
  _problem->wSpace()->getObstacle(itObst++)->setName("topBox");

  // Another obstacles...
  for (;itObst < _problem->wSpace()->obstaclesCount(); itObst++)
    _problem->wSpace()->getObstacle(itObst)->setName("can");

  return true;
}

bool Kauthamplanner::ArmInverseKinematics(unsigned int Robot, vector<KthReal> &carm)
{
  RobConf rc;

  try
  {
    rc = _problem->wSpace()->getRobot(Robot)->InverseKinematics(carm);
  }
  catch (InvKinEx &ex)
  {
    //std::cout << ex.what() << std::endl;
    return false;
  }

  //load the six joint values of the arm
  for (int k = 0; k < 6; k++)
    carm[k] = rc.getRn().getCoordinate(k);

  KthReal low[6];
  KthReal high[6];
  for (int k = 0; k < 6; k++)
  {
    //normalize
    low[k] = *_problem->wSpace()->getRobot(Robot)->getLink(k + 1)->getLimits(true);
    high[k] = *_problem->wSpace()->getRobot(Robot)->getLink(k + 1)->getLimits(false);
    carm[k] = floorf((carm[k] - low[k]) / (high[k] - low[k]) * 1000) / 1000;
  }

  vector < KthReal > cords(7);
  for (unsigned i = 0; i < 6; i++)
    cords[i] = carm[i];

  if (handType == SAH)
  {
    cords[6] = 0.0;
    cords[7] = 0.757;
  }
  else
  {
    cords[6] = 0.200;
    cords[7] = 0.810;
  }

  //ToDo: Collision with Non movable Obstacles
  Sample* smp = new Sample(_dim);
  smp->setCoords(cords);

  cout << "Muestra: ";
  for (unsigned i = 0; i < _dim; i++)
    cout << smp->getCoords().at(i) << " ";
  cout << endl;

  if (_problem->wSpace()->collisionCheckHard(smp))
  {
    //cout << "Sample in collision with no movable obstacle" << endl;
    return false;
  }

  return true;
}

bool Kauthamplanner::isGraspableObject(int numObject, std::vector<float> Position, Sample &smp)
{

  float *goalPosition = _problem->wSpace()->getObstacle(numObject)->getElement()->getPosition();
  float *goalOrientation = _problem->wSpace()->getObstacle(numObject)->getElement()->getOrientation();

  // Obtain the transformation relative to Object
  mt::Point3 pGoal(goalPosition);
  mt::Rotation rGoal(goalOrientation);
  mt::Transform T1(rGoal, pGoal);

  mt::Unit3 axis(Position.at(0), Position.at(1), Position.at(2));
  mt::Scalar angle = Position.at(3);
  mt::Rotation rot(axis, angle);
  mt::Point3 tr(Position.at(4), Position.at(5), Position.at(6));

  mt::Transform T2(rot, tr);

  mt::Transform Tfinal = T1 * T2;

  vector < KthReal > cords(7);

  mt::Rotation nrot = Tfinal.getRotation();
  mt::Point3 ntr = Tfinal.getTranslation();

  cords[0] = ntr.at(0);
  cords[1] = ntr.at(1);
  cords[2] = ntr.at(2);
  cords[3] = nrot.at(0);
  cords[4] = nrot.at(1);
  cords[5] = nrot.at(2);
  cords[6] = nrot.at(3);

  bool invKinSolved = ArmInverseKinematics(0, cords);

  cords.resize(_dim);
  if (handType == SAH)
  {
    cords[6] = 0.0;
    cords[7] = 0.757;
  }
  else
  {
    cords[6] = 0.200;
    cords[7] = 0.810;
  }

  if (invKinSolved == true)
  {

    // Testing code, to visualize the robot. Must be removed in the code
    Sample* tsmp = new Sample(_dim);
    tsmp->setCoords(cords);
    _problem->wSpace()->moveRobotsTo(tsmp);

    smp = tsmp;

    return true;
  }

  return false;
}

bool Kauthamplanner::testGrasp(unsigned Obstgoal)
{

  int maxObst = _problem->wSpace()->obstaclesCount();
  //cout << "Total de obstaculos: " << maxObst << endl;
  graspInfo.clear();
  graspInfo.resize(maxObst);

  vector<float> grasp;
  grasp.resize(7, 0.0);
  unsigned numCol = 0;
  KthReal distToInit;

  Sample * Init = new Sample(_dim);
  Init->setCoords(_samples->getSampleAt(0)->getCoords());

  //for (int it = 0; it < maxObst; it++){
  unsigned it = Obstgoal;
  numCol = 0;
  Obstacle* Obst = _problem->wSpace()->getObstacle(it);

  if (Obst->getName() == "cangoal" || Obst->getName() == "can")
  {

    if (handType == SDH /*|| handType == SDH1*/)
    {
      // fill the grasp relative position
      grasp.at(0) = 0.0;
      grasp.at(1) = 1.0;
      grasp.at(2) = 0.0;
      grasp.at(3) = 0.0;
      grasp.at(4) = 10.0;
      grasp.at(5) = 15.0;
      grasp.at(6) = -200.0;
      Sample* grasp_smp = new Sample(_dim);

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.0;
      grasp.at(1) = 1.0;
      grasp.at(2) = 0.0;
      grasp.at(3) = 0.79;
      grasp.at(4) = -133.0;
      grasp.at(5) = 15.0;
      grasp.at(6) = -147.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.0;
      grasp.at(1) = 1.0;
      grasp.at(2) = 0.0;
      grasp.at(3) = 1.57;
      grasp.at(4) = -200.0;
      grasp.at(5) = 15.0;
      grasp.at(6) = -10.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.0;
      grasp.at(1) = 1.0;
      grasp.at(2) = 0.0;
      grasp.at(3) = 2.36;
      grasp.at(4) = -140.0;
      grasp.at(5) = 15.0;
      grasp.at(6) = 133.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.0;
      grasp.at(1) = 1.0;
      grasp.at(2) = 0.0;
      grasp.at(3) = 3.1415;
      grasp.at(4) = -10.0;
      grasp.at(5) = 15.0;
      grasp.at(6) = 200.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.0;
      grasp.at(1) = 1.0;
      grasp.at(2) = 0.0;
      grasp.at(3) = 3.93;
      grasp.at(4) = 133.0;
      grasp.at(5) = 15.0;
      grasp.at(6) = 147.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.0;
      grasp.at(1) = 1.0;
      grasp.at(2) = 0.0;
      grasp.at(3) = 4.71;
      grasp.at(4) = 200.0;
      grasp.at(5) = 15.0;
      grasp.at(6) = 10.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.0;
      grasp.at(1) = 1.0;
      grasp.at(2) = 0.0;
      grasp.at(3) = 5.5;
      grasp.at(4) = 147.0;
      grasp.at(5) = 15.0;
      grasp.at(6) = -133.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position Topside grasp
      grasp.at(0) = 1.0;
      grasp.at(1) = 0.0;
      grasp.at(2) = 0.0;
      grasp.at(3) = 1.57;
      grasp.at(4) = 10.0;
      grasp.at(5) = 230.0;
      grasp.at(6) = 0.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        //cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }
      //
    }
  }
  if (Obst->getName() == "topBox")
  {
    if (handType == SDH /*|| handType == SDH1*/)
    {
      // fill the grasp relative position
      grasp.at(0) = 0.7;
      grasp.at(1) = 0.7;
      grasp.at(2) = 0.0;
      grasp.at(3) = 3.1415;
      grasp.at(4) = 160.0;
      grasp.at(5) = 185.0;
      grasp.at(6) = 320.0;
      Sample* grasp_smp = new Sample(_dim);

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.7;
      grasp.at(1) = 0.7;
      grasp.at(2) = 0.0;
      grasp.at(3) = 3.1415;
      grasp.at(4) = 150.0;
      grasp.at(5) = 185.0;
      grasp.at(6) = 320.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.7;
      grasp.at(1) = 0.7;
      grasp.at(2) = 0.0;
      grasp.at(3) = 3.1415;
      grasp.at(4) = 170.0;
      grasp.at(5) = 185.0;
      grasp.at(6) = 320.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.7;
      grasp.at(1) = -0.7;
      grasp.at(2) = 0.0;
      grasp.at(3) = 3.1415;
      grasp.at(4) = 160.0;
      grasp.at(5) = 165.0;
      grasp.at(6) = 320.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.7;
      grasp.at(1) = -0.7;
      grasp.at(2) = 0.0;
      grasp.at(3) = 3.1415;
      grasp.at(4) = 150.0;
      grasp.at(5) = 165.0;
      grasp.at(6) = 320.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      // fill the grasp relative position
      grasp.at(0) = 0.7;
      grasp.at(1) = -0.7;
      grasp.at(2) = 0.0;
      grasp.at(3) = 3.1415;
      grasp.at(4) = 170.0;
      grasp.at(5) = 165.0;
      grasp.at(6) = 320.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }
    }
    else if (handType == SAH)
    {
      // fill the grasp relative position
      grasp.at(0) = 0.72;
      grasp.at(1) = -0.69;
      grasp.at(2) = 0.08;
      grasp.at(3) = 3.414;
      grasp.at(4) = 100.0;
      grasp.at(5) = 180.0;
      grasp.at(6) = 327.0;
      Sample* grasp_smp = new Sample(_dim);

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      grasp.at(0) = 0.72;
      grasp.at(1) = -0.69;
      grasp.at(2) = 0.08;
      grasp.at(3) = 3.414;
      grasp.at(4) = 150.0;
      grasp.at(5) = 180.0;
      grasp.at(6) = 327.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      grasp.at(0) = 0.72;
      grasp.at(1) = -0.69;
      grasp.at(2) = 0.08;
      grasp.at(3) = 3.414;
      grasp.at(4) = 200.0;
      grasp.at(5) = 180.0;
      grasp.at(6) = 327.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }
      grasp.at(0) = 0.72;
      grasp.at(1) = -0.69;
      grasp.at(2) = 0.08;
      grasp.at(3) = 3.414;
      grasp.at(4) = 250.0;
      grasp.at(5) = 180.0;
      grasp.at(6) = 327.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      grasp.at(0) = 0.700;
      grasp.at(1) = 0.700;
      grasp.at(2) = -0.125;
      grasp.at(3) = 2.900;
      grasp.at(4) = 100.0;
      grasp.at(5) = 185.0;
      grasp.at(6) = 325.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }

      grasp.at(0) = 0.700;
      grasp.at(1) = 0.700;
      grasp.at(2) = -0.125;
      grasp.at(3) = 2.900;
      grasp.at(4) = 150.0;
      grasp.at(5) = 185.0;
      grasp.at(6) = 325.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }
      grasp.at(0) = 0.700;
      grasp.at(1) = 0.700;
      grasp.at(2) = -0.125;
      grasp.at(3) = 2.900;
      grasp.at(4) = 200.0;
      grasp.at(5) = 185.0;
      grasp.at(6) = 325.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }
      grasp.at(0) = 0.700;
      grasp.at(1) = 0.700;
      grasp.at(2) = -0.125;
      grasp.at(3) = 2.900;
      grasp.at(4) = 250.0;
      grasp.at(5) = 185.0;
      grasp.at(6) = 325.0;

      if (isGraspableObject(it, grasp, *grasp_smp))
      {
        numCol = _problem->wSpace()->collisionCheckCount(grasp_smp);
        graspInfo.at(it).graspSmp.push_back(grasp_smp);
        graspInfo.at(it).numCollision.push_back(numCol);
        cout << "Obstacle: " << it << ", Collisions:  " << numCol << endl;
        distToInit = _problem->wSpace()->distanceBetweenSamples(*grasp_smp, *Init, CONFIGSPACE);
        graspInfo.at(it).distance.push_back(distToInit);
      }
    }
  }

  cout << "Obstacle: " << _problem->wSpace()->getObstacle(it)->getName() << ", Samples:  "
      << graspInfo.at(it).graspSmp.size() << endl;
  //}

  // Second criterion. First Minimum of collision sample.
  // Arrange information of graspInfo.at(goal)

  Sample* tmp_smp = new Sample(_dim);
  int tmp_col;

  for (unsigned i = 1; i < graspInfo.at(Obstgoal).graspSmp.size(); i++)
  {
    for (unsigned j = 0; j < graspInfo.at(Obstgoal).graspSmp.size() - i; j++)
    {
      if (graspInfo.at(Obstgoal).numCollision.at(j) > graspInfo.at(Obstgoal).numCollision.at(j + 1))
      {

        tmp_smp = graspInfo.at(Obstgoal).graspSmp.at(j);
        graspInfo.at(Obstgoal).graspSmp.at(j) = graspInfo.at(Obstgoal).graspSmp.at(j + 1);
        graspInfo.at(Obstgoal).graspSmp.at(j + 1) = tmp_smp;

        tmp_col = graspInfo.at(Obstgoal).numCollision.at(j);
        graspInfo.at(Obstgoal).numCollision.at(j) = graspInfo.at(Obstgoal).numCollision.at(j + 1);
        graspInfo.at(Obstgoal).numCollision.at(j + 1) = tmp_col;
      }
    }
  }

  // Print ordered vector
  /* for (int it = 0; it < graspInfo.at(2).graspSmp.size(); it++)
   cout << "Sample " << it << " has " << graspInfo.at(2).numCollision.at(it) << " Collisions" << endl;

   // Choose the best goal Sample.
   cout << "Moving to Sample Goal" << endl;
   _problem->wSpace()->moveRobotsTo( graspInfo.at(2).graspSmp.at( 0 ) );*/

  return true;

}

bool Kauthamplanner::solveProblem(unsigned Obstgoal)
{

  bool _solved = false;
  bool collwithGoal = false;
  bool noSolution = false;
  bool found = false;
  SolutionsInfo.clear();

  vector < KthReal > init = _samples->getSampleAt(0)->getCoords();

  do
  {

    _samples->clear();

    // Set initial sample from file xml information
    Sample* smp = new Sample(_dim);
    smp->setCoords(init);
    _samples->add(smp);
    _planner->setInitSamp(smp);

    cout << "**** Numero de formas de sujetar el objeto: " << graspInfo.at(Obstgoal).graspSmp.size() << " ****" << endl;

    // Search for possible solutions for every grasp
    for (unsigned itSol = 0; itSol < graspInfo.at(Obstgoal).graspSmp.size(); itSol++)
    {

      Solution solTmp;

      // Set goal sample from grasp information
      vector < KthReal > goal(graspInfo.at(Obstgoal).graspSmp.at(itSol)->getCoords());
      smp = new Sample(_dim);
      smp->setCoords(goal);
      _samples->add(smp);
      _planner->setGoalSamp(smp);

      cout << "**** Calculando el plan de IDA ****" << endl;
      // If exist a solution
      if (_planner->solveAndInherit())
      {

        found = true;

        // Take the simulation path
        solTmp.SimPath.resize(_planner->getSimulationPath()->size());

        // For every sample on simulation path
        for (unsigned i = 0; i < _planner->getSimulationPath()->size(); i++)
        {

          collwithGoal = false;

          // Take every dof
          solTmp.SimPath.at(i).resize(_dim);
          for (unsigned j = 0; j < _dim; j++)
          {
            solTmp.SimPath.at(i).at(j) = _planner->getSimulationPath()->at(i)->getCoords().at(j);
          }

          // Delete vector with obstacles information
          collobs.resize(0);

          // Check collisions in the sample
          if (_problem->wSpace()->collisionCheckObstacles(_planner->getSimulationPath()->at(i), collobs))
          {

            // For every obstacle in collision with the sample
            for (unsigned k = 0; k < collobs.size(); k++)
            {
              bool isDiff = true;

              if (collobs.at(k) == Obstgoal)
              {
                collwithGoal = true;
                break; // Dont check more collisions
              }

              // Simulation path do not have Obstacles yet
              if (solTmp.ObsColl.size() == 0)
              {
                solTmp.ObsColl.push_back(collobs.at(k));
                isDiff = false;
              }
              else
              {
                // For every obstacle in simulation path obstacle vector
                for (unsigned m = 0; m < solTmp.ObsColl.size(); m++)
                {
                  if (collobs.at(k) == solTmp.ObsColl.at(m))
                  {
                    // Obstacle is in simulation path obstacles vector
                    isDiff = false;
                    break;
                  }
                }
              }
              if (isDiff)
                solTmp.ObsColl.push_back(collobs.at(k));
            }

            // If collwithGoal dont check collisions for other samples on simulation path
            if (collwithGoal)
              break;
          } // endif check collision
        } // endfor every sample

        // If there is a solution without collisions
        if ((solTmp.ObsColl.size() == 0) && !collwithGoal)
        {
          _solved = true;
        }

        cout << "*************************************** " << itSol << endl;

        // Store Solution
        if (!collwithGoal)
          SolutionsInfo.push_back(solTmp);

      } //endif solveAndInherit()

      else
      {
        found = false;
        if (SolutionsInfo.size() == 0)
          noSolution = true;
      }

      if (_solved)
      {
        break; // Finded a path without collisions
      }

      if (noSolution)
      {
        _samples->clear();

        // Set initial sample from file xml information
        Sample* smp = new Sample(_dim);
        smp->setCoords(init);
        _samples->add(smp);
        _planner->setInitSamp(_samples->getSampleAt(0));
      }

    } // endfor Solution for every grasp

    cout << "Numero de Soluciones encontradas: " << SolutionsInfo.size() << endl;

    //------
    if (SolutionsInfo.size() == 0)
    {
      cout << "Sin Solucion" << endl;
      // noSolution = true;
      break;
    }

    //---
    // Organizar las soluciones por ObsColl.size()
    Solution tmp_Sol;
    if (SolutionsInfo.size() > 1)
    {
      for (unsigned i = 1; i < SolutionsInfo.size(); i++)
      {
        for (unsigned j = 0; j < SolutionsInfo.size() - i; j++)
        {
          if (SolutionsInfo.at(j).ObsColl.size() > SolutionsInfo.at(j + 1).ObsColl.size())
          {

            tmp_Sol = SolutionsInfo.at(j);
            SolutionsInfo.at(j) = SolutionsInfo.at(j + 1);
            SolutionsInfo.at(j + 1) = tmp_Sol;
          }
        }
      }
    }

    if (SolutionsInfo.size() > 0)
      _solved = true;
    else
      _solved = false;

  } while (!_solved);

  if (SolutionsInfo.size() == 0)
  {
    cout << "retornando Falso no hay solucion" << endl;
    return false;
  }
  else
  {
    cout << "La trayectoria tiene : " << SolutionsInfo.at(0).SimPath.size() << " posiciones" << endl;
    return true;
  }
}

bool Kauthamplanner::SrvSetForbidden(kautham_ros::Forbidden::Request &req, kautham_ros::Forbidden::Response &res)
{

  ROS_INFO("SrvSeForbidden launched");

  for (unsigned i = 0; i < req.forbiddens.size(); i++)
  {
    bool insert = true;
    for (unsigned j = 0; j < _problem->wSpace()->_forbiddenObstacles.size(); j++)
    {
      if (_problem->wSpace()->_forbiddenObstacles.at(j) == req.forbiddens.at(i))
      {
        insert = false;
      }
    }

    if (insert)
      _problem->wSpace()->_forbiddenObstacles.push_back(req.forbiddens.at(i));
  }

  return true;
}

bool Kauthamplanner::SrvSolveProblem(kautham_ros::SolveProblem::Request &req, kautham_ros::SolveProblem::Response &res)
{

  ROS_INFO("SrvSolveProblem launched for goal %d", req.obstGoal);

  testGrasp(req.obstGoal);

  if (solveProblem(req.obstGoal))
  {

    _points.resize(SolutionsInfo.at(0).SimPath.size());
    for (unsigned i = 0; i < SolutionsInfo.at(0).SimPath.size(); i++)
    {
      _points.at(i).positions.resize(_dim);
      for (unsigned j = 0; j < _dim; j++)
      {
        _points.at(i).positions.at(j) = SolutionsInfo.at(0).SimPath.at(i).at(j);
      }
    }

    res.SimPlan.points = _points;

    res.ObstColl = SolutionsInfo.at(0).ObsColl;

    return true;
  }
  else
    return false;
}

bool Kauthamplanner::SrvAttachObject(kautham_ros::AttObj::Request &req, kautham_ros::AttObj::Response &res)
{

  ROS_INFO("SrvAttachObject launched for Robot %d", req.Robot);

  Obstacle* Obst = _problem->wSpace()->getObstacle(req.Obj);

  cout << "Obstacle attached " << Obst->getName() << endl;

  _problem->wSpace()->getRobot(req.Robot)->attachObject(Obst,
                                                        _problem->wSpace()->getRobot(req.Robot)->getLink(7)->getName());
  //Obst->setEnableCollisions( false );

  return true;
}

bool Kauthamplanner::SrvDetachObject(kautham_ros::DetObj::Request &req, kautham_ros::DetObj::Response &res)
{

  ROS_INFO("SrvDetachObject launched for Robot %d", req.Robot);

  if (_problem->wSpace()->getRobot(req.Robot)->detachObject(
      _problem->wSpace()->getRobot(req.Robot)->getLink(7)->getName()))
  {
    ROS_INFO("SrvDetachObject DONE!!!");
    res.status = true;
  }
  else
  {
    res.status = false;
    ROS_INFO("SrvDetachObject ERROR!!!");
  }
  return true;
}

// JointTrajectory.msg
/*
 *  Header header
 uint32 seq
 time stamp
 string frame_id
 string[] joint_names
 JointTrajectoryPoint[] points
 float64[] positions
 float64[] velocities
 float64[] accelerations
 duration time_from_start
 */
