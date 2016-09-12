/***************************************************************************
*               Generated by StarUML(tm) C++ Add-In                        *
***************************************************************************/
/***************************************************************************
*                                                                          *
*           Institute of Industrial and Control Engineering                *
*                 Technical University of Catalunya                        *
*                        Barcelona, Spain                                  *
*                                                                          *
*                Project Name:       Kautham Planner                       *
*                                                                          *
*     Copyright (C) 2007 - 2009 by Alexander Pérez and Jan Rosell          *
*            alexander.perez@upc.edu and jan.rosell@upc.edu                *
*                                                                          *
*             This is a motion planning tool to be used into               *
*             academic environment and it's provided without               *
*                     any warranty by the authors.                         *
*                                                                          *
*          Alexander Pérez is also with the Escuela Colombiana             *
*          de Ingeniería "Julio Garavito" placed in Bogotá D.C.            *
*             Colombia.  alexander.perez@escuelaing.edu.co                 *
*                                                                          *
***************************************************************************/
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
 
  
#include <Inventor/Qt/SoQt.h>
#include <QFile>
#include <QString>
#include <QMessageBox>
#include <sstream>
#include <libproblem/ivworkspace.h>
#include <mt/transform.h>

#include <libutil/kauthamdefs.h>
#include "kauthamviewer_teleop.h"

#include "ros/ros.h"


using namespace libProblem;
using namespace libPlanner;

Application::Application() { 
  Q_INIT_RESOURCE(kauthamRes);
  
  initApp();

  timerUpdate = new QTimer(this);
  connect(timerUpdate, SIGNAL(timeout()), this, SLOT(rosUpdate()));
  timerUpdate->start(10);

  it = 0;

  in_loop = false;
}

void Application::initApp(){
  mainWindow = new GUI();
  SoQt::show(mainWindow);
  setActions();
  mainWindow->setText("Welcome to the KAUTHAM VIEWER (with topic!)");
  mainWindow->setText("Open a problem file to start...");
  appState = INITIAL ;
  _problem = NULL;
}

Application::~Application() {

}

void Application::setActions(){
  mainWindow->setAction(FILETOOL,"&Open","CTRL+O",":/icons/fileopen.xpm",this,SLOT(openFile()));
  mainWindow->addSeparator(TOOLBAR);

  mainWindow->setAction(FILETOOL,"&Close","CTRL+Q",":/icons/close.xpm",this,SLOT(closeProblem()));
  mainWindow->addSeparator(TOOLBAR);
  mainWindow->setAction(ACTIONTOOL,"Chan&ge Colour","CTRL+G",
                                ":/icons/determ.xpm", mainWindow, SLOT(changeActiveBackground()));
}

void Application::openFile(){
  QString path,dir;
  QDir workDir;
  mainWindow->setCursor(QCursor(Qt::WaitCursor));

  switch(appState){
    case INITIAL:
      path = QFileDialog::getOpenFileName(  mainWindow,
                                            "Choose a file to open",
                                            workDir.absolutePath(),
                                            "All configuration files (*.xml)");
      if(!path.isEmpty()){
        mainWindow->setText("Kautham is opening a problem file...");
        dir = path;
        dir.truncate(dir.lastIndexOf("/"));
        problemSetup(path.toUtf8().constData());
        stringstream tmp;
        tmp << "Kautham ";
        tmp << MAJOR_VERSION;
        tmp << ".";
        tmp << MINOR_VERSION;
        tmp << " - ";
        tmp << path.toUtf8().constData();
        mainWindow->setWindowTitle( tmp.str().c_str() );
        mainWindow->setText(QString("File: ").append(path).toUtf8().constData() );
	mainWindow->setText("opened successfully.");
      }
      break;
    case PROBLEMLOADED:
      break;
    default:
      break;
  }

  mainWindow->setCursor(QCursor(Qt::ArrowCursor));
}

void Application::closeProblem(){
  mainWindow->setCursor(QCursor(Qt::WaitCursor));
  switch(appState){
  case INITIAL:
    mainWindow->setText("First open a problem");
    break;
  case PROBLEMLOADED:
    mainWindow->restart();
    delete _problem;
    appState = INITIAL;
    break;
  }
  mainWindow->setCursor(QCursor(Qt::ArrowCursor));
}

bool Application::SrvProblemSetup(kautham_ros::ProblemSetup::Request &req,
                     kautham_ros::ProblemSetup::Response &res){

  if (problemSetup( req.problem.data )){
    res.status = true;
    res.dim = _dim;
  }
  else{
    res.status = false;
    res.dim = 0;
  }

  return true;
}

bool Application::problemSetup(string path){
  mainWindow->setCursor(QCursor(Qt::WaitCursor));

  _problem = new Problem();
  if( _problem->setupFromFile( path ) ){
      cout << "The problem file has been loaded successfully.\n";

      // Configuring the problem from information on file
      _dim =  _problem->getDimension();

      mainWindow->addViewerTab("WSpace", libGUI::SPACE, ((IVWorkSpace*)_problem->wSpace())->getIvScene());

      appState = PROBLEMLOADED;
      mainWindow->setCursor(QCursor(Qt::ArrowCursor));

      return true;
  }
  else
    cout << "Problem loading file..." << endl;

return false;
}

void Application::rosUpdate(){

  ros::spinOnce();
  
  if (!in_loop)		in_loop = true;
}

void Application::SubMoveRobots(const trajectory_msgs::JointTrajectoryPoint& msg){
  
  vector<KthReal> pose(_dim);
  
//   cout << "dim: " << _dim << std::endl;
  
//   cout << "Moving to: ";

  for (int i = 0; i < _dim; i++){
    pose[i] = msg.positions[i];
//     cout << msg.positions[i] << " ";
  }

//   cout << endl;

  Sample* smp = new Sample(_dim);
  smp->setCoords( pose );
  _problem->wSpace()->moveRobotsTo( smp );

  //ROS_INFO("Moving Robots...");

//   cout << it++ << endl;
}


bool Application::SrvMoveRobot(kautham_ros::MoveRobot::Request  &req,
                   	   	       kautham_ros::MoveRobot::Response &res){

  //ToDo: generalizar para n robots diferentes

  unsigned Robot0Dim = _problem->wSpace()->getRobot(req.Robot)->getNumControls();
  vector<KthReal> poseR0(Robot0Dim);

  unsigned Robot1Dim = _problem->wSpace()->getRobot(req.Robot)->getNumControls();
  vector<KthReal> poseR1(Robot1Dim);

  vector<KthReal> pose(_dim);

  if ( req.Robot == 0){
    // Pose to robot 0
    for (unsigned i = 0; i < Robot0Dim; i++){
      poseR0[i] = req.Pose.positions[i];
      //cout << req.Pose.positions[i] << " ";
    }
    //Pose to robot 1
    poseR1 = _problem->wSpace()->getRobot( 1 )->getCurrentPos()->getRn().getCoordinates();

  }else{ // req.Robot == 1

    //Pose to robot 0
    poseR0 = _problem->wSpace()->getRobot( 0 )->getCurrentPos()->getRn().getCoordinates();

    // Pose to robot 1
    for (unsigned i = 0; i < Robot1Dim; i++){
      poseR1[i] = req.Pose.positions[i];
      //cout << req.Pose.positions[i] << " ";
    }
  }

  //Total pose
  for (unsigned i = 0;i < Robot0Dim; i++)
	  pose[i] = poseR0[i];
  for (unsigned i = 0;i < Robot1Dim; i++)
	  pose[i+Robot0Dim] = poseR1[i];


  Sample* smp = new Sample(_dim);
  smp->setCoords( pose );
  _problem->wSpace()->moveRobotsTo( smp );

  //ROS_INFO("Moving Robots...");
  return true;
}

bool Application::SrvAttachObject(kautham_ros::AttObj::Request  &req,
                                  kautham_ros::AttObj::Response &res){

  ROS_INFO("SrvAttachObject launched");

  //_problem->wSpace()->getObstacle( req.Obj )->setEnableCollisions( false );

  _problem->wSpace()->getRobot( req.Robot )->attachObject( _problem->wSpace()->getObstacle( req.Obj ),
                                                           _problem->wSpace()->getRobot( req.Robot )->getLink( 7 )->getName() );

  return true;
}

bool Application::SrvDetachObject(kautham_ros::DetObj::Request  &req,
                                  kautham_ros::DetObj::Response &res){

  ROS_INFO("SrvDetachObject launched for Robot: %d", req.Robot);

  _problem->wSpace()->getRobot( req.Robot )->detachObject( _problem->wSpace()->getRobot( req.Robot )->getLink( 7 )->getName() );

  return true;
}

bool Application::SrvCollisionCheck(kautham_ros::ReqCollisionCheck::Request  &req,
                                    kautham_ros::ReqCollisionCheck::Response &res)
{
  ROS_INFO("SrvCollisionCheck launched");

  vector<KthReal> pose;
  pose.resize(_dim,0);

  for (unsigned i = 0; i < _dim; i++)
          pose[i] = req.pose.positions[i];

  Sample* smp = new Sample(_dim);
  smp->setCoords( pose );

  if (_problem->wSpace()->collisionCheck( smp ))
    res.collision = true;
  else{
          //_problem->wSpace()->moveTo( smp );
          res.collision = false;
  }

  return true;
}

bool Application::SrvKViewOn(kautham_ros::kViewOn::Request  &req,
                             kautham_ros::kViewOn::Response &res)
{
  ROS_INFO("Checking if Kautham Viewer is ready");

  
  
  if (in_loop)
    res.kview_on = true;
  else
    res.kview_on = false;

  return true;
}