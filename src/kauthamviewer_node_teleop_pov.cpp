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
 
 

#include "kauthamviewer_teleop_pov.h"
#include <QApplication>
#include <QWidget>
#include <Inventor/Qt/SoQt.h>

#include "ros/ros.h"




int main(int argc, char* argv[]){       
  try{

    QWidget *app = SoQt::init(argv[0]);
    app->setVisible(false);
    
    ros::init(argc, argv, "kautham_viewer_pov_ros ");
    ros::NodeHandle n;

    Application kauthApp;

    ROS_INFO("Kautham Viewer POV launched");

    ros::Subscriber subscriber_move = n.subscribe("Move_Robots", 1000, &Application::SubMoveRobots, &kauthApp);
    ros::Subscriber subscriber_camera = n.subscribe("Set_ActiveCamera_Pose", 1000, &Application::SubActiveCameraPose, &kauthApp);    
//     ros::ServiceServer service_move = n.advertiseService("Move_Robots", &Application::SrvMoveRobots, &kauthApp);
   
    kauthApp.pub_camera_pose = n.advertise<geometry_msgs::PoseStamped>("Current_ActiveCamera_Pose", 10);    
    
    
    ros::ServiceServer service_move_r = n.advertiseService("Move_Robot", &Application::SrvMoveRobot, &kauthApp);
    ros::ServiceServer service_setup = n.advertiseService("Problem_Setup", &Application::SrvProblemSetup, &kauthApp);
    ros::ServiceServer service_attach = n.advertiseService("Attach_Object", &Application::SrvAttachObject, &kauthApp);
    ros::ServiceServer service_detach = n.advertiseService("Detach_Object", &Application::SrvDetachObject, &kauthApp);
    ros::ServiceServer service_collision = n.advertiseService("Request_Collision_Check", &Application::SrvCollisionCheck, &kauthApp);
    ros::ServiceServer service_kview_on = n.advertiseService("Request_KauthamViewer_ON", &Application::SrvKViewOn, &kauthApp);
   
    SoQt::mainLoop();

    return 0;
  }catch(...){
   std::cout << "Unexpected error in the Kautham initialization.\n";
  }
}

