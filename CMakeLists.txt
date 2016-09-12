cmake_minimum_required(VERSION 2.8.3)
project(kautham_ros)

# nomes compila si esta a camaleon
site_name(HOSTNAME)
MESSAGE("El nom del host es ${HOSTNAME}")

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs trajectory_msgs sensor_msgs std_msgs message_generation)


if (NOT ${HOSTNAME} MATCHES "vm_ubuntu")
if (NOT ${HOSTNAME} MATCHES "bmm1")

OPTION( KAUTHAM_PLANNER "Build the console planner version" ON )
OPTION( KAUTHAM_VIEWER  "Build the viewer gui version" ON )
OPTION( KAUTHAM_VIEWER_TOPIC  "Build the viewer gui version with topic" ON )
OPTION( KAUTHAM_VIEWER_POV_TOPIC  "Build the viewer gui version with topic and dynamic Point Of View" ON )


SET(CMAKE_CXX_FLAGS -fPIC)


SET(hardlab_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hardlab)
SET(mt_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libmt)
SET(problem_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libproblem)
SET(problem_pov_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libproblem_pov)
# SET(planner_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libplanner)
SET(util_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libutil)
SET(ann_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libann)


SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/modules)


# Setup BOOST library
FIND_PACKAGE( Boost COMPONENTS  system serialization thread REQUIRED )
IF(Boost_FOUND)
  MESSAGE(STATUS "Boost library found at first attempt.")
ELSE(Boost_FOUND)
  SET( BOOST_ROOT $ENV{BOOSTDIR})
	MESSAGE(STATUS "Boost library NOT found at first attempt. Setting BOOST_ROOT to" $ENV{BOOSTDIR})
	FIND_PACKAGE( Boost COMPONENTS  system serialization thread REQUIRED )
	IF(Boost_FOUND)
		MESSAGE(STATUS "Boost library found in " ${BOOST_ROOT})
        INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
        LINK_DIRECTORIES(${Boost_LIBRARY_DIRS} )
	ELSE(Boost_FOUND)
		MESSAGE(STATUS "Boost library not found. Boost-dependent headers will NOT be available")
	ENDIF(Boost_FOUND)
ENDIF(Boost_FOUND)
FIND_PACKAGE( mt REQUIRED )
FIND_PACKAGE( Coin REQUIRED )
FIND_PACKAGE( GSL )


ADD_SUBDIRECTORY ( libann )
ADD_SUBDIRECTORY ( libutil )
ADD_SUBDIRECTORY ( libpqp )
ADD_SUBDIRECTORY ( libproblem )
ADD_SUBDIRECTORY ( libproblem_pov )
ADD_SUBDIRECTORY ( libsampling )
ADD_SUBDIRECTORY ( libplanner )
IF( KAUTHAM_VIEWER OR KAUTHAM_VIEWER_TOPIC)
  FIND_PACKAGE( Qt4 REQUIRED )
  IF( QT4_FOUND )
    SET( QT_USE_QTMAIN TRUE )
    INCLUDE ( ${QT_USE_FILE} )
    ADD_DEFINITIONS( ${QT_DEFINITIONS} )
  ENDIF( QT4_FOUND )

  FIND_PACKAGE( SoQt4 REQUIRED )
  IF( SOQT_LIBRARY_FOUND )
    ADD_DEFINITIONS( -DSOQT_DLL -DCOIN_DLL )
    INCLUDE_DIRECTORIES( ${SOQT_INCLUDE_DIR} )
  ENDIF( SOQT_LIBRARY_FOUND )

  INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR} 
                       ${mt_DIR}
                       ${Boost_INCLUDE_DIRS})
                       
  LINK_DIRECTORIES(${Boost_LIBRARY_DIRS} )
  
  ADD_SUBDIRECTORY ( ${CMAKE_CURRENT_SOURCE_DIR}/libgui )
    
  FILE( GLOB RES_FILES *.rc )
  QT4_WRAP_CPP( QT_MOC_SRC src/application.h )
ENDIF(  KAUTHAM_VIEWER OR KAUTHAM_VIEWER_TOPIC )

endif (NOT ${HOSTNAME} MATCHES "bmm1")


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  AttObj.srv 
  LoadData.srv 
  ReqPlan.srv 
  DetObj.srv 
  MoveRobot.srv 
  ReqQuery.srv 
  empty.srv 
  MoveRobots.srv 
  SaveData.srv 
  EraseObj.srv
  ProblemSetup.srv 
  SolveProblem.srv 
  Forbidden.srv 
  ReqCollisionCheck.srv
  kViewOn.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  std_msgs
)



###################################
## catkin specific configuration ##
###################################


if (NOT ${HOSTNAME} MATCHES "bmm1")

catkin_package(
  INCLUDE_DIRS libutil ${CMAKE_CURRENT_SOURCE_DIR} ${mt_DIR} ${COIN_INCLUDE_DIR}
  CATKIN_DEPENDS message_runtime geometry_msgs trajectory_msgs sensor_msgs std_msgs
)

else  (NOT ${HOSTNAME} MATCHES "bmm1")

catkin_package(
  INCLUDE_DIRS libutil ${CMAKE_CURRENT_SOURCE_DIR} ${mt_DIR}
  CATKIN_DEPENDS message_runtime geometry_msgs trajectory_msgs sensor_msgs std_msgs
)

endif (NOT ${HOSTNAME} MATCHES "bmm1")


INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS})



if (NOT ${HOSTNAME} MATCHES "bmm1")

###########
## Build ##
###########

#Making Kauthan_Planner
IF( KAUTHAM_PLANNER )
  SET( SRC_FILES src/kauthamplanner_node.cpp src/KauthamplannerRealScene.cpp)
 
  ADD_EXECUTABLE( Kautham_Planner ${SRC_FILES} ${HDR_FILES} )

  TARGET_LINK_LIBRARIES( Kautham_Planner ${catkin_LIBRARIES} libDNN libPQP libproblem libkthutil )
ENDIF ( KAUTHAM_PLANNER )

#Making Kautham Viewer with service
IF( KAUTHAM_VIEWER )
  SET( SRC_FILES_GUI src/kauthamviewer_node.cpp src/kauthamviewer.cpp)
  SET( HDR_FILES_GUI src/kauthamviewer.h )
  
  ADD_EXECUTABLE( Kautham_Viewer ${SRC_FILES_GUI} ${HDR_FILES_GUI} ${QT_MOC_SRC} ${RES_FILES} )

  TARGET_LINK_LIBRARIES( Kautham_Viewer ${catkin_LIBRARIES} libDNN libPQP libgui libsampling libproblem libkthutil )
ENDIF( KAUTHAM_VIEWER )

#Making Kautham Viewer with topic
IF( KAUTHAM_VIEWER_TOPIC )
  SET( SRC_FILES_GUI src/kauthamviewer_node_teleop.cpp src/kauthamviewer_teleop.cpp)
  SET( HDR_FILES_GUI src/kauthamviewer_teleop.h )
  
  ADD_EXECUTABLE( Kautham_Viewer_Topic ${SRC_FILES_GUI} ${HDR_FILES_GUI} ${QT_MOC_SRC} ${RES_FILES} )

  TARGET_LINK_LIBRARIES( Kautham_Viewer_Topic ${catkin_LIBRARIES} libDNN libPQP libgui libsampling libproblem libkthutil )
ENDIF( KAUTHAM_VIEWER_TOPIC )


#Making Kautham Viewer with topic and dynamic Point Of View
IF( KAUTHAM_VIEWER_POV_TOPIC )
  include_directories(/usr/include/eigen3)
  # include_directories(/usr/include/eigen3/Eigen/src/Core)
  include_directories(/usr/include/eigen3/Eigen/src/Geometry)

  SET( SRC_FILES_GUI src/kauthamviewer_node_teleop_pov.cpp src/kauthamviewer_teleop_pov.cpp)
  SET( HDR_FILES_GUI src/kauthamviewer_teleop_pov.h )
  
  ADD_EXECUTABLE( Kautham_Viewer_Pov_Topic ${SRC_FILES_GUI} ${HDR_FILES_GUI} ${QT_MOC_SRC} ${RES_FILES} )

  TARGET_LINK_LIBRARIES( Kautham_Viewer_Pov_Topic ${catkin_LIBRARIES} libDNN libPQP libgui libsampling libproblem_pov libkthutil )
ENDIF( KAUTHAM_VIEWER_POV_TOPIC )


#############
## Install ##
#############

set(KAUTHAMROS_TARGETS 
      Kautham_Planner
      Kautham_Viewer
      Kautham_Viewer_Topic
      Kautham_Viewer_Pov_Topic
      # Libraries
#       libDNN
#       libPQP
#       libgui
#       libsampling
#       libproblem_pov
#       libkthutil
      )

install(TARGETS ${KAUTHAMROS_TARGETS} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



endif (NOT ${HOSTNAME} MATCHES "bmm1")
endif (NOT ${HOSTNAME} MATCHES "vm_ubuntu")