# this file makes the libplanner library.

INCLUDE_DIRECTORIES( ${mt_DIR} 
                     ${Boost_INCLUDE_DIRS} 
                     ${CMAKE_CURRENT_SOURCE_DIR}/..
                     ${util_DIR}
                     ${problem_DIR}
                     ${COIN_INCLUDE_DIR}
                     ${Armadillo_INCLUDE_DIR} )
                     
FILE( GLOB SRC_FILES *.cpp )
FILE( GLOB HDR_FILES *.h )

ADD_DEFINITIONS( -DCOIN_DLL )

ADD_LIBRARY( libplanner STATIC ${SRC_FILES} ${HDR_FILES} )
TARGET_LINK_LIBRARIES( libplanner libproblem libsampling libkthutil ${COIN_LIBS}) 

IF(UNIX)
  SET_TARGET_PROPERTIES( libplanner PROPERTIES OUTPUT_NAME planner )
ENDIF(UNIX)

###########
# Install #
###########
SET(INSTALL_DIR_RAW ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION})
install(TARGETS libplanner
  ARCHIVE DESTINATION ${INSTALL_DIR_RAW}
  LIBRARY DESTINATION ${INSTALL_DIR_RAW}
  RUNTIME DESTINATION ${INSTALL_DIR_RAW}
)