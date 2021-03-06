# This CMakeFile is used con create libproblem_pov(.lib / .a)

INCLUDE_DIRECTORIES( ${hardlab_DIR}
                     ${util_DIR}
                     ${CMAKE_CURRENT_SOURCE_DIR}/..
                     ${mt_DIR} 
                     ${Boost_INCLUDE_DIRS}
                     ${COIN_INCLUDE_DIR}
                     ${Armadillo_INCLUDE_DIR} )

ADD_DEFINITIONS( -DCOIN_DLL )
ADD_DEFINITIONS( -DKAUTHAM_COLLISION_PQP ) # Remove it if you don't want to create a PQP collision models.

FILE( GLOB SRC_FILES *.cpp )
FILE( GLOB HDR_FILES *.h )

SET( SRC_FILES ${SRC_FILES} ${hardlab_DIR}/robot/txrobot.cpp ${hardlab_DIR}/robot/txrobot.h )

IF( GSL_FOUND )
    INCLUDE_DIRECTORIES( ${GSL_INCLUDE_DIR} )
    ADD_DEFINITIONS( -DKAUTHAM_USE_GSL )
    SET( EXTRA_LIBS ${GSL_LIBRARIES} ${GSL_CBLAS_LIBRARIES} )
ENDIF( GSL_FOUND )

ADD_LIBRARY(libproblem_pov STATIC ${SRC_FILES} ${HDR_FILES})
TARGET_LINK_LIBRARIES( libproblem_pov libPQP libsampling libkthutil libplanner ${BOOST_LIBRARIES} ${COIN_LIBRARY} ${EXTRA_LIBS} )

IF(UNIX)
 SET_TARGET_PROPERTIES( libproblem_pov PROPERTIES OUTPUT_NAME problem )
ENDIF(UNIX)

###########
# Install #
###########
SET(INSTALL_DIR_RAW ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION})
install(TARGETS libproblem_pov
  ARCHIVE DESTINATION ${INSTALL_DIR_RAW}
  LIBRARY DESTINATION ${INSTALL_DIR_RAW}
  RUNTIME DESTINATION ${INSTALL_DIR_RAW}
)