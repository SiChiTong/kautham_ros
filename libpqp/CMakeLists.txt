# libPQP

FILE( GLOB SRC_FILES *.cpp )
FILE( GLOB HDR_FILES *.h )

ADD_LIBRARY( libPQP ${SRC_FILES} ${HDR_FILES} )

IF(UNIX)
  SET_TARGET_PROPERTIES( libPQP PROPERTIES OUTPUT_NAME PQP )
ENDIF(UNIX)

# add_library(libPQP 
		# Build.h
		# BV.h
		# MatVec.h
		# PQP.cpp 
		# PQP_Compile.h
		# RectDist.h
		# TriDist.cpp
		# Build.cpp
		# BV.cpp
		# BVTQ.h
		# GetTime.h
		# OBB_Disjoint.h
		# PQP.h
		# PQP_Internal.h
		# Tri.h
		# TriDist.h)


###########
# Install #
###########
SET(INSTALL_DIR_RAW ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION})
install(TARGETS libPQP
  ARCHIVE DESTINATION ${INSTALL_DIR_RAW}
  LIBRARY DESTINATION ${INSTALL_DIR_RAW}
  RUNTIME DESTINATION ${INSTALL_DIR_RAW}
)